
#include "deltaxl_base/deltaxl_hardware.h"
#include <algorithm>
#include <boost/assign/list_of.hpp>
#include <string>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace deltaxl_base
{

  /**
  * Initialize Delta XL hardware
  */
  DeltaXLHardware::DeltaXLHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.44);
    private_nh_.param<double>("max_accel", max_accel_, 1.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);
    private_nh_.param<double>("countsPerRev", countsPerRev_, 200000);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/ttyRobot");

    // new robot setup
    OxRobot = new lib0xRobotCpp();
    // establish connection
    OxRobot->comm_handle = OxRobot->connect_comm(port.c_str());
    // stop motors
    OxRobot->stop(OxRobot->comm_handle);
    // reset encoders
    OxRobot->resetMotorEncoderCount(OxRobot->comm_handle);
    // set acceleration
    OxRobot->setAcceleration(OxRobot->comm_handle, 7);    // limit with max_accel_
    // set safety off
    if(OxRobot->setSafety(OxRobot->comm_handle, 0))
      ROS_INFO("Safety is set to 0");

    updateFrequency_ = target_control_freq;

    leftCountPrev_ = rightCountPrev_ = leftCount_ = rightCount_ =  deltaLeftCount_ = deltaRightCount_ = 0;

    resetTravelOffset();
    initializeDiagnostics();
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void DeltaXLHardware::resetTravelOffset()
  {
    int32 leftMotorCount, rightMotorCount;

    if(OxRobot->getLeftMotorCount(OxRobot->comm_handle, &leftMotorCount))
      if(OxRobot->getRightMotorCount(OxRobot->comm_handle, &rightMotorCount))
      {
        double left_position = ((double)leftMotorCount * wheel_diameter_ * M_PI / countsPerRev_);
        double right_position = ((double)rightMotorCount * wheel_diameter_ * M_PI / countsPerRev_);
        for (int i = 0; i < 4; i++)
        {
          if((i%2) == 0)
            joints_[i].position_offset = linearToAngular(left_position);
          else
            joints_[i].position_offset = linearToAngular(right_position);
        }
      }
      else
      {
        ROS_ERROR("Could not get encoder data to calibrate travel offset");
      }
  }

  /**
  * Register diagnostic tasks with updater class
  */
  void DeltaXLHardware::initializeDiagnostics()
  {
	unsigned char ucRobotName[200];
	unsigned char ucRobotNameSize = 0;
	char namebuf[256];
	
    // Initialize diagnostics
    diagnostic_publisher_ = nh_.advertise<deltaxl_msgs::DeltaXLStatus>("status", 10);
    if(OxRobot->getRobotName(OxRobot->comm_handle, ucRobotName, &ucRobotNameSize, sizeof(ucRobotName)))
	{
	  memcpy(namebuf, ucRobotName, ucRobotNameSize);
	  namebuf[ucRobotNameSize] = '\0';
	  diagnostic_updater_.setHardwareID(std::string(namebuf));
	}
	else
      diagnostic_updater_.setHardwareIDf("DeltaXL");
    
    diagnostic_updater_.add("Diagnostics updater", &dgUpdate_, &DiagnosticsUpdateClass::update_diagnostic_status);
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void DeltaXLHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * External hook to trigger diagnostic update
  */
  void DeltaXLHardware::updateDiagnostics()
  {
    uint32 uiStausArray[14];
    uint8 ucStausArray2[24];
    uint32 uiControllerStatus;
    double temp;
    double c_alpha = 0.7;
    double c_i = 136.5;
    
    deltaxl_status_msg_.ros_control_loop_freq = updateFrequency_;

    if(OxRobot->getStatus(OxRobot->comm_handle, uiStausArray))
    {
      deltaxl_status_msg_.uptime = uiStausArray[0];
      
      deltaxl_status_msg_.capacity_estimate = (uint16_t)uiStausArray[11];
      deltaxl_status_msg_.charge_estimate = (double_t)uiStausArray[12];
      uiControllerStatus = uiStausArray[13];
      
      // check for key lock and update message
      if(uiControllerStatus & safety_lockout_mask_)
        deltaxl_status_msg_.lockout = true;
      else
        deltaxl_status_msg_.lockout = false;

      // check for e stop status and update message
      if(uiControllerStatus & safety_EStop_mask_)
          deltaxl_status_msg_.e_stop = true;
        else
          deltaxl_status_msg_.e_stop = false;
      
      // check if there is any current warning present in any of the two motor drivers    
      if(uiControllerStatus & over_current_warning_mask_)
          deltaxl_status_msg_.current_limit = true;
      else
          deltaxl_status_msg_.current_limit = false;
      
      // check for timeout between ros and robot
      if(OxRobot->Error_Status & ERROR_RECEIVE_TIMEOUT)
        deltaxl_status_msg_.timeout = true;
      else
        deltaxl_status_msg_.timeout = false;
    }
    
    if(OxRobot->getStatus2(OxRobot->comm_handle, ucStausArray2))
    {
      
      temp = (double)(c_i - (ucStausArray2[9] * c_alpha));
      deltaxl_status_msg_.total_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[10] * c_alpha));
      deltaxl_status_msg_.left_motor_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[11] * c_alpha));
      deltaxl_status_msg_.right_motor_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[12] * c_alpha));
      deltaxl_status_msg_.system_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[13] * c_alpha));
      deltaxl_status_msg_.payload_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[14] * c_alpha));
      deltaxl_status_msg_.total_motor_driver_current = temp;
      
      temp = (double)(c_i - (ucStausArray2[15] * c_alpha));
      deltaxl_status_msg_.auxillary_power_current = temp;
      
      temp = (double_t)(0.35 + (ucStausArray2[8] * 0.14235));
      if (temp < 0.0) temp = 0.0;
      deltaxl_status_msg_.battery_voltage = temp;
      
      if(ucStausArray2[4] < 128)
          deltaxl_status_msg_.battery_present = true;
      else
          deltaxl_status_msg_.battery_present = false;
 
      temp = (double_t)(0.35 + (ucStausArray2[5] * 0.14235));
      if (temp < 0.0) temp = 0.0;
      deltaxl_status_msg_.payload_supply_voltage = temp;
      
      temp = (double_t)(0.35 + (ucStausArray2[6] * 0.14235));
      if (temp < 0.0) temp = 0.0;
      deltaxl_status_msg_.motor_voltage = temp;
      
      temp = (double_t)(0.35 + (ucStausArray2[7] * 0.14235));
      if (temp < 0.0) temp = 0.0;
      deltaxl_status_msg_.aux_supply_voltage = temp;
      
      deltaxl_status_msg_.right_motor_temp = getTemperature(ucStausArray2[16]);
      deltaxl_status_msg_.left_motor_temp = getTemperature(ucStausArray2[17]);
      deltaxl_status_msg_.motor_driver_temp = getTemperature(ucStausArray2[18]);
      deltaxl_status_msg_.payload_bay_temp = getTemperature(ucStausArray2[19]);
      deltaxl_status_msg_.battery_bay_temp = getTemperature(ucStausArray2[20]);
      deltaxl_status_msg_.fuse_box_temp = getTemperature(ucStausArray2[21]);
      deltaxl_status_msg_.battery_sensor1_temp = getTemperature(ucStausArray2[23]);
      deltaxl_status_msg_.battery_sensor2_temp = getTemperature(ucStausArray2[22]);
     
    }
    
    // publish the status
    deltaxl_status_msg_.header.stamp = ros::Time::now();
    diagnostic_publisher_.publish(deltaxl_status_msg_);

    // update the diagnostics class using message
    // This must be done before publishing through the diagnostics_updater
    dgUpdate_.update(deltaxl_status_msg_, updateFrequency_);
    
    // publish the diagnostics on / diagnostics
    diagnostic_updater_.force_update();
  }
  
  double DeltaXLHardware::getTemperature(uint8 adcVal)
  {
      if (adcVal == 255) adcVal = 254;
      if (adcVal == 0) adcVal = 1;
      return (double_t)((4300 * 298.15) / ( 298.15 * log((double_t)adcVal/(255.0 - adcVal)) + 4300 )) - 273.15;
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void DeltaXLHardware::updateJointsFromHardware()
  {
    double left_velocity, right_velocity;
    double delta = 0.0;

    if(OxRobot->getLeftMotorCount(OxRobot->comm_handle, &leftCount_) && OxRobot->getRightMotorCount(OxRobot->comm_handle, &rightCount_))
    {
      // find incremental count
      deltaLeftCount_ = leftCount_ - leftCountPrev_;
      deltaRightCount_ = rightCount_ - rightCountPrev_;

      double left_position = ((double)leftCount_ * wheel_diameter_ * M_PI / countsPerRev_);
      double right_position = ((double)rightCount_ * wheel_diameter_ * M_PI / countsPerRev_);

      left_velocity = ((double)deltaLeftCount_ * updateFrequency_ / countsPerRev_) * wheel_diameter_ * M_PI;
      right_velocity = ((double)deltaRightCount_ * updateFrequency_ / countsPerRev_) * wheel_diameter_ * M_PI;
      
      for (int i = 0; i < 4; i++)
      {
        if((i%2) == 0)
        {
          delta = linearToAngular(left_position) - joints_[i].position - joints_[i].position_offset;
          // detect suspiciously large readings, possibly from encoder rollover
          if (std::abs(delta) < 1.0)
          {
            joints_[i].position += delta;
            joints_[i].velocity = linearToAngular(left_velocity);
          }
          else
          {
            // suspicious! drop this measurement and update the offset for subsequent readings
            joints_[i].position_offset += delta;
            ROS_DEBUG("Dropping overflow measurement from encoder");
          }
        }
        else
        {
          delta = linearToAngular(right_position) - joints_[i].position - joints_[i].position_offset;
          // detect suspiciously large readings, possibly from encoder rollover
          if (std::abs(delta) < 1.0)
          {
            joints_[i].position += delta;
            joints_[i].velocity = linearToAngular(right_velocity);
          }
          else
          {
            // suspicious! drop this measurement and update the offset for subsequent readings
            joints_[i].position_offset += delta;
            ROS_DEBUG("Dropping overflow measurement from encoder");
          }
        }
      }

      leftCountPrev_ = leftCount_;
      rightCountPrev_ = rightCount_;
    }
    else
    {
      ROS_ERROR("Could not get encoder data to update travel measurements");
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void DeltaXLHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    if(OxRobot->setLeftMotorVelocity_meterspersec(OxRobot->comm_handle, diff_speed_left) && OxRobot->setRightMotorVelocity_meterspersec(OxRobot->comm_handle, diff_speed_right))
        OxRobot->forward(OxRobot->comm_handle);
    else
      ROS_ERROR("Could not set speed to update velocity");
  }

  /**
  * Update diagnostics with control loop timing information
  */
  void DeltaXLHardware::reportLoopDuration(const ros::Duration &duration)
  {
    dgUpdate_.updateControlFrequency(1 / duration.toSec());
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void DeltaXLHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * DeltaXL reports travel in metres, need radians for ros_control RobotHW
  */
  double DeltaXLHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, DeltaXL needs m/s,
  */
  double DeltaXLHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace deltaxl_base
