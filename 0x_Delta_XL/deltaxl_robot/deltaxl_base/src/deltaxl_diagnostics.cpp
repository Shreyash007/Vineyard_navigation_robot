
#include "deltaxl_base/deltaxl_diagnostics.h"
#include <algorithm>
#include <limits>

namespace
{
  const int UNDERVOLT_ERROR = 19;
  const int UNDERVOLT_WARN = 20;
  const int OVERVOLT_ERROR = 28;
  const int OVERVOLT_WARN = 26;
  const int OVERCURRENT_TOTAL_ERROR = 100;
  const int OVERCURRENT_TOTAL_WARN = 80;
  const int OVERCURRENT_MOTOR_DRIVER_ERROR = 100;
  const int OVERCURRENT_MOTOR_DRIVER_WARN = 80;
  const int OVERCURRENT_MOTOR_ERROR = 55;
  const int OVERCURRENT_MOTOR_WARN = 45;
  const int OVERCURRENT_SYSTEM_ERROR = 10;
  const int OVERCURRENT_SYSTEM_WARN = 8;
  const int OVERCURRENT_PAYLOAD_ERROR = 10;
  const int OVERCURRENT_PAYLOAD_WARN = 8;
  const int OVERCURRENT_AUX_ERROR = 10;
  const int OVERCURRENT_AUX_WARN = 8;
  const int OVERTEMP_MOTOR_DRIVER_ERROR = 60;
  const int OVERTEMP_MOTOR_DRIVER_WARN = 50;
  const int OVERTEMP_MOTOR_ERROR = 80;
  const int OVERTEMP_MOTOR_WARN = 65;
  const int OVERTEMP_PAYLOAD_BAY_ERROR = 55;
  const int OVERTEMP_PAYLOAD_BAY_WARN = 45;
  const int OVERTEMP_BATTERY_BAY_ERROR = 55;
  const int OVERTEMP_BATTERY_BAY_WARN = 45;
  const int OVERTEMP_BATTERY_ERROR = 55;
  const int OVERTEMP_BATTERY_WARN = 45;
  const int OVERTEMP_FUSE_BOX_ERROR = 55;
  const int OVERTEMP_FUSE_BOX_WARN = 45;
  const int LOWPOWER_ERROR = 20;
  const int LOWPOWER_WARN = 30;
  const int CONTROLFREQ_WARN = 90;
}  // namespace

namespace deltaxl_base
{

  DiagnosticsUpdateClass::DiagnosticsUpdateClass()
  {
    
  }

  void DiagnosticsUpdateClass::update_diagnostic_status(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {

    stat.add("Uptime", msg_.uptime);
    

    stat.add("Battery Voltage", msg_.battery_voltage);
    stat.add("Motor Voltage", msg_.motor_voltage);
    stat.add("Payload Voltage", msg_.payload_supply_voltage);
    stat.add("Auxillary Power Voltage", msg_.aux_supply_voltage);

    stat.add("Total Current", msg_.total_current);
    stat.add("Total Motor Driver Current", msg_.total_motor_driver_current);
    stat.add("Left Motor Current", msg_.left_motor_current);
    stat.add("Right Motor Current", msg_.right_motor_current);
    stat.add("System Current", msg_.system_current);
    stat.add("Payload Current", msg_.payload_current);
    stat.add("Auxillary Power Current", msg_.auxillary_power_current);

    stat.add("Motor Driver Temp (C)", msg_.motor_driver_temp);
    stat.add("Left Motor Temp (C)", msg_.left_motor_temp);
    stat.add("Right Motor Temp (C)", msg_.right_motor_temp);
    stat.add("Payload Bay Temp (C)", msg_.payload_bay_temp);
    stat.add("Battery Bay Temp (C)", msg_.battery_bay_temp);
    stat.add("Battery Sensor 1 Temp (C)", msg_.battery_sensor1_temp);
    stat.add("Battery Sensor 2 Temp (C)", msg_.battery_sensor2_temp);
    stat.add("Fuse Box Temp (C)", msg_.fuse_box_temp);

    stat.add("Charge (%)", msg_.charge_estimate);
    stat.add("Battery Capacity (Wh)", msg_.capacity_estimate);

    stat.add("Timeout", static_cast<bool>(msg_.timeout));
    stat.add("Lockout", static_cast<bool>(msg_.lockout));
    stat.add("Emergency Stop", static_cast<bool>(msg_.e_stop));
    stat.add("Current limit", static_cast<bool>(msg_.current_limit));
    stat.add("Battery Present in Bay", static_cast<bool>(msg_.battery_present));

    stat.add("ROS control loop frequency", msg_.ros_control_loop_freq);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System Status OK");
    
    // Voltage related errors and warnings
    if (msg_.battery_voltage > OVERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery voltage too high");
    }
    else if (msg_.battery_voltage > OVERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main battery voltage too high");
    }
    else if (msg_.battery_voltage < UNDERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery voltage too low");
    }
    else if (msg_.battery_voltage < UNDERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main battery voltage too low");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Main battery voltage OK");
    }
    
    if (msg_.motor_voltage > OVERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor voltage too high");
    }
    else if (msg_.motor_voltage > OVERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor voltage too high");
    }
    else if (msg_.motor_voltage < UNDERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor voltage too low");
    }
    else if (msg_.motor_voltage < UNDERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor voltage too low");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Motor voltage OK");
    }
    
    if (msg_.payload_supply_voltage > OVERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Payload voltage too high");
    }
    else if (msg_.payload_supply_voltage > OVERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Payload voltage too high");
    }
    else if (msg_.payload_supply_voltage < UNDERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Payload voltage too low");
    }
    else if (msg_.payload_supply_voltage < UNDERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Payload voltage too low");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Payload voltage OK");
    }
    
    if (msg_.aux_supply_voltage > OVERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Aux power voltage too high");
    }
    else if (msg_.aux_supply_voltage > OVERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Aux power voltage too high");
    }
    else if (msg_.aux_supply_voltage < UNDERVOLT_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Aux power voltage too low");
    }
    else if (msg_.aux_supply_voltage < UNDERVOLT_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Aux power voltage too low");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Aux power voltage OK");
    }
    
    // Current related errors and warnings
    if (msg_.total_current > OVERCURRENT_TOTAL_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Total current is over 80 Amp");
    }
    else if (msg_.total_current > OVERCURRENT_TOTAL_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Total current is over 60 Amp");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Total current OK");
    }
    
    if (msg_.total_motor_driver_current > OVERCURRENT_MOTOR_DRIVER_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor Driver Over current Error");
    }
    else if (msg_.total_motor_driver_current > OVERCURRENT_MOTOR_DRIVER_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor Driver Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Driver current OK");
    }
    
    if (msg_.left_motor_current > OVERCURRENT_MOTOR_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Left Motor Over current Error");
    }
    else if (msg_.left_motor_current > OVERCURRENT_MOTOR_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Left Motor Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Left Motor current OK");
    }
    
    if (msg_.right_motor_current > OVERCURRENT_MOTOR_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Right Motor Over current Error");
    }
    else if (msg_.right_motor_current > OVERCURRENT_MOTOR_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Right Motor Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Right Motor current OK");
    }
    
    if (msg_.system_current > OVERCURRENT_SYSTEM_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "System Over current Error");
    }
    else if (msg_.system_current > OVERCURRENT_SYSTEM_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "System Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "System current OK");
    }
    
    if (msg_.payload_current > OVERCURRENT_PAYLOAD_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Payload Power Over current Error");
    }
    else if (msg_.payload_current > OVERCURRENT_PAYLOAD_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Payload Power Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Payload Power current OK");
    }
    
    if (msg_.auxillary_power_current > OVERCURRENT_AUX_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Auxillary Power Over current Error");
    }
    else if (msg_.auxillary_power_current > OVERCURRENT_AUX_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Auxillary Power Over current Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Auxillary Power current OK");
    }

    // Temperature errors and warnings
    if (msg_.motor_driver_temp > OVERTEMP_MOTOR_DRIVER_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor driver over temperature Error");
    }
    else if (msg_.motor_driver_temp > OVERTEMP_MOTOR_DRIVER_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor driver over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Driver temperature OK");
    }
    
    if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > OVERTEMP_MOTOR_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motors over temperature Error");
    }
    else if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > OVERTEMP_MOTOR_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motors over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Motor temperature OK");
    }

    if (msg_.payload_bay_temp > OVERTEMP_PAYLOAD_BAY_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Payload bay over temperature Error");
    }
    else if (msg_.payload_bay_temp > OVERTEMP_PAYLOAD_BAY_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Payload bay over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Payload bay temperature OK");
    }
 
    if (msg_.battery_bay_temp > OVERTEMP_BATTERY_BAY_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery bay over temperature Error");
    }
    else if (msg_.battery_bay_temp > OVERTEMP_BATTERY_BAY_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery bay over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Battery bay temperature OK");
    }
    
    if (std::max(msg_.battery_sensor1_temp, msg_.battery_sensor2_temp) > OVERTEMP_BATTERY_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery over temperature Error");
    }
    else if (std::max(msg_.battery_sensor1_temp, msg_.battery_sensor2_temp) > OVERTEMP_BATTERY_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Battery temperature OK");
    }
    
    if (msg_.fuse_box_temp > OVERTEMP_FUSE_BOX_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Fuse box over temperature Error");
    }
    else if (msg_.fuse_box_temp > OVERTEMP_FUSE_BOX_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Fuse box over temperature Warning");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Fuse box temperature OK");
    }

    // Battery charge related errors and warnings
    if (msg_.charge_estimate < LOWPOWER_ERROR)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Low power");
    }
    else if (msg_.charge_estimate < LOWPOWER_WARN)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Low power");
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Charge OK");
    }
    
    if (msg_.battery_present == false)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery absent in battery bay");
    }

    double margin = msg_.ros_control_loop_freq / target_control_freq_ * 100;

    if (margin < CONTROLFREQ_WARN)
    {
        std::ostringstream message;
        message << "Control loop executing " << 100 - static_cast<int>(margin) << "% slower than desired";
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, message.str());
    }

  }

  void DiagnosticsUpdateClass::update(deltaxl_msgs::DeltaXLStatus &msg, double target_control_frequency)
  {
      msg_ = msg;
      target_control_freq_ = target_control_frequency;
  }

  void DiagnosticsUpdateClass::updateControlFrequency(double frequency)
  {
    // Keep minimum observed frequency for diagnostics purposes
    control_freq_ = std::min(control_freq_, frequency);
  }

}  // namespace deltaxl_base
