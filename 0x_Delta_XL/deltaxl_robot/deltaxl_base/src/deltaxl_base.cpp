
#include "deltaxl_base/deltaxl_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Deltaxl, not realtime safe
*/
void controlLoop(deltaxl_base::DeltaXLHardware &Deltaxl,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  Deltaxl.reportLoopDuration(elapsed);
  Deltaxl.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  Deltaxl.writeCommandsToHardware();
}

/**
* Diagnostics loop for Deltaxl, not realtime safe
*/
void diagnosticLoop(deltaxl_base::DeltaXLHardware &Deltaxl)
{
  Deltaxl.updateDiagnostics();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "deltaxl_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  deltaxl_base::DeltaXLHardware Deltaxl(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&Deltaxl, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Deltaxl hardware - lib0xRobotCPP is not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue Deltaxl_queue;
  ros::AsyncSpinner Deltaxl_spinner(1, &Deltaxl_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(Deltaxl), boost::ref(cm), boost::ref(last_time)),
    &Deltaxl_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(
    ros::Duration(1 / diagnostic_frequency),
    boost::bind(diagnosticLoop, boost::ref(Deltaxl)),
    &Deltaxl_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  Deltaxl_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
