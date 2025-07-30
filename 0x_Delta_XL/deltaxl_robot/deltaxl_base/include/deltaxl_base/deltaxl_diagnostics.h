
#ifndef DELTAXL_BASE_DELTAXL_DIAGNOSTICS_H
#define DELTAXL_BASE_DELTAXL_DIAGNOSTICS_H

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "deltaxl_msgs/DeltaXLStatus.h"

namespace deltaxl_base
{

  class DiagnosticsUpdateClass
  {
  public:
    DiagnosticsUpdateClass();
    
    void update_diagnostic_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    
    void update(deltaxl_msgs::DeltaXLStatus &msg, double target_control_frequency);

    void updateControlFrequency(double frequency);
    
  private:
    deltaxl_msgs::DeltaXLStatus msg_;
    double control_freq_, target_control_freq_;
  };

}  // namespace deltaxl_base
#endif  // DELTAXL_BASE_DELTAXL_DIAGNOSTICS_H
