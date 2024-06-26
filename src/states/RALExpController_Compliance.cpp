#include "RALExpController_Compliance.h"

#include <RALExpController/RALExpController.h>

void RALExpController_Compliance::configure(const mc_rtc::Configuration & config) {}

void RALExpController_Compliance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(4.0);
  ctl.compPostureTask->weight(1);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->positionTask->stiffness(10);
  ctl.compEETask->positionTask->position(ctl.taskPosition_);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(10);
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
  ctl.solver().addTask(ctl.compEETask);

  ctl.compPostureTask->makeCompliant(true);

  ctl.waitingForInput = true;

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_Compliance::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  if(not ctl.waitingForInput)
  {
    output("OK");
    return true;
  }

  return false;
}

void RALExpController_Compliance::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_Compliance", RALExpController_Compliance)
