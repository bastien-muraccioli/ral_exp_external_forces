#include "RALExpController_VelLimitPose.h"

#include <RALExpController/RALExpController.h>

void RALExpController_VelLimitPose::configure(const mc_rtc::Configuration & config) {}

void RALExpController_VelLimitPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.5, false, true));
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);

  // Deactivate feedback from external forces estimator (safer)
  if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.1);
  ctl.compPostureTask->target(ctl.postureVelLimit);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_VelLimitPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  if(ctl.compPostureTask->eval().norm() < 0.01)
  {
    if(ctl.sequenceOutput.compare("FINISHED") == 0)
    {
      ctl.sequenceOutput = "B";
      ctl.velLimitCounter = 0;
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpController_VelLimitPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_VelLimitPose", RALExpController_VelLimitPose)
