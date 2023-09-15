#include "RALExpController_JointLimNonComp.h"

#include <RALExpController/RALExpController.h>

void RALExpController_JointLimNonComp::configure(const mc_rtc::Configuration & config) {}

void RALExpController_JointLimNonComp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.6, false, true));
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);

  // Deactivate feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
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
  ctl.compPostureTask->stiffness(10.0);
  ctl.compPostureTask->target(ctl.postureJointLim);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  elapsedTime_ = 0;
  ctl.jointLimitCounter++;

  jointRef_ = ctl.postureJointLim["joint_4"][0];

  ctl.logger().addLogEntry("JointLimit_ref_pose", [this]() { return this->jointRef_; });

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_JointLimNonComp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  elapsedTime_ += ctl.timeStep;

  if(elapsedTime_ >= ctl.jointLimitDuration)
  {
    if(ctl.jointLimitCounter > ctl.jointLimitCount)
    {
      ctl.sequenceOutput = "FINISHED";
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpController_JointLimNonComp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_JointLimNonComp", RALExpController_JointLimNonComp)
