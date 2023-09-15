#include "RALExpController_VelLimitEF.h"

#include <mc_tvm/Robot.h>
#include <RALExpController/RALExpController.h>

void RALExpController_VelLimitEF::configure(const mc_rtc::Configuration & config) {}

void RALExpController_VelLimitEF::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.5, false, true));
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
  ctl.compPostureTask->target(ctl.postureVelLimit);
  ctl.compPostureTask->makeCompliant(true);
  ctl.solver().removeTask(ctl.compEETask);

  elapsedTime_ = 0;
  ctl.velLimitCounter++;

  jointVel = 0.0;
  upperLimit = 0.6 * ctl.robot().tvmRobot().limits().vu[3];
  lowerLimit = 0.6 * ctl.robot().tvmRobot().limits().vl[3];
  maxLimitCross_ = 0.0;

  ctl.logger().addLogEntry("VelLimit_Ef_limit_violated", [this]() {
    double ret;
    if(jointVel > upperLimit)
    {
      ret = 1.0;
    }
    else if(jointVel < lowerLimit)
    {
      ret = 1.0;
    }
    else
    {
      ret = 0.0;
    }
    return ret;
  });

  ctl.logger().addLogEntry("VelLimit_Ef_velocity", [this]() { return this->jointVel; });
  ctl.logger().addLogEntry("VelLimit_Ef_upperLimit", [this]() { return this->upperLimit; });
  ctl.logger().addLogEntry("VelLimit_Ef_lowerLimit", [this]() { return this->lowerLimit; });
  ctl.logger().addLogEntry("VelLimit_Ef_maxLimitCross", [this]() { return this->maxLimitCross_; });

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_VelLimitEF::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  jointVel = ctl.realRobot().encoderVelocities()[3];

  if(jointVel < maxLimitCross_)
  {
    maxLimitCross_ = jointVel;
  }

  elapsedTime_ += ctl.timeStep;

  if(elapsedTime_ >= ctl.velLimitDuration)
  {
    if(ctl.velLimitCounter > ctl.velLimitCount)
    {
      ctl.sequenceOutput = "FINISHED";
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpController_VelLimitEF::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  ctl.logger().removeLogEntry("VelLimit_Ef_limit_violated");
  ctl.logger().removeLogEntry("VelLimit_Ef_velocity");
  ctl.logger().removeLogEntry("VelLimit_Ef_upperLimit");
  ctl.logger().removeLogEntry("VelLimit_Ef_lowerLimit");
  ctl.logger().removeLogEntry("VelLimit_Ef_maxLimitCross");
}

EXPORT_SINGLE_STATE("RALExpController_VelLimitEF", RALExpController_VelLimitEF)
