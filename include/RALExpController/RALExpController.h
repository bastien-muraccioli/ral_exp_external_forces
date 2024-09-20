#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/VirtualTorqueSensor.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_rbdyn/Collision.h>

#include <RALExpController/api.h>

#define HIGH_RESIDUAL_GAIN 10.0
#define LOW_RESIDUAL_GAIN 0.5

struct RALExpController_DLLAPI RALExpController : public mc_control::fsm::Controller
{
  RALExpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void updateConstraints(bool closeLoop);

  void updateConstraints(void);

  void updateCollisions(double m, double lambda);

  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

  bool moveNextState;
  std::map<std::string, std::vector<double>> postureVelLimit;
  std::map<std::string, std::vector<double>> postureJointLim;
  std::map<std::string, std::vector<double>> postureTarget;
  Eigen::VectorXd posture_target_log;

  int velLimitCount;
  double velLimitDuration;

  int jointLimitCount;
  double jointLimitDuration;

  int velLimitCounter;
  int jointLimitCounter;

  int currentSequence;
  std::string sequenceOutput;
  bool waitingForInput;
  Eigen::MatrixXd taskOrientation_; // Rotation Matrix
  Eigen::Vector3d taskPosition_;
  double m_;
  double lambda_;
  bool closeLoopVelocityDamper_;

private:
  mc_rtc::Configuration config_;
  std::vector<mc_rbdyn::Collision> collisions_;
  void getPostureTarget(void);
  int stateIndex_;
  bool velocityDamperFlag_;
  double dt_;
  double xsiOff_;
  const double i_ = 0.03;
  const double s_ = 0.015;
  const double d_ = 0.0;
};
