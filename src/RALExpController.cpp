#include <mc_tasks/CompliantEndEffectorTask.h>
#include <RALExpController/RALExpController.h>

RALExpController::RALExpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  dt_ = dt;
  xsiOff_ = 0.0;
  m_ = 2.0;
  lambda_ = 100.0;
  //selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
  // Define a minimal set of self-collisions
  // collisions_ = {{"base_link", "spherical_wrist_1_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"shoulder_link", "spherical_wrist_1_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_1_link", "spherical_wrist_1_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_2_link", "spherical_wrist_1_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"base_link", "spherical_wrist_2_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"shoulder_link", "spherical_wrist_2_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_1_link", "spherical_wrist_2_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_2_link", "spherical_wrist_2_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"base_link", "bracelet_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"shoulder_link", "bracelet_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_1_link", "bracelet_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_2_link", "bracelet_link", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"base_link", "FT_adapter", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"shoulder_link", "FT_adapter", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_1_link", "FT_adapter", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_2_link", "FT_adapter", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"base_link", "FT_sensor_mounting", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"shoulder_link", "FT_sensor_mounting", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_1_link", "FT_sensor_mounting", i_, s_, d_, {}, {}, false, false, m_, lambda_},
  //               {"half_arm_2_link", "FT_sensor_mounting", i_, s_, d_, {}, {}, false, false, m_, lambda_}};
  
  //selfCollisionConstraint->editCollisions(solver(), collisions_);
  selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});


  taskOrientation_ = Eigen::Quaterniond(1,-1,-1,-1).normalized().toRotationMatrix();
  taskPosition_ = Eigen::Vector3d(0.68, 0.0, 0.45);

  // Setup custom dynamic constraints
  // dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
  //     new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);
  
  

  posture_target_log.setZero(robot().mb().nrJoints());

  postureTarget = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {0.54}}};

  solver().removeTask(getPostureTask(robot().name()));
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->stiffness(0.0);
  compPostureTask->damping(4.0);
  compPostureTask->target(postureTarget);
  solver().addTask(compPostureTask);

  compEETask = std::make_shared<mc_tasks::CompliantEndEffectorTask>("FT_sensor_mounting", robots(),
                                                                    robot().robotIndex(), 1.0, 10000.0);

  postureVelLimit = {{"joint_1", {1.57}}, {"joint_2", {0}}, {"joint_3", {1.57}}, {"joint_4", {-1.57}},
                     {"joint_5", {1.57}}, {"joint_6", {0}}, {"joint_7", {-1.06}}};

  postureJointLim = {{"joint_1", {0}}, {"joint_2", {1.57}}, {"joint_3", {-0.6}}, {"joint_4", {-1.57}},
                     {"joint_5", {0}}, {"joint_6", {0}},    {"joint_7", {2.08}}};

  velLimitCount = config("sequences")("velLimit")("repetitions");
  velLimitDuration = config("sequences")("velLimit")("duration");
  velLimitCounter = 0;

  jointLimitCount = config("sequences")("jointLimit")("repetitions");
  jointLimitDuration = config("sequences")("jointLimit")("duration");
  jointLimitCounter = 0;

  sequenceOutput = "A";
  waitingForInput = true;
  velocityDamperFlag_ = true;
  closeLoopVelocityDamper_ = true;

  // For usage on real robot with mc_kortex allows to switch control mode
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  // Add GUI for switching states
  gui()->addElement({"Controller"},
                    mc_rtc::gui::Label("Current state :", [this]() { return this->executor_.state(); }));

  gui()->addElement({"Controller"},
                    mc_rtc::gui::Label("Next state :", [this]() { return this->executor_.next_state(); }));

  gui()->addElement({"Controller"}, mc_rtc::gui::Button("Move to next state", [this]() { waitingForInput = false; }));

  gui()->addElement({"Controller"}, mc_rtc::gui::Checkbox("Close Loop Velocity Damper", 
                    [this]() { return velocityDamperFlag_; }, [this]() { velocityDamperFlag_ = !velocityDamperFlag_; }));

  gui()->addElement({"Controller"}, mc_rtc::gui::NumberInput("m", [this]() { return m_; },
                    [this](double m) { m_ = m; }));
  
  gui()->addElement({"Controller"}, mc_rtc::gui::NumberInput("lambda", [this]() { return lambda_; },
                    [this](double lambda) { lambda_ = lambda; }));

  gui()->addElement({"Controller"}, mc_rtc::gui::Button("SEND", [this]() { updateConstraints(); }));
  


  // Add log entries
  logger().addLogEntry("ControlMode",
                       [this]()
                       {
                         auto mode = datastore().get<std::string>("ControlMode");
                         if(mode.compare("") == 0) return 0;
                         if(mode.compare("Position") == 0) return 1;
                         if(mode.compare("Velocity") == 0) return 2;
                         if(mode.compare("Torque") == 0) return 3;
                         return 0;
                       });

  logger().addLogEntry("PostureTarget",
                       [this]()
                       {
                         this->getPostureTarget();
                         return this->posture_target_log;
                       });

  logger().addLogEntry("StateIndex", [this]() { return this->stateIndex_; });

  stateIndex_ = 0;

  mc_rtc::log::success("[RALExpController] Controller's init ... DONE ");
}

bool RALExpController::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");

  if (velocityDamperFlag_ && !closeLoopVelocityDamper_)
  {
    updateConstraints(true);
    closeLoopVelocityDamper_ = true;
  }
  else if (!velocityDamperFlag_ && closeLoopVelocityDamper_)
  {
    updateConstraints(false);
    closeLoopVelocityDamper_ = false;
  }

  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  return false;
}

void RALExpController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


void RALExpController::getPostureTarget(void)
{
  posture_target_log = rbd::dofToVector(robot().mb(), compPostureTask->posture());
}

void RALExpController::updateConstraints(bool closeLoop)
{
  if(closeLoop)
  {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
    solver().addConstraintSet(dynamicsConstraint);
    updateCollisions(m_, lambda_);

    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is enabled");
  }
  else
  {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, dt_, {0.1, 0.01, 0.5}, 0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    updateCollisions(0.0, 0.0);

    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is deactivated");
  }
}

void RALExpController::updateConstraints(void)
{
    if(m_ < 1.0 || lambda_ < 1.0)
    {
      solver().removeConstraintSet(dynamicsConstraint);
      dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, dt_, {0.1, 0.01, 0.5}, 0.9, false, true));
      solver().addConstraintSet(dynamicsConstraint);
      updateCollisions(0.0, 0.0);
      velocityDamperFlag_ = false;
      closeLoopVelocityDamper_ = false;
    }
    else // Close loop velocity damper
    {
      solver().removeConstraintSet(dynamicsConstraint);
      dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
      solver().addConstraintSet(dynamicsConstraint);
      updateCollisions(m_, lambda_);
      velocityDamperFlag_ = true;
      closeLoopVelocityDamper_ = true;
    }
    mc_rtc::log::info("[RALExpController] Constraints updated");
}

void RALExpController::updateCollisions(double m, double lambda)
{
  // for(auto & col : collisions_)
  // {
  //   col.overDamping = m;
  //   col.lambda = lambda;
  // }
  //selfCollisionConstraint->editCollisions(solver(), collisions_);
  selfCollisionConstraint->setCollisionsDampers(solver(), {m, lambda});
}