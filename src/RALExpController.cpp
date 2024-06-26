#include <mc_tasks/CompliantEndEffectorTask.h>
#include <RALExpController/RALExpController.h>

RALExpController::RALExpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  taskOrientation_ = Eigen::Quaterniond(1,-1,-1,-1).normalized().toRotationMatrix();
  taskPosition_ = Eigen::Vector3d(0.68, 0.0, 0.45);

  // Setup custom dynamic constraints
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));

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

  // For usage on real robot with mc_kortex allows to switch control mode
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  // Add GUI for switching states
  gui()->addElement({"Controller"},
                    mc_rtc::gui::Label("Current state :", [this]() { return this->executor_.state(); }));

  gui()->addElement({"Controller"},
                    mc_rtc::gui::Label("Next state :", [this]() { return this->executor_.next_state(); }));

  gui()->addElement({"Controller"}, mc_rtc::gui::Button("Move to next state", [this]() { waitingForInput = false; }));

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