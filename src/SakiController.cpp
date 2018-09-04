#include "SakiController.h"
#include <iostream>

SakiController::SakiController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  LOG_SUCCESS("SakiController init done " << this)
}

bool SakiController::run()
{
  //std::cout << "waiting for the preparation" << std::endl;
  bool ret =  mc_control::MCController::run();
  //std::cout << timeStep << std::endl;
  if(running == false) {
    return ret;
  }
  if(running_count%1500==0){
    if(up){
      ef_->set_ef_pose({
          Eigen::Quaterniond(0.56, -0.46, 0.55, -0.44),
            Eigen::Vector3d(0.21, -0.24, 0.97)});
    }
    else{
      ef_->set_ef_pose({
          Eigen::Quaterniond(0.77, 0, 0, -0.04),
            Eigen::Vector3d(0.24, -0.25, 0.97)});
    }
    up = !up;
  }
  running_count++;
  if(running_count > 3000) {
    running = false;
    running_count = 0;
    up = true;
  }
  
  return ret;
}


void SakiController::reset(const mc_control::ControllerResetData & reset_data)
{

  mc_control::MCController::reset(reset_data);
  solver().setContacts({
      {robots(), "LFullSole", "AllGround"},
        {robots(), "RFullSole", "AllGround"}
    });
  com_ = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
  solver().addTask(com_);
  ef_ = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK6", robots(), 0);
  solver().addTask(ef_);
  ef_->selectActiveJoints(solver(), {
      "RARM_JOINT0",
        "RARM_JOINT1",
        "RARM_JOINT2",
        "RARM_JOINT3",
        "RARM_JOINT4",
        "RARM_JOINT5",
        "RARM_JOINT6"}); 
    std::cout << "set_ef_pose start" << std::endl;
  ef_->set_ef_pose({
      Eigen::Quaterniond(0.56, -0.46, 0.55, -0.44),
            Eigen::Vector3d(0.21, -0.24, 0.97)});
  std::cout << "set_ef_pose end" << std::endl;

  gui_->addElement({"SakiController"},
                   mc_rtc::gui::Button("Run",
                                       [this]() {this->running = true;}));
  running = false;
  running_count = 0;
  up = true;
}
CONTROLLER_CONSTRUCTOR("SakiController", SakiController)

  ////////////////////
  // From Test code //
  ////////////////////
  // MCController::reset(reset_data);
  // /* Reset the task to the current end-effector position */
  // efTask->reset();
  // comTask->reset();
  // /* Move the end-effector 10cm forward, 10 cm to the right and 10 cm upward */
  // efTask->set_ef_pose(sva::PTransformd(sva::RotY<double>(-M_PI/2),
  //                                      efTask->get_ef_pose().translation() + Eigen::Vector3d(0.3, -0.1, 0.2)));
