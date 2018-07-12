#include "SakiController.h"


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
  return mc_control::MCController::run();
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

 ef_->set_ef_pose({
                    Eigen::Quaterniond(0.56, -0.46, 0.54, -0.43),
                    Eigen::Vector3d(0.15, -0.23, 0.73)});

 /*ef_->set_ef_pose({
                    Eigen::Quaterniond(0.62, -0.32, 0.70, -0.16),
                    Eigen::Vector3d(0.65, -0.15, 0.76)});*/
}
CONTROLLER_CONSTRUCTOR("SakiController", SakiController)


	/*printf("まっています\n");
	sleep(30);
	printf("待ちました\n");
	return 0;*/
 /* ef_->set_ef_pose({
                    Eigen::Quaterniond(0.44, -0.53, 0.58, -0.42),
                    Eigen::Vector3d(0.70, -0.25, 0.74)});*/

