#pragma once

#include <mc_control/mc_controller.h>


#include <mc_control/mc_controller.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include <Tasks/QPContactConstr.h>
#include <mc_rtc/logging.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>


struct MC_CONTROL_DLLAPI SakiController : public mc_control::MCController
{
    SakiController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
    std::shared_ptr<mc_tasks::CoMTask> com_;
    std::shared_ptr<mc_tasks::EndEffectorTask> ef_;
    std::shared_ptr<mc_tasks::OrientationTask> ori_;
    //std::shared_ptr<mj_traj::mj_trajectory> trajectoryTask = nullptr;
    private:
    bool ready = true;
    bool first_traj = true;

    bool running;
    bool up;
    int running_count;
};
