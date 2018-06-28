#pragma once

#include <mc_control/mc_controller.h>


struct MC_CONTROL_DLLAPI SakiController : public mc_control::MCController
{
    SakiController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
};