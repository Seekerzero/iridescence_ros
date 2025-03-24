#pragma once
#include "RobotControlMode.hpp"

namespace irtconfig{
    
    struct MainMenuPanelConfig{
        RobotControlMode robot_control_mode = MANUAL;
    };
}