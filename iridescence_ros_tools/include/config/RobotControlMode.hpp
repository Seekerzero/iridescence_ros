#pragma once

namespace irtconfig{

    enum RobotControlMode
    {
        MANUAL = 0,
        AUTO = 1,
    };

    static std::string robotControlModeToString(RobotControlMode mode)
    {
        switch (mode)
        {
        case MANUAL:
            return "MANUAL";
        case AUTO:
            return "AUTO";
        default:
            return "UNKNOWN";
        }
    }
}