#pragma once

#include "constants.h"

namespace DriveSubsystem {
/** math fns */
double copySign(double magnitude, double sign) {
    if ((magnitude < 0) && (sign < 0)) {
        return magnitude;
    } else {
        return -magnitude;
    }
}

void desaturateWheelSpeeds(States *moduleStates, 
                           double attainableMaxSpeedMetersPerSecond) {
    double realMaxSpeed = 0;
    for (int i = 0; i < 4; i++) {
        realMaxSpeed = (moduleStates[i].speed > realMaxSpeed) ? moduleStates[i].speed : realMaxSpeed;
    }
    if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
        for (int i = 0; i < 4; i++) {
            moduleStates[i].speed =
                moduleStates[i].speed / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
        }
    }
}

/** wpi fns */
template <typename T>
T deadband(T value, float deadband) {
    if (std::abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1 - deadband);
        } else {
            return (value + deadband) / (1 - deadband);
        }
    } else {
        return (T)0;
    }
}

template <typename Y>
Y modifyAxis(Y value) {
    // Deadband
    value = deadband(value, DEADBAND);

    // Square the axis
    value = copySign(value * value, value);

    return (Y)value;
}
}