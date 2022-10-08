#pragma once

#include "constants.h"

/** math fns */
double copySign(double magnitude, double sign) {
    if ((magnitude < 0) && (sign < 0)) {
        return magnitude;
    } else {
        return -magnitude;
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