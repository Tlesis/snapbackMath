#pragma once

#include <cmath>

#include "constants.h"

#define getSteerAngle() 0

double oldAngle;

namespace SDS {

void setReferenceAngle(double referenceAngleRadians, double speed, int index) {
    double currentAngleRadians = getSteerAngle() * sensorPositionCoefficient;

    double currentAngleRadiansMod = std::fmod(currentAngleRadians, TWO_PI);
    if (currentAngleRadiansMod < 0.0) {
        currentAngleRadiansMod += TWO_PI;
    }

    // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > PI) {
        adjustedReferenceAngleRadians -= TWO_PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -PI) {
        adjustedReferenceAngleRadians +=  TWO_PI;
    }

    if (adjustedReferenceAngleRadians / sensorPositionCoefficient != 0 && speed != 0) {
        D.steerMotorPos = oldAngle = adjustedReferenceAngleRadians / sensorPositionCoefficient;
    } else {
        D.steerMotorPos = oldAngle;
    }

    switch (index) {
        case 0:
            D.print(Input::final, "FL");
            break;
        case 1:
            D.print(Input::final, "FR");
            break;
        case 2:
            D.print(Input::final, "BL");
            break;
        case 3:
            D.print(Input::final, "BR");
            break;
    }

}

void set(double driveVoltage, double steerAngle, int index) {
    steerAngle = std::fmod(steerAngle, TWO_PI);
    if (steerAngle < 0.0) {
        steerAngle += TWO_PI;
    }

    double difference = steerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
    if (difference >= PI) {
        steerAngle -= TWO_PI;
    } else if (difference < -PI) {
        steerAngle += TWO_PI;
    }
    difference = steerAngle - getSteerAngle(); // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > PI / 2.0 || difference < -PI / 2.0) {
        // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
        steerAngle += PI;
        driveVoltage *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle = std::fmod(steerAngle, TWO_PI);
    if (steerAngle < 0.0) {
        steerAngle += TWO_PI;
    }

    D.driveMotorSpeed = driveVoltage / 12;
    setReferenceAngle(steerAngle, abs(D.driveMotorSpeed), index);
}
}