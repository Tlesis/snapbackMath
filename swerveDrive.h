#pragma once

#include <cmath>

#include "constants.h"

void fromFieldRelativeSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double robotAngle) {
    D.xFinal = vxMetersPerSecond * std::cos(robotAngle) + vyMetersPerSecond * std::sin(robotAngle);
    D.yFinal = vyMetersPerSecond = -vxMetersPerSecond * std::sin(robotAngle) + vyMetersPerSecond * std::cos(robotAngle);
    D.thetaFinal = omegaRadiansPerSecond;
}