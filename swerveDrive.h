#pragma once

#include <cmath>

#include "constants.h"

namespace SwerveDrive {
void fromFieldRelativeSpeeds(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        double robotAngle) {
    D.xIntermediate = vxMetersPerSecond * std::cos(robotAngle) + vyMetersPerSecond * std::sin(robotAngle);
    D.yIntermediate = vyMetersPerSecond = -vxMetersPerSecond * std::sin(robotAngle) + vyMetersPerSecond * std::cos(robotAngle);
    D.thetaIntermediate = omegaRadiansPerSecond;
}
}