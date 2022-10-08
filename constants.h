#pragma once

#define PI 3.14159265358979323846
#define TWO_PI (2 * PI)

#define DEADBAND 0.4

#define MAX_VOLTAGE 6.0

#define MAX_VELOCITY_METERS_PER_SECOND 4.968230455
#define MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND 61.2152594258

struct Data {
    float xInput,
          yInput,
          thetaInput,

          xFinal,
          yFinal,
          thetaFinal,

          driveMotorSpeed,
          steerMotorPos;

} D;