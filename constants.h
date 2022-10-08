#pragma once

#include <iostream>

#define DEBUG_PRINT

#define PI 3.14159265358979323846
#define TWO_PI (2 * PI)

#define DEADBAND 0.4

#define MAX_VOLTAGE 6.0

#define MAX_VELOCITY_METERS_PER_SECOND 4.968230455
#define MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND 61.2152594258

enum class Input {
    input,
    intermediate,
    final,

    finalPrint
};

struct Data {

    float xInput,
          yInput,
          thetaInput,

          xIntermediate,
          yIntermediate,
          thetaIntermediate,

          driveMotorSpeed,
          steerMotorPos;

          void print(Input value, char* fn, int loopCount = 0) {
            switch (value) {
                case Input::input:
                    std::cout << fn << '\n' <<
                        "xInput:\t\t"   << xInput     << '\n'   <<
                        "yInput:\t\t"   << yInput     << '\n'   <<
                        "thetaInput:\t" << thetaInput << "\n\n";
                    break;
                case Input::intermediate:
                    std::cout << fn << '\n' <<
                        "xIntermediate:\t\t"   << xIntermediate     << '\n' <<
                        "yIntermediate:\t\t"   << yIntermediate     << '\n' <<
                        "thetaIntermediate:\t" << thetaIntermediate << "\n\n";
                    break;
                case Input::final:
                    std::cout << fn << '\n' <<
                        "driveMotor:\t" << driveMotorSpeed << '\n' <<
                        "steerMotor:\t" << steerMotorPos   << "\n\n";
                    break;
                case Input::finalPrint:
                    std::cout <<
                        "----------  " << loopCount << "  ----------\n"<<
                        "xInput:\t\t"   << xInput          << '\n'   <<
                        "yInput:\t\t"   << yInput          << '\n'   <<
                        "thetaInput:\t" << thetaInput      << "\n\n" <<
                        "driveMotor:\t" << driveMotorSpeed << '\n'   <<
                        "steerMotor:\t" << steerMotorPos   << '\n'   <<
                        "-------------------------\n\n";
                    break;
            }
          }

} D;

struct States {
    float speed, angle;
} S;