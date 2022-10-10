#pragma once

#include <iostream>

#define NDEBUG_PRINT

#define PI 3.14159265358979323846
#define TWO_PI (2 * PI)

#define DEADBAND 0.4

#define MAX_VOLTAGE 6.0

#define MAX_VELOCITY_METERS_PER_SECOND 4.968230455
#define MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND 61.2152594258

#define TICKS_PER_ROTATION 2048
#define STEER_REDUCTION ((15.0 / 32.0) * (10.0 / 60.0))

constexpr auto sensorPositionCoefficient = TWO_PI / TICKS_PER_ROTATION * STEER_REDUCTION;
constexpr auto sensorVelocityCoefficient = sensorPositionCoefficient * 10;

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

          void print(Input value, const char* fn, int loopCount = 0) {
            switch (value) {
                case Input::input:
                    std::cerr << fn << '\n' <<
                        "xInput:\t\t"   << xInput     << '\n'   <<
                        "yInput:\t\t"   << yInput     << '\n'   <<
                        "thetaInput:\t" << thetaInput << "\n\n";
                    break;
                case Input::intermediate:
                    std::cerr << fn << '\n' <<
                        "xIntermediate:\t\t"   << xIntermediate     << '\n' <<
                        "yIntermediate:\t\t"   << yIntermediate     << '\n' <<
                        "thetaIntermediate:\t" << thetaIntermediate << "\n\n";
                    break;
                case Input::final:
                    std::cerr << 
                        "----------  " << fn << "  ----------\n"   <<
                        "driveMotor:\t" << driveMotorSpeed << "\t\t\t" << (driveMotorSpeed * 100) << "%% Speed\n" <<
                        "steerMotor:\t" << steerMotorPos   << " Ticks\t\t" << toDeg() << " DEG\n" <<
                        "-------------------------\n\n";
                    break;
                case Input::finalPrint:
                    std::cerr <<
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
 private:
    float toDeg() {
        float motorDeg = steerMotorPos * 0.087890625;

        if (motorDeg <= -360) {
            motorDeg += 360;
        } else if (motorDeg >= 360) {
            motorDeg -= 360;
        }

        return motorDeg;
    }

} D;

struct States {
    float speed, angle;
} S;