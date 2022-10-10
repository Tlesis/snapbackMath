#include <iostream>
#include <cmath>
#include <array>

/** constants **/
#define NDEBUG_PRINT

#define PI 3.14159265358979323846
#define TWO_PI (2 * PI)

#define DEADBAND 0.4

#define MAX_VOLTAGE 6.0

#define MAX_VELOCITY_METERS_PER_SECOND 4.968230455
#define MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND 61.2152594258

#define TICKS_PER_ROTATION 2048
#define STEER_REDUCTION ((15.0 / 32.0) * (10.0 / 60.0))

#define getSteerAngle() 0
#define LOOP_AMOUNT INFINITY

constexpr auto sensorPositionCoefficient = TWO_PI / TICKS_PER_ROTATION * STEER_REDUCTION;
constexpr auto sensorVelocityCoefficient = sensorPositionCoefficient * 10;

enum class Input {
    input,
    intermediate,
    final
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

          void print(Input value, const char* fn) {
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
                        "driveMotor:\t" << driveMotorSpeed << "\t\t\t" << (driveMotorSpeed * 100) << "\% Speed\n" <<
                        "steerMotor:\t" << steerMotorPos   << " Ticks\t\t" << toDeg() << " DEG\n" <<
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

/** driveSub */
namespace DriveSubsystem {
/** math fns */
double copySign(double magnitude, double sign) {
    if ((magnitude < 0) && (sign < 0)) {
        return magnitude;
    } else {
        return -magnitude;
    }
}

void desaturateWheelSpeeds(States moduleStates[],
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

/** swerveDrive */
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

/** moduleFactory */
namespace SDS {

double oldAngle;

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

/** mat */
namespace Matrix {
const float DRIVE_KINEMATICS[8][3] =
    {{1, 0, -0.25},
     {0, 1,  0.3246},
     {1, 0,  0.25},
     {0, 1,  0.3246},
     {1, 0, -0.25},
     {0, 1, -0.3246},
     {1, 0,  0.25},
     {0, 1, -0.3246}};

std::array<float, 8> mult(float B[3]) {
    std::array<float, 8> C =
    {(DRIVE_KINEMATICS[0][0] * B[0]) + (DRIVE_KINEMATICS[0][1] * B[1]) + (DRIVE_KINEMATICS[0][2] * B[2]),
     (DRIVE_KINEMATICS[1][0] * B[0]) + (DRIVE_KINEMATICS[1][1] * B[1]) + (DRIVE_KINEMATICS[1][2] * B[2]),
     (DRIVE_KINEMATICS[2][0] * B[0]) + (DRIVE_KINEMATICS[2][1] * B[1]) + (DRIVE_KINEMATICS[2][2] * B[2]),
     (DRIVE_KINEMATICS[3][0] * B[0]) + (DRIVE_KINEMATICS[3][1] * B[1]) + (DRIVE_KINEMATICS[3][2] * B[2]),
     (DRIVE_KINEMATICS[4][0] * B[0]) + (DRIVE_KINEMATICS[4][1] * B[1]) + (DRIVE_KINEMATICS[4][2] * B[2]),
     (DRIVE_KINEMATICS[5][0] * B[0]) + (DRIVE_KINEMATICS[5][1] * B[1]) + (DRIVE_KINEMATICS[5][2] * B[2]),
     (DRIVE_KINEMATICS[6][0] * B[0]) + (DRIVE_KINEMATICS[6][1] * B[1]) + (DRIVE_KINEMATICS[6][2] * B[2]),
     (DRIVE_KINEMATICS[7][0] * B[0]) + (DRIVE_KINEMATICS[7][1] * B[1]) + (DRIVE_KINEMATICS[7][2] * B[2])};

    return C;
}
}

int main() {
    for (int loop = 0; loop < LOOP_AMOUNT; loop++) {
        D = {0, 0, 0, 0, 0, 0, 0, 0};
        std::cout << "input x, y, and theta joystick inputs [-1, 1]\n$ ";
        std::cin >> D.xInput;
        std::cin >> D.yInput;
        std::cin >> D.thetaInput;

        #ifdef DEBUG_PRINT
        D.print(Input::input, "INPUT");
        #endif

        /** swerveDrive command */
        SwerveDrive::fromFieldRelativeSpeeds(
            -DriveSubsystem::modifyAxis(D.yInput) * MAX_VELOCITY_METERS_PER_SECOND,
            -DriveSubsystem::modifyAxis(D.xInput) * MAX_VELOCITY_METERS_PER_SECOND,
            -DriveSubsystem::modifyAxis(D.thetaInput / 1.25) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            0);

        #ifdef DEBUG_PRINT
        D.print(Input::intermediate, "fromFieldRelativeSpeeds");
        #endif

        /** `toSwerveModuleStates()` */
        std::array<float, 8> vector;
        States states[4];
        float B[3] = {D.xIntermediate, D.yIntermediate, D.thetaIntermediate};
        vector = Matrix::mult(B);

        for (int i = 0; i < 4; i++) {
            float x = vector[i * 2];
            float y = vector[i * 2 + 1];

            float speed = std::hypot(x, y);
            float angle, sin, cos;

            if (speed > 1e-6) {
                sin = y / speed;
                cos = x / speed;
            } else {
                sin = 0;
                cos = 1;
            }
            angle = std::atan2(sin, cos);

            states[i] = {speed, angle};
        } /** end `toSwerveModuleStates()` */

        #ifdef DEBUG_PRINT
        std::cout << "toSwerveModuleStates\n";
        for (int i = 0; i < 4; i++) {
            std::cout << "speed: " << states[i].speed <<
                       "\nangle: " << states[i].angle << '\n';
        }
        #endif

        DriveSubsystem::desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        #ifdef DEBUG_PRINT
        std::cout << "desaturateWheelSpeeds\n";
        for (int i = 0; i < 4; i++) {
            std::cout << "speed: " << states[i].speed <<
                       "\nangle: " << states[i].angle << '\n';
        }
        #endif

        SDS::set(states[0].speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle * (180/PI), 0);
        SDS::set(states[1].speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle * (180/PI), 1);
        SDS::set(states[2].speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle * (180/PI), 2);
        SDS::set(states[3].speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle * (180/PI), 3);
    }
}
