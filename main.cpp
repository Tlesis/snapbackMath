#include <iostream>
#include <cmath>
#include <array>

#include "constants.h"
#include "driveSub.h"
#include "swerveDrive.h"
#include "moduleFactory.h"
#include "mat.h"

#define LOOP_AMOUNT 1

int main() {
    for (int loop = 0; loop < LOOP_AMOUNT; loop++) {
        D = {0};
        std::cout << "input x, y, and theta joystick inputs\n$ ";
        std::cin >> D.xInput;
        std::cin >> D.yInput;
        std::cin >> D.thetaInput;

        /** swerveDrive command */
        SwerveDrive::fromFieldRelativeSpeeds(
            -DriveSubsystem::modifyAxis(D.yInput) * MAX_VELOCITY_METERS_PER_SECOND,
            -DriveSubsystem::modifyAxis(D.xInput) * MAX_VELOCITY_METERS_PER_SECOND,
            -DriveSubsystem::modifyAxis(D.thetaInput / 1.25) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            0
        );

        /** `toSwerveModuleStates()` */
        std::array<float, 8> vector;
        States states[4];
        float B[3] = {D.xFinal, D.yFinal, D.thetaFinal};
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

        DriveSubsystem::desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // Front Left
        SDS::set(states[0].speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle * (180/PI));

        /** return collected data */
        std::cout <<
        "----------  " << loop << "  ----------\n"     <<
        "\nxInput:\t\t" << D.xInput          << '\n'   <<
        "yInput:\t\t"   << D.yInput          << '\n'   <<
        "thetaInput:\t" << D.thetaInput      << "\n\n" <<
        "driveMotor:\t" << D.driveMotorSpeed << '\n'   <<
        "steerMotor:\t" << D.steerMotorPos   << '\n'   <<
        "-------------------------\n\n";
    }
}
