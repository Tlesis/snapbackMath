#include <iostream>
#include <cmath>
#include <array>

#include "constants.h"
#include "driveSub.h"
#include "swerveDrive.h"
#include "moduleFactory.h"
#include "mat.h"

#define LOOP_AMOUNT INFINITY

int main() {
    for (int loop = 0; loop < LOOP_AMOUNT; loop++) {
        D = {0, 0, 0, 0, 0, 0, 0, 0};
        std::cout << "input x, y, and theta joystick inputs\n$ ";
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
            -(10 * (PI/180))
        );

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
