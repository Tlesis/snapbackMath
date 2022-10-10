#pragma once

#include <array>

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