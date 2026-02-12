#ifndef ARM_PARAMETERS_H
#define ARM_PARAMETERS_H

#include <cstdint>

constexpr auto SHOULDER_OVERHANG = 4.519;
constexpr auto SHOULDER_LENGTH = 8.256;
constexpr auto BICEP_LENGTH = 17.0;
constexpr auto FOREARM_ROLL_LENGTH = 5.0;
constexpr auto FOREARM_ROLL_PARTIAL_LENGTH = 9.25;
constexpr auto FOREARM_PARTIAL_LENGTH = 7.5;
constexpr auto FOREARM_LENGTH = FOREARM_ROLL_PARTIAL_LENGTH + FOREARM_PARTIAL_LENGTH;
constexpr auto WRIST_LENGTH = 2.926;
constexpr auto GRIPPER_LENGTH = 6.5; // ish

struct JointPositions {
    float X, J2, J3, J4, J5, J6;
};

// Soft Limits
constexpr auto X_REV_LIM = INT32_MIN;
constexpr auto X_FWD_LIM = INT32_MAX;
constexpr auto X_ENC_PER_IN = ((8300 - 13100) / 1.5);

constexpr auto J2_REV_LIM = 600;
constexpr auto J2_ZERO = 1700;
constexpr auto J2_FWD_LIM = 2400;
constexpr auto J2_ENC_PER_DEG = ((1700 - 700) / 90.0);

// J3 Encoder Reversed!
constexpr auto J3_REV_LIM = -600;
constexpr auto J3_ZERO = 1200 - 900;
constexpr auto J3_FWD_LIM = 1200;
constexpr auto J3_ENC_PER_DEG = ((1200 - 300) / 90.0);

constexpr auto J4_REV_LIM = -2000;
constexpr auto J4_ZERO = 2150;
constexpr auto J4_FWD_LIM = 6300;
constexpr auto J4_ENC_PER_DEG = ((2150 - 3200) / 90.0);

constexpr auto J5_REV_LIM = -600;
constexpr auto J5_ZERO = 350;
constexpr auto J5_FWD_LIM = 1350;
constexpr auto J5_ENC_PER_DEG = ((1350 - 350) / 90.0);

constexpr auto J6_ENC_PER_DEG = ((12400 - 6170) / 180.0);

#endif // ARM_PARAMETERS_H
