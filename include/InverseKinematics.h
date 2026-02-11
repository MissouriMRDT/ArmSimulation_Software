#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "RoveMatrix.h"

// #define J2_LENGTH 18
// #define J3_LENGTH 18.5

// #define J1_FWD_LIM 6.3
// #define J1_REV_LIM -6.3

// #define J2_FWD_LIM 164
// #define J2_REV_LIM -54

// #define J3_POS_LIM 90
// #define J3_NEG_LIM -116.8
// #define J3_MID_LIM 15

// #define PITCH_FWD_LIM 80
// #define PITCH_REV_LIM 100

// #define J4_FWD_LIM 260
// #define J4_REV_LIM 280

// #define WRIST_RAD 2.887499685
// #define SHOULDER_LENGTH 7.328739921
// #define VALK_LENGTH 6.24943834646

namespace IK {

    struct DHParameters {
        float theta; // Angle about previous Z from old X to new X
        float d; // Distance along previous Z to common normal
        float alpha; // Angle about common normal from old Z to new Z
        float a; // Length of common normal
    };

    TransfMatrix TransformFromDH(const DHParameters &params);
    TransfMatrix CalculateForwardTransform(DHParameters joints[], size_t count);

    //If returns false, calculated angle targets are outside of range. HoldCurrentPosition() can then be called
    //Input angles are in degrees
    // bool CalculateInverseKinematics(TransfMatrix valkTransf, Vector &pos, Vector &gripperPos, float wristJ4, float wristPitch, float wristValkyrie, float &q1, float &q2, float &q3, float &q4, float &qP, float &qV, float &J3FwdLim, float &J3RevLim, bool lockMode);

}

#endif /*INVERSE_KINEMATICS_H*/
