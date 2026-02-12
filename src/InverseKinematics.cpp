#include <raylib.h>
#include "InverseKinematics.h"
#include <iostream>

// DH Parameters, all distances in inches, all rotations in radians
IK::DHParameters IK::DHTable[6] = {
    { M_PI_2, 0 /*q1*/, 0, SHOULDER_LENGTH },
    { 0 /*-q2*/, 0, 0, BICEP_LENGTH },
    { 0 /*-q3*/, 0, M_PI_2, FOREARM_ROLL_LENGTH },
    { 0 /*q4*/, FOREARM_LENGTH, -M_PI_2, 0},
    { 0 /*q5*/, 0, M_PI_2, 0},
    { 0 /*q6*/, WRIST_LENGTH + GRIPPER_LENGTH, 0, 0 }
};

#define DH_1 IK::DHTable[0]
#define DH_2 IK::DHTable[1]
#define DH_3 IK::DHTable[2]
#define DH_4 IK::DHTable[3]
#define DH_5 IK::DHTable[4]
#define DH_6 IK::DHTable[5]

// sqrt() is not constexpr until C++26 :(


// The hypotenuse of the right triangle formed by a3 and d4
static const float l1 = sqrt(DH_3.a*DH_3.a + DH_4.d*DH_4.d);
// The angle of the right triangle formed by a3 and d4
static const float theta1 = atan(DH_4.d / DH_3.a);

TransfMatrix IK::TransformFromDH(const DHParameters &params) {
    float cosTheta = cos(params.theta);
    float sinTheta = sin(params.theta);
    float cosAlpha = cos(params.alpha);
    float sinAlpha = sin(params.alpha);

    // T n-1 -> n = [Zn-1]*[Xn]
    // [Zi] = RotateZ(thetai) * TranslateZ(di)
    // [Xi] = RotateX(alpha) * TranslateX(a)
    // This gives
    /*
    *  |      |   |
    *  |  R   | T |
    *  |      |   |
    *  |----------|
    *  |0 0 0 | 1 |
    */

    return {
        cosTheta, -sinTheta*cosAlpha, sinTheta*sinAlpha,  params.a*cosTheta,
        sinTheta, cosTheta*cosAlpha,  -cosTheta*sinAlpha, params.a*sinTheta,
        0,        sinAlpha,           cosAlpha,           params.d,
        // 0,        0,                  0,                  1
    };
}

TransfMatrix IK::CalculateForwardTransform(const JointPositions &q) {
    DH_1.d = q.X;
    DH_2.theta = q.J2 * M_PI/180;
    DH_3.theta = q.J3 * M_PI/180;
    DH_4.theta = q.J4 * M_PI/180;
    DH_5.theta = q.J5 * M_PI/180;
    DH_6.theta = q.J6 * M_PI/180;
    TransfMatrix forward = Identity();
    for (const DHParameters &param : DHTable) {
        forward = forward * TransformFromDH(param); // recall associative property of matrices
    }
    return forward;
}

bool IK::CalculateInverseKinematics(const TransfMatrix &targetPose, JointPositions &outPositions) {
    Vector p06 = { targetPose.m03, targetPose.m13, targetPose.m23 };
    Vector z06 = {};
    // Wrist center
    Vector p0w = p06;
    // X axis alone determines the position of the wrist center point along z0
    float q1 = p0w.z;
    
    // The link a3, d4 forms a right triangle, so consider it as a single object
    // No reason to recalculate these, so they're constants elsewhere
    // float l1 = sqrt(DH_3.a*DH_3.a + DH_4.d*DH_4.d);
    // float theta1 = atan(DH_4.d / DH_3.a);

    // Distance from J2 to J5
    float l2 = sqrt(p0w.x*p0w.x + (p0w.y-DH_1.a)*(p0w.y-DH_1.a));

    // Legs of triangle longer than hypotenuse
    if (DH_2.a + l1 < l2) return false;

    // Use law of cosines to find angle between a2 and l1 in the triangle formed by a2, l1, and l2
    // l2^2 = l1^2 + a2^2 - 2*l1*a2*cos(theta2)
    float theta2 = acos((l1*l1 + DH_2.a*DH_2.a - l2*l2) / (2 * l1 * DH_2.a));
    // Where 0 < theta2 < pi

    // There are two possible solutions we must account for
    float q3_1 = M_PI - theta1 - theta2;
    float q3_2 = M_PI - theta1 + theta2;

    // Use law of cosines to find angle between a1 and l2 in the triangle formed by a2, l1, and l2
    // l1^2 = l2^2 + a2^2 - 2*l2*a2*cos(theta2)
    float theta3 = acos((l2*l2 + DH_2.a*DH_2.a - l1*l1) / (2 * l2 * DH_2.a));

    // Use law of cosines to find angle between x2 and l2
    float theta4 = acos(( (p0w.y-DH_1.a)*(p0w.y-DH_1.a) + l2*l2 - p0w.x*p0w.x ) / (2 * (p0w.y - DH_1.a) * l2));

    // Determined uniquely by choice of q3
    
    float q2_1, q2_2;
    if (p0w.x > 0) {
        q2_1 = - theta3 + theta4;
        q2_2 = theta3 + theta4;
    } else {
        q2_1 = - theta3 - theta4;
        q2_2 = theta3 - theta4;
    }

    outPositions.X = q1;
    outPositions.J2 = -q2_1 * 180 / M_PI;
    outPositions.J3 = -q3_1 * 180 / M_PI;

    // For debug purposes
    DH_1.d = outPositions.X;
    DH_2.theta = outPositions.J2 * M_PI/180;
    DH_3.theta = outPositions.J3 * M_PI/180;
    DH_4.theta = outPositions.J4 * M_PI/180;
    DH_5.theta = outPositions.J5 * M_PI/180;
    DH_6.theta = outPositions.J6 * M_PI/180;

    return true;
    
}
