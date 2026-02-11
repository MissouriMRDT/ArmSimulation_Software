#include <raylib.h>
#include "InverseKinematics.h"

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
        0,        0,                  0,                  1
    };
}

TransfMatrix IK::CalculateForwardTransform(DHParameters joints[], size_t count) {
    TransfMatrix forward = Identity();
    for (int i = 0; i < count; i++) {
        forward = forward * TransformFromDH(joints[i]); // recall associative property of matrices
    }
    return forward;
}

// bool CalculateInverseKinematics(TransfMatrix valkTransf, Vector &pos, Vector &gripperPos, float wristJ4, float wristPitch, float wristValkyrie, float &q1, float &q2, float &q3, float &q4, float &qP, float &qV, float &J3FwdLim, float &J3RevLim, bool lockMode) 
// {
    
// 	//Calculate which solution to use
// 	bool underMode = (q3 > 0)? true : false;

// 	//Limit J3 based on which solution is used
// 	if (underMode) {
// 		J3FwdLim = J3_POS_LIM;
// 		J3RevLim = J3_MID_LIM;
// 	} else {
// 		J3FwdLim = -J3_MID_LIM;
// 		J3RevLim = J3_NEG_LIM;
// 	}

//     qP = wristPitch - (q2 + q3);
//     qV = wristValkyrie;

//     q4 = 0; //Lock J4
//     gripperPos = {0,0,0}; //pixels
//     gripperPos = gripperPos * (Translation(-VALK_LENGTH, 0, 0) * valkTransf);

// 	//Calculate target angles using IK
// 	q1 = pos.z;
// 	q3 = RAD2DEG*acos((pow(pos.x,2)+pow(pos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));

// 	if (underMode) q2 = RAD2DEG*(atan2(pos.y, pos.x) - atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
// 	else q2 = RAD2DEG*(atan2(pos.y, pos.x) + atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
	
// 	q3 = underMode? q3 : -q3;

// 	// Check if calculated angle is invalid and limit movement
// 	if (!(isInSafeZone(J1_FWD_LIM, J1_REV_LIM, q1) && isInSafeZone(J2_FWD_LIM, J2_REV_LIM, q2) && isInSafeZone(J3FwdLim, J3RevLim, q3) && isInSafeZone(PITCH_FWD_LIM, PITCH_REV_LIM, qP) && isInSafeZone(J4_FWD_LIM, J4_REV_LIM, q4))) return false;
//     return true;
// }
