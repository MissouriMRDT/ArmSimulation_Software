#include "Arm.h"

#include <iostream>

// static ACAN_T4 canChannel = ACAN_T4({125000});

// // DH Parameters, all distances in inches, all rotations in radians
// IK::DHParameters ArmParameters[6] = {
//     { M_PI_2, 0 /*q1*/, 0, SHOULDER_LENGTH },
//     { 0 /*-q2*/, 0, 0, BICEP_LENGTH },
//     { 0 /*-q3*/, 0, M_PI_2, FOREARM_ROLL_LENGTH },
//     { 0 /*q4*/, FOREARM_LENGTH, -M_PI_2, 0},
//     { 0 /*q5*/, 0, M_PI_2, 0},
//     { 0 /*q6*/, WRIST_LENGTH + GRIPPER_LENGTH, 0, 0 }
// };

Arm::Arm() :
    XMotor(degToEnc(-6.33, 0, X_ENC_PER_IN), 0.5 * X_ENC_PER_IN),
    J2Motor(degToEnc(60, J2_ZERO, J2_ENC_PER_DEG), 20 * J2_ENC_PER_DEG),
    J3Motor(degToEnc(-80, J3_ZERO, J3_ENC_PER_DEG), 20 * J3_ENC_PER_DEG),
    J4Motor(degToEnc(0, J4_ZERO, J4_ENC_PER_DEG), 20 * J4_ENC_PER_DEG),
    J5Motor(degToEnc(56, J5_ZERO, J5_ENC_PER_DEG), 20 * J5_ENC_PER_DEG),
    J6Motor(degToEnc(0, 0, J6_ENC_PER_DEG), 6 * J6_ENC_PER_DEG),
    GripperMotor(0, 1)
{
        // Set PID gains
    XMotor.setPID(0.7, 0, 0);
    J2Motor.setPID(0.7, 0, 0);
    J3Motor.setPID(0.7, 0, 0);
    J4Motor.setPID(0.7, 0, 0);
    J5Motor.setPID(0.7, 0, 0);
    J6Motor.setPID(0.7, 0, 0);
    GripperMotor.setPID(0.7, 0, 0);

    // Set soft limits
    XMotor.setSoftLimitPosition(X_REV_LIM, X_FWD_LIM);
    J2Motor.setSoftLimitPosition(J2_REV_LIM, J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(J3_REV_LIM, J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(J4_REV_LIM, J4_FWD_LIM);
    J5Motor.setSoftLimitPosition(J5_REV_LIM, J5_FWD_LIM);
    J6Motor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    // Set ramp rates
    XMotor.setRampRate(100.0);
    J2Motor.setRampRate(100.0);
    J3Motor.setRampRate(100.0);
    J4Motor.setRampRate(100.0);
    J5Motor.setRampRate(100.0);
    J6Motor.setRampRate(100.0);
    GripperMotor.setRampRate(100.0);
}

// Drive joints with given powers
void Arm::driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty) {
    XMotor.driveOpenLoop(XDuty);
    J2Motor.driveOpenLoop(J2Duty);
    J3Motor.driveOpenLoop(J3Duty);
    J4Motor.driveOpenLoop(J4Duty);
    J5Motor.driveOpenLoop(J5Duty);
    J6Motor.driveOpenLoop(J6Duty);
}
// Drive joints to target angles
void Arm::driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    XMotor.driveTargetPosition(degToEnc(XAngle, 0, X_ENC_PER_IN), 0.05);
    J2Motor.driveTargetPosition(degToEnc(J2Angle, J2_ZERO, J2_ENC_PER_DEG), 0.05);
    J3Motor.driveTargetPosition(degToEnc(J3Angle, J3_ZERO, J3_ENC_PER_DEG), 0.05);
    J4Motor.driveTargetPosition(degToEnc(J4Angle, J4_ZERO, J4_ENC_PER_DEG), 0.05);
    J5Motor.driveTargetPosition(degToEnc(J5Angle, J5_ZERO, J5_ENC_PER_DEG), 0.05);
    J6Motor.driveTargetPosition(degToEnc(J6Angle, J6Zero, J6_ENC_PER_DEG), 0.05);
}

void Arm::driveInverseKinematics(const TransfMatrix& targetPose) {
    JointPositions angles = getJointPositions();
    if (IK::CalculateInverseKinematics(targetPose, angles)) {
        std::cout << angles.J6 << std::endl;
        uint16_t triggeredLimits = wouldViolateSoftLimits(angles);
        if (triggeredLimits) {
            std::cout << "Outside limits" << std::endl;
        } else {
            driveTargetAngles(angles.X, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6);
        }
    } else {
        std::cout << "IK failed" << std::endl;
    }
}

void Arm::limitSwitchOverride(uint16_t bitmask) {
    XMotor.configIgnoreLimits(bitmask & (1 << 0), bitmask & (1 << 1));
    J2Motor.configIgnoreLimits(bitmask & (1 << 2), bitmask & (1 << 3));
    J3Motor.configIgnoreLimits(bitmask & (1 << 4), bitmask & (1 << 5));
    J4Motor.configIgnoreLimits(bitmask & (1 << 6), bitmask & (1 << 7));
    J5Motor.configIgnoreLimits(bitmask & (1 << 8), bitmask & (1 << 9));
}

// Configure soft limits
void Arm::softLimitOverride(uint16_t bitmask) {
    XMotor.setSoftLimitPosition(bitmask & (1 << 0) ? INT32_MIN : X_REV_LIM,
                                bitmask & (1 << 1) ? INT32_MAX : X_FWD_LIM);
    J2Motor.setSoftLimitPosition(bitmask & (1 << 2) ? INT32_MIN : X_REV_LIM,
                                    bitmask & (1 << 3) ? INT32_MAX : J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(bitmask & (1 << 4) ? INT32_MIN : X_REV_LIM,
                                    bitmask & (1 << 5) ? INT32_MAX : J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(bitmask & (1 << 6) ? INT32_MIN : J4_REV_LIM,
                                    bitmask & (1 << 7) ? INT32_MAX : J4_FWD_LIM);
    J5Motor.setSoftLimitPosition(bitmask & (1 << 8) ? INT32_MIN : J5_REV_LIM,
                                    bitmask & (1 << 9) ? INT32_MAX : J5_FWD_LIM);
}
// Update arm logic (collision handling, etc)
void Arm::update(float delta) {
    XMotor.update(delta);
    J2Motor.update(delta);
    J3Motor.update(delta);
    J4Motor.update(delta);
    J5Motor.update(delta);
    J6Motor.update(delta);
    GripperMotor.update(delta);
}

JointPositions Arm::getJointPositions() const {
    return {
        encToDeg(XMotor.getPosition(), 0, X_ENC_PER_IN),
        encToDeg(J2Motor.getPosition(), J2_ZERO, J2_ENC_PER_DEG),
        encToDeg(J3Motor.getPosition(), J3_ZERO, J3_ENC_PER_DEG),
        encToDeg(J4Motor.getPosition(), J4_ZERO, J4_ENC_PER_DEG),
        encToDeg(J5Motor.getPosition(), J5_ZERO, J5_ENC_PER_DEG),
        encToDeg(J6Motor.getPosition(), J6Zero, J6_ENC_PER_DEG),
    };
}

Vector Arm::getGripperCoordinates() const {
    JointPositions angles = getJointPositions();
    return
    // Rotation(0, -M_PI_2, 0) // Initial frame
    IK::CalculateForwardTransform(angles)
    * Vector{0, 0, 0};
}

uint16_t Arm::wouldViolateSoftLimits(const JointPositions &angles) const {
    return (uint16_t) 0
        | (angles.X > X_FWD_LIM_IN) << 0
        | (angles.X < X_REV_LIM_IN) << 1
        | (angles.J2 > J2_FWD_LIM_DEG) << 2
        | (angles.J2 < J2_REV_LIM_DEG) << 3
        | (angles.J3 > J3_FWD_LIM_DEG) << 4
        | (angles.J3 < J3_REV_LIM_DEG) << 5
        | (angles.J4 > J4_FWD_LIM_DEG) << 6
        | (angles.J4 < J4_REV_LIM_DEG) << 7
        | (angles.J5 > J5_FWD_LIM_DEG) << 8
        | (angles.J5 < J5_REV_LIM_DEG) << 9;
}
