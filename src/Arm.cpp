#include "Arm.h"

#include <iostream>

Arm::Arm() :
    XMotor(degToEnc(6.33, 0, X_ENC_PER_IN), 0.5 * X_ENC_PER_IN),
    J2Motor(degToEnc(60, J2_ZERO, J2_ENC_PER_DEG), 20 * J2_ENC_PER_DEG),
    J3Motor(degToEnc(-80, J3_ZERO, J3_ENC_PER_DEG), 20 * J3_ENC_PER_DEG),
    J4Motor(degToEnc(0, J4_ZERO, J4_ENC_PER_DEG), 20 * J4_ENC_PER_DEG),
    J5Motor(degToEnc(56, J5_ZERO, J5_ENC_PER_DEG), 20 * J5_ENC_PER_DEG),
    J6Motor(degToEnc(0, 0, J6_ENC_PER_DEG), 6 * J6_ENC_PER_DEG),
    GripperMotor(0, 1)
{
    XMotor.configAngleConversion(0, X_ENC_PER_IN);
    J2Motor.configAngleConversion(J2_ZERO, J2_ENC_PER_DEG);
    J3Motor.configAngleConversion(J3_ZERO, J3_ENC_PER_DEG);
    J4Motor.configAngleConversion(J4_ZERO, J4_ENC_PER_DEG);
    J5Motor.configAngleConversion(J5_ZERO, J5_ENC_PER_DEG);
    J6Motor.configAngleConversion(J6Zero, J6_ENC_PER_DEG);

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
    if (currentMode != ControlMode::OPEN_LOOP) std::cout << "SETTING TO OPEN LOOP" << std::endl;
    currentMode = ControlMode::OPEN_LOOP;
    XMotor.driveOpenLoop(XDuty);
    J2Motor.driveOpenLoop(J2Duty);
    J3Motor.driveOpenLoop(J3Duty);
    J4Motor.driveOpenLoop(J4Duty);
    J5Motor.driveOpenLoop(J5Duty);
    J6Motor.driveOpenLoop(J6Duty);
}
// Drive joints to target angles
void Arm::driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    if (currentMode != ControlMode::CLOSED_LOOP) std::cout << "SETTING TO CLOSED LOOP" << std::endl;
    currentMode = ControlMode::CLOSED_LOOP;
    XMotor.driveTargetAngle(XAngle, 0.05);
    J2Motor.driveTargetAngle(J2Angle, 0.05);
    J3Motor.driveTargetAngle(J3Angle, 0.05);
    J4Motor.driveTargetAngle(J4Angle, 0.05);
    J5Motor.driveTargetAngle(J5Angle, 0.05);
    J6Motor.driveTargetAngle(J6Angle, 0.05);
}

// Increment joint angles
void Arm::incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    if (currentMode != ControlMode::CLOSED_LOOP) {
        driveTargetAngles(XMotor.getAngle(), J2Motor.getAngle(), J3Motor.getAngle(), J4Motor.getAngle(),
                            J5Motor.getAngle(), J6Motor.getAngle());
    }
    currentMode = ControlMode::CLOSED_LOOP;
    XMotor.driveTargetAngle(XMotor.getTargetAngle() + XAngle, 0.05);
    J2Motor.driveTargetAngle(J2Motor.getTargetAngle() + J2Angle, 0.05);
    J3Motor.driveTargetAngle(J3Motor.getTargetAngle() + J3Angle, 0.05);
    J4Motor.driveTargetAngle(J4Motor.getTargetAngle() + J4Angle, 0.05);
    J5Motor.driveTargetAngle(J5Motor.getTargetAngle() + J5Angle, 0.05);
    J6Motor.driveTargetAngle(J6Motor.getTargetAngle() + J6Angle, 0.05);
}

void Arm::incrementInverseKinematicsPosition(float x, float y, float z, float j4, float j5, float j6) {
    if (currentMode != ControlMode::IK_WRIST) {
        JointPositions levelAngles = getJointPositions();
        // wrist center
        TransfMatrix currentPose = IK::CalculateForwardTransform(levelAngles);
        Vector wristCoords = currentPose.getTranslation() - currentPose.getRotation() * ((WRIST_LENGTH+GRIPPER_LENGTH)*BASIS_Z);
        // gripper center
        gripperTarget = wristCoords + (WRIST_LENGTH+GRIPPER_LENGTH)*BASIS_X;
        // compute what the angles would be
        IK::CalculateInverseKinematics(Translation(gripperTarget.x, gripperTarget.y, gripperTarget.z)*Rotation(0, M_PI_2, 0), levelAngles);
        // compute what the angles should be
        j4j5j6Target = {
            J4Motor.getAngle() - levelAngles.J4,
            J5Motor.getAngle() - levelAngles.J5,
            J6Motor.getAngle() - levelAngles.J6
        };
        std::cout << "SETTING TO WRIST CONTROL" << std::endl;
    }
    currentMode = ControlMode::IK_WRIST;

    TransfMatrix targetPose = Translation(gripperTarget.x + x, gripperTarget.y + y, gripperTarget.z + z) // gripper coords
                                * Rotation(0, M_PI_2, 0); // wrist facing forward
    // calculate IK up to wrist
    JointPositions angles = getJointPositions();
    if (!IK::CalculateInverseKinematics(targetPose, angles)) return;

    // set back to current target angles
    JointPositions currentAngles = getJointPositions();
    
    angles.J4 += j4j5j6Target.x + j4;
    angles.J5 += j4j5j6Target.y + j5;
    angles.J6 += j4j5j6Target.z + j6;
    
    if (!isPositionWithinLimits(angles)) return;

    gripperTarget.x += x;
    gripperTarget.y += y;
    gripperTarget.z += z;
    j4j5j6Target.x += j4;
    j4j5j6Target.y += j5;
    j4j5j6Target.z += j6;

    XMotor.driveTargetAngle(angles.X, 0.05);
    J2Motor.driveTargetAngle(angles.J2, 0.05);
    J3Motor.driveTargetAngle(angles.J3, 0.05);
    J4Motor.driveTargetAngle(angles.J4, 0.05);
    J5Motor.driveTargetAngle(angles.J5, 0.05);
    J6Motor.driveTargetAngle(angles.J6, 0.05);

}

void Arm::incrementInverseKinematicsPose(float tx, float ty, float tz, float rx, float ry, float rz) {
    if (currentMode != ControlMode::IK_POSE) {
        TransfMatrix currentPose = IK::CalculateForwardTransform(getJointPositions());
        gripperTarget = currentPose.getTranslation();
        wristRotation = currentPose.getRotation();
        std::cout << "SETTING TO POSE CONTROL" << std::endl;
    }
    currentMode = ControlMode::IK_POSE;

    TransfMatrix targetPose = Translation(gripperTarget.x + tx, gripperTarget.y + ty, gripperTarget.z + tz)
    // incrementally rotate wrist in world space
    * Rotation(0, ry * M_PI / 180, 0) // rotate about Y
    * Rotation(rx * M_PI / 180, 0, 0) // rotate about X
    * Rotation(0, 0, rz * M_PI / 180) // rotate about Z
    * wristRotation;

    driveInverseKinematics(targetPose);
}

void Arm::driveInverseKinematics(const TransfMatrix& targetPose) {
    currentMode = ControlMode::IK_POSE;
    JointPositions angles = getJointPositions();
    if (!IK::CalculateInverseKinematics(targetPose, angles)) return;
    if (!isPositionWithinLimits(angles)) return;
    gripperTarget = targetPose.getTranslation();
    wristRotation = targetPose.getRotation();
    XMotor.driveTargetAngle(angles.X, 0.05);
    J2Motor.driveTargetAngle(angles.J2, 0.05);
    J3Motor.driveTargetAngle(angles.J3, 0.05);
    J4Motor.driveTargetAngle(angles.J4, 0.05);
    J5Motor.driveTargetAngle(angles.J5, 0.05);
    J6Motor.driveTargetAngle(angles.J6, 0.05);
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

bool Arm::isPositionWithinLimits(const JointPositions& angles) {
    return XMotor.isAngleWithinLimits(angles.X) && J2Motor.isAngleWithinLimits(angles.J2) &&
           J3Motor.isAngleWithinLimits(angles.J3) && J4Motor.isAngleWithinLimits(angles.J4) &&
           J5Motor.isAngleWithinLimits(angles.J5) && J6Motor.isAngleWithinLimits(angles.J6);
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
        XMotor.getAngle(),
        J2Motor.getAngle(),
        J3Motor.getAngle(),
        J4Motor.getAngle(),
        J5Motor.getAngle(),
        J6Motor.getAngle(),
    };
}

Vector Arm::getGripperCoordinates() const {
    JointPositions angles = getJointPositions();
    return IK::CalculateForwardTransform(angles) * ORIGIN;
}
