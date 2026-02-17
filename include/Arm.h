#ifndef ARM_H
#define ARM_H

#include <cstdint>

#include "ArmParameters.h"
#include "InverseKinematics.h"
#include "RoveMatrix.h"
#include "Smoco.h"

enum class ControlMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    IK_POSE,
    IK_WRIST
};

class Arm {
private:
    Smoco XMotor;
    Smoco J2Motor;
    Smoco J3Motor;
    Smoco J4Motor;
    Smoco J5Motor;
    Smoco J6Motor;
    Smoco GripperMotor;
    int32_t J6Zero = 0;

    ControlMode currentMode = ControlMode::OPEN_LOOP;
    Vector gripperTarget = {0};
    Vector j4j5j6Target = {0};
    TransfMatrix wristRotation = Rotation(0, M_PI_2, 0);

public:
    Arm();
    // Drive joints with given powers
    void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty);
    // Drive joints to target angles
    void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);
    // Increment joint targets
    void incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);
    
    void incrementInverseKinematicsPosition(float x, float y, float z, float j4, float j5, float j6);
    
    void incrementInverseKinematicsPose(float tx, float ty, float tz, float rx, float ry, float rz);

    void driveInverseKinematics(const TransfMatrix& targetPose);
    // Configure limits
    void limitSwitchOverride(uint16_t bitmask);
    void softLimitOverride(uint16_t bitmask);
    // Update arm logic (collision handling, etc)
    void update(float delta);

    JointPositions getJointPositions() const;
    Vector getGripperCoordinates() const;
    bool isPositionWithinLimits(const JointPositions& angles);
    // void holdCurrentPosition();

    ControlMode getCurrentMode() const { return currentMode; }
    Vector getTarget() const { return gripperTarget; }

private:
    // Use GJK to detect and resolve collisions
    // void handleCollisions();
};

#endif // ARM_H
