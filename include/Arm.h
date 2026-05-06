#ifndef ARM_H
#define ARM_H

#include <cstdint>

#include "ArmParameters.h"
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

    void setControlMode(ControlMode);
    ControlMode currentMode = ControlMode::OPEN_LOOP;
    Vector j4j5j6Target = {0};
    Vector gripperTarget = {0};
    TransfMatrix gripperRotation = Rotation(0, M_PI, 0);
    float snappingThreshold = 0.15; // set to 0 to disable

public:
    Arm();
    void estop();
    // Drive joints with given powers
    void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty);
    // Drive joints to target angles
    void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);
    // Increment joint targets
    void incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);

    void incrementInverseKinematicsPosition(float x, float y, float z, float j4, float j5, float j6);

    void incrementInverseKinematicsToolPose(float tx, float ty, float tz, float rx, float ry, float rz);

    void incrementInverseKinematicsWorldPose(float tx, float ty, float tz, float rx, float ry, float rz);

    void driveInverseKinematics(const TransfMatrix& targetPose);

    void snapTargetPoseToYZ();

    void setYZSnappingThreshold(float threshold);

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
