#ifndef ARM_H
#define ARM_H

#include <cstdint>

#include "ArmParameters.h"
#include "InverseKinematics.h"
#include "RoveMatrix.h"
#include "Smoco.h"

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

public:
    Arm();
    // Drive joints with given powers
    void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty);
    // Drive joints to target angles
    void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);
    // Drive joints such that J5 is centered at the given coordinate
    void driveInverseKinematics(float x, float y, float z, float J4Angle, float J5Angle, float J6Angle);
    // Configure limits
    void limitSwitchOverride(uint16_t bitmask);
    void softLimitOverride(uint16_t bitmask);
    // Update arm logic (collision handling, etc)
    void update(float delta);

    JointPositions getJointPositions() const;
    Vector getGripperCoordinates() const;
    // void holdCurrentPosition();


private:
    // Use GJK to detect and resolve collisions
    // void handleCollisions();
};

#endif // ARM_H
