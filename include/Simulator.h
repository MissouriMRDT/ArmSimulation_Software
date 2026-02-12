#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include <raylib.h>
#include <resource_dir.h>

#include "ArmParameters.h"
#include "RoveMatrix.h"
#include "Arm.h"

#define OPEN_LOOP_DUTY (INT16_MAX / 2)
#define CLOSED_LOOP_ANGULAR_SPEED 10.0f // degrees per second
#define CLOSED_LOOP_LINEAR_SPEED 0.5f // inches per second
#define IK_TARGET_SPEED 1.0f // inches per second
#define SPD_MOD2 2.0f
#define SPD_MOD4 4.0f

enum AxesNames {
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    RIGHT_STICK_X = 2,
    RIGHT_STICK_Y = 3,
    BUMPERS = 4,
    TRIGGERS = 5,
    D_PAD_X = 6,
    D_PAD_Y = 7,
    LEFT_BUMPER_TRIGGER = 8,
    RIGHT_BUMPER_TRIGGER = 9,
    AXES_COUNT
};

class Simulator {
    private:

        Vector wristTarget, wristRotation;
        JointPositions targetAngles;
        float axes[AXES_COUNT];

        int selectedGamepad = 0;

        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP,
            INVERSE_KINEMATICS
        };

        Camera camera;
        Vector2 orbit;

        Model XAxisModel;
        Model ShoulderModel;
        Model BicepModel;
        Model ForearmRollModel;
        Model ForearmModel;
        Model WristModel;
        Model GripperModel;

        Arm arm;

        ControlMode currentMode, prevMode;
        int buttonInput;

    public:
        Simulator();
        ~Simulator();

        void Draw();
        void LoadModels();
        void UnloadModels();
        void ProcessInput();
        void Update(float delta);
        void Reset();
        void ToggleModes();
    
    private:
        void DrawArm(const JointPositions &angles);
        void DrawDHLinks();
};

#endif /*SIMULATOR_H*/
