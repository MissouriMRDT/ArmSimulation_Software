#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include <raylib.h>
#include <resource_dir.h>

#include "InverseKinematics.h"
#include "RoveMatrix.h"
#include "Arm.h"

#define OPEN_LOOP_DUTY (INT16_MAX / 2)
#define CLOSED_LOOP_SPEED 0.5f
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
    D_PAD_Y = 7
};

class Simulator {
    private:

        Vector wristTarget;
        JointPositions targetAngles;
        float axes[8];

        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP,
            INVERSE_KINEMATICS
        };

        Camera camera;
        Model XAxisModel;
        Model ShoulderModel;
        Model BicepModel;
        Model ForearmRollModel;
        Model ForearmModel;
        Model WristModel;
        Model GripperModel;

        Arm arm;

        bool underMode, lockMode, limsOverride, direction;
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
};

#endif /*SIMULATOR_H*/
