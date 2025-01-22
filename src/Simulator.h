#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include "raylib.h"
#include "raymath.h"
#include "resource_dir.h"
#include "InverseKinematics.h"
#include "RoveMatrix.h"

#define SPEED 0.2f
#define SPD_MOD2 2.0f
#define SPD_MOD4 4.0f

class Simulator {
    private:

        Vector WristPos;
        Vector GripperPos;

        struct joint {
            float qMotor; //in degrees
            float qTarget; //in degrees
            float FwdLim; //in degrees
            float RevLim; //in degrees
            int button;
            TransfMatrix transf;
            Model model;
        };

        struct Wrist {
            float j4;
            float pitch;
            float valk;
        };

        joint J1, J2, J3, J4, Pitch, Valkyrie, Solenoid;
        Wrist wrist;

        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP,
            INVERSE_KINEMATICS
        };

        bool underMode, lockMode, limsOverride, direction;
        ControlMode currentMode, prevMode;
        int buttonInput;

    public:
        Simulator() {
            WristPos = {0, 0, 0};
            GripperPos = {0, 0, 0};
            wrist = {0, 0, 0};

            J1 = {0, 0, J1_FWD_LIM, J1_REV_LIM, 1};
            J2 = {60, 0, J2_FWD_LIM, J2_REV_LIM, 2};
            J3 = {-100, 0, J3_POS_LIM, J3_NEG_LIM, 3};
            J4 = {0, 0, J4_FWD_LIM, J4_REV_LIM, 4};
            Pitch = {30, 0, PITCH_FWD_LIM, PITCH_REV_LIM, 5};
            Valkyrie = {0, 0, 1000, -1000, 6};
 
            underMode = lockMode = 0;
            limsOverride = false;
            direction = false;
            buttonInput = 0;
            currentMode = OPEN_LOOP;
            prevMode = OPEN_LOOP;
            J2.model = LoadModel("J2Model.obj");
            J3.model = LoadModel("J3Model.obj");
            J4.model = LoadModel("J4Model.obj");
            Pitch.model = LoadModel("PitchModel.obj");
            Valkyrie.model = LoadModel("ValkModel.obj");
            Solenoid.model = LoadModel("SolModel.obj");
        }

        void Draw();
        void Unload();
        void TransformArm();
        void Keyboard();
        bool atFwdLim(joint J);
        bool atRevLim(joint J);
        void LimitJoint(joint &J);
        void UpdateJoint(joint &J);
        void Update();
        void HoldCurrentPosition();
        void UpdateRayLibMatrix(joint &J);
};

#endif /*SIMULATOR_H*/