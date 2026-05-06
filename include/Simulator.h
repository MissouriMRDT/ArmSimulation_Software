#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <raylib.h>
#include <resource_dir.h>
#include <list>

#include "ArmParameters.h"
#include "InverseKinematics.h"
#include "RoveMatrix.h"
#include "Arm.h"

#define OPEN_LOOP_DUTY (INT16_MAX / 2)
#define CLOSED_LOOP_ANGULAR_SPEED 10.0f // degrees per second
#define CLOSED_LOOP_LINEAR_SPEED 0.5f // inches per second
#define IK_TARGET_SPEED 1.0f // inches per second
#define CONTROLLER_DEAD_ZONE 0.35f

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
        float axes[AXES_COUNT];
        bool useToolPose = false;
        bool snappingEnabled = true;

        int selectedGamepad = 0;

        Camera camera;
        Vector2 orbit;

        Model XAxisModel;
        Model ShoulderModel;
        Model BicepModel;
        Model ForearmRollModel;
        Model ForearmModel;
        Model WristModel;
        Model GripperModel;

        static constexpr size_t HISTORY_BUFFER_SIZE = 1024;
        std::list<Vector> targetHistory;
        std::list<Vector> positionHistory;

        Arm arm;

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
        void DrawHistoryBuffer(const std::list<Vector> &buffer, Color lineColor = BLACK);
        void DrawDHLinks(const IK::DHParameters links[6], Color linkColor = LIGHTGRAY, const TransfMatrix &transform = Identity());
        void DrawDHTable(const IK::DHParameters links[6], int x, int y, int width, int height);
};

#endif /*SIMULATOR_H*/
