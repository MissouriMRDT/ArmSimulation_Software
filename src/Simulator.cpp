#include "Simulator.h"
#include <raylib.h>
#include <raymath.h>

Simulator::Simulator() {
	camera.position = { 10, 15, 20 };
	camera.target = { 10, 15, 0 };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 90.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	// DisableCursor();

	LoadModels();
	wristTarget = {0, 0, 0};

	underMode = lockMode = 0;
	limsOverride = false;
	direction = false;
	buttonInput = 0;
	currentMode = OPEN_LOOP;
	prevMode = OPEN_LOOP;

	Reset();
}

Simulator::~Simulator() {
	UnloadModels();
}

void Simulator::Draw() 
{
	JointPositions angles = arm.getJointPositions();
	XAxisModel.transform = MatrixIdentity();
	ShoulderModel.transform = MatrixTranslate(SHOULDER_OVERHANG, 0, angles.X);
	BicepModel.transform = MatrixRotateZ(-angles.J2*DEG2RAD) * MatrixTranslate(0, SHOULDER_LENGTH, 0) * ShoulderModel.transform;
	ForearmRollModel.transform = MatrixRotateZ(-angles.J3*DEG2RAD) * MatrixTranslate(0, BICEP_LENGTH, 0) * BicepModel.transform;
	ForearmModel.transform = MatrixRotateX(angles.J4*DEG2RAD) * MatrixTranslate(FOREARM_ROLL_PARTIAL_LENGTH, FOREARM_ROLL_LENGTH, 0) * ForearmRollModel.transform;
	WristModel.transform = MatrixRotateZ(angles.J5*DEG2RAD) * MatrixTranslate(FOREARM_PARTIAL_LENGTH, 0, 0) * ForearmModel.transform;
	GripperModel.transform = MatrixRotateX(angles.J6) * MatrixTranslate(WRIST_LENGTH, 0, 0) * WristModel.transform;

	UpdateCamera(&camera, CAMERA_CUSTOM);

	ClearBackground(WHITE);
	BeginDrawing();
	BeginMode3D(camera);

	DrawModel(XAxisModel, {0.0f, 0.0f, 0.0f }, 1.0f, GRAY);		
	DrawModel(ShoulderModel, {0.0f, 0.0f, 0.0f }, 1.0f, RED);		
	DrawModel(BicepModel, {0, 0, 0}, 1.0f, BLUE);
	DrawModel(ForearmRollModel, {0, 0, 0}, 1.0f, ORANGE);
	DrawModel(ForearmModel, {0, 0, 0}, 1.0f, PURPLE);
	DrawModel(WristModel, {0, 0, 0}, 1.0f, GREEN);
	DrawModel(GripperModel, {0, 0, 0}, 1.0f, MAROON);
	Vector gripperPos = arm.getGripperCoordinates();
	DrawSphere({gripperPos.x, gripperPos.y, gripperPos.z}, 2, MAROON);

	DrawCube({0, 0, 0}, 1, 1, 1, LIGHTGRAY);
	DrawCube({1, 0, 0}, 0.2, 0.2, 0.2, RED);
	DrawCube({0, 1, 0}, 0.2, 0.2, 0.2, BLUE);
	DrawCube({0, 0, 1}, 0.2, 0.2, 0.2, LIME);
	DrawGrid(100, 10.0f);

	EndMode3D();

	DrawText("Arm Simulation", 5,5,20,BLACK);
	DrawText("CM: ", 5,700,20,BLACK);
	if (currentMode == OPEN_LOOP) DrawText("O", 150,700,20,BLACK);
	else if (currentMode == CLOSED_LOOP) DrawText("C", 150,700,20,BLACK);
	else DrawText("IK", 150,700,20,BLACK);
	if (lockMode) DrawText("LOCK", 80,700,20,BLACK);
	DrawText("X: ", 5,650,20,BLACK);
	DrawText(TextFormat("%.2f", wristTarget.x), 40,650,20,BLACK);
	DrawText("Y: ", 150,650,20,BLACK);
	DrawText(TextFormat("%.2f", wristTarget.y), 190,650,20,BLACK);
	// DrawText("P: ", 5,600,20,BLACK);
	// DrawText(TextFormat("%.2f", Pitch.qMotor), 60,600,20,BLACK);
	// DrawText("Valkyriet: ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*Valkyrie.qTarget), 210,600,20,BLACK);

	EndDrawing();
}

void Simulator::LoadModels() {
	SearchAndSetResourceDir("resources");
	XAxisModel = LoadModel("Athena/XAxis.obj");
	ShoulderModel = LoadModel("Athena/Shoulder.obj");
	BicepModel = LoadModel("Athena/Bicep.obj");
	ForearmRollModel = LoadModel("Athena/Forearm_Roll.obj");
	ForearmModel = LoadModel("Athena/Forearm.obj");
	WristModel = LoadModel("Athena/Wrist.obj");
	GripperModel = LoadModel("Athena/Gripper.obj");
}

void Simulator::UnloadModels() 
{
	UnloadModel(XAxisModel);
	UnloadModel(ShoulderModel);
	UnloadModel(BicepModel);
	UnloadModel(ForearmRollModel);
	UnloadModel(ForearmModel);
	UnloadModel(WristModel);
	UnloadModel(GripperModel);
}

void Simulator::ProcessInput() 
{
	axes[LEFT_STICK_X] = IsKeyDown(KEY_A) - IsKeyDown(KEY_D);
	axes[LEFT_STICK_Y] = IsKeyDown(KEY_W) - IsKeyDown(KEY_S);
	axes[RIGHT_STICK_X] = IsKeyDown(KEY_J) - IsKeyDown(KEY_L);
	axes[RIGHT_STICK_Y] = IsKeyDown(KEY_I) - IsKeyDown(KEY_K);
	axes[BUMPERS] = IsKeyDown(KEY_U) - IsKeyDown(KEY_E);
	axes[TRIGGERS] = IsKeyDown(KEY_O) - IsKeyDown(KEY_Q);
	axes[D_PAD_X] = IsKeyDown(KEY_RIGHT) - IsKeyDown(KEY_LEFT);
	axes[D_PAD_Y] = IsKeyDown(KEY_UP) - IsKeyDown(KEY_DOWN);

	if (IsKeyPressed(KEY_M)) {
		if (currentMode == OPEN_LOOP) currentMode = CLOSED_LOOP;
		else if (currentMode == CLOSED_LOOP) currentMode = INVERSE_KINEMATICS;
		else currentMode = OPEN_LOOP;
	}

	if(currentMode == INVERSE_KINEMATICS && IsKeyPressed(KEY_L)) {
		lockMode = lockMode? false : true;
	}

	if(IsKeyPressed(KEY_P)) {
		Reset();
		underMode = false;
	}
}

void Simulator::Update(float delta) 
{	
	if (currentMode == OPEN_LOOP) {
		lockMode = false;
		arm.driveOpenLoop(
			axes[RIGHT_STICK_X] * OPEN_LOOP_DUTY,
			axes[RIGHT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_X] * OPEN_LOOP_DUTY,
			axes[TRIGGERS] * OPEN_LOOP_DUTY,
			axes[BUMPERS] * OPEN_LOOP_DUTY
		);
	} else if (currentMode == INVERSE_KINEMATICS) {
		wristTarget.x += axes[RIGHT_STICK_X] * CLOSED_LOOP_SPEED * delta;
		wristTarget.y += axes[RIGHT_STICK_Y] * CLOSED_LOOP_SPEED * delta;
		wristTarget.z += axes[LEFT_STICK_Y] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J4 += axes[LEFT_STICK_X] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J5 += axes[TRIGGERS] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J6 += axes[BUMPERS] * CLOSED_LOOP_SPEED * delta;
		
	} else if (currentMode == CLOSED_LOOP) {
		lockMode = false;
		targetAngles.X += axes[RIGHT_STICK_X] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J2 += axes[RIGHT_STICK_Y] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J3 += axes[LEFT_STICK_Y] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J4 += axes[LEFT_STICK_X] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J5 += axes[TRIGGERS] * CLOSED_LOOP_SPEED * delta;
		targetAngles.J6 += axes[BUMPERS] * CLOSED_LOOP_SPEED * delta;
		arm.driveTargetAngles(
			targetAngles.X,
			targetAngles.J2,
			targetAngles.J3,
			targetAngles.J4,
			targetAngles.J5,
			targetAngles.J6
		);
	}
	arm.update(delta);
}

void Simulator::Reset() {
	wristTarget.x = 10;
	wristTarget.y = 0;
	wristTarget.z = 0;

	targetAngles.X = 0;
	targetAngles.J2 = 0;
	targetAngles.J3 = 0;
	targetAngles.J4 = 0;
	targetAngles.J5 = 0;
	targetAngles.J6 = 0;

	arm.driveTargetAngles(
		targetAngles.X,
		targetAngles.J2,
		targetAngles.J3,
		targetAngles.J4,
		targetAngles.J5,
		targetAngles.J6
	);
} 
