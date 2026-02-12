#include "Simulator.h"
#include <raylib.h>
#include <raymath.h>

Simulator::Simulator() {
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "ArmSimulation");
	SetTargetFPS(60);

	camera.fovy = 70.0f;
	camera.projection = CAMERA_ORTHOGRAPHIC;
	orbit = {M_PI_2, 0};

	LoadModels();
	wristTarget = {0, 0, 0};

	buttonInput = 0;
	currentMode = OPEN_LOOP;
	prevMode = OPEN_LOOP;

	Reset();

	arm.driveTargetAngles(-6.33, 63, -90, 0, 56, 0);
}

Simulator::~Simulator() {
	UnloadModels();
	CloseWindow();
}

void Simulator::Draw() 
{
	JointPositions angles = arm.getJointPositions();

	ClearBackground(WHITE);
	BeginDrawing();
	BeginMode3D(camera);

	DrawArm(angles);
	if (currentMode == INVERSE_KINEMATICS) {
		DrawDHLinks();
	}

	Vector gripperPos = arm.getGripperCoordinates();
	DrawSphere({gripperPos.x, gripperPos.y, gripperPos.z}, 0.5, RED);
	if (currentMode == INVERSE_KINEMATICS) {
		DrawSphere({wristTarget.x, wristTarget.y, wristTarget.z}, 0.5, YELLOW);
	}

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

	DrawText("Joint Target:", 5,650,20,BLACK);
	DrawText(TextFormat("X:%.2f Y:%.2f Z:%.2f", wristTarget.x, wristTarget.y, wristTarget.z), 250,650,20,BLACK);
	DrawText("Joint Angles:", 5,670,20, BLACK);
	DrawText(TextFormat("X:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f J6:%.2f", angles.X, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6), 250,670,20,BLACK);

	// DrawText("P: ", 5,600,20,BLACK);
	// DrawText(TextFormat("%.2f", Pitch.qMotor), 60,600,20,BLACK);
	// DrawText("Valkyriet: ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*Valkyrie.qTarget), 210,600,20,BLACK);

	EndDrawing();
}

void Simulator::DrawArm(const JointPositions &angles) {
	XAxisModel.transform = MatrixTranslate(-SHOULDER_OVERHANG, 0, 0);
	ShoulderModel.transform = MatrixTranslate(0, 0, angles.X);
	BicepModel.transform = MatrixRotateZ(angles.J2*DEG2RAD) * MatrixTranslate(0, SHOULDER_LENGTH, 0) * ShoulderModel.transform;
	ForearmRollModel.transform = MatrixRotateZ(angles.J3*DEG2RAD) * MatrixTranslate(0, BICEP_LENGTH, 0) * BicepModel.transform;
	ForearmModel.transform = MatrixRotateX(angles.J4*DEG2RAD) * MatrixTranslate(FOREARM_ROLL_PARTIAL_LENGTH, FOREARM_ROLL_LENGTH, 0) * ForearmRollModel.transform;
	WristModel.transform = MatrixRotateZ(angles.J5*DEG2RAD) * MatrixTranslate(FOREARM_PARTIAL_LENGTH, 0, 0) * ForearmModel.transform;
	GripperModel.transform = MatrixRotateX(angles.J6*DEG2RAD) * MatrixTranslate(WRIST_LENGTH, 0, 0) * WristModel.transform;

	DrawModel(XAxisModel, {0.0f, 0.0f, 0.0f }, 1.0f, GRAY);		
	DrawModel(ShoulderModel, {0.0f, 0.0f, 0.0f }, 1.0f, RED);		
	DrawModel(BicepModel, {0, 0, 0}, 1.0f, BLUE);
	DrawModel(ForearmRollModel, {0, 0, 0}, 1.0f, ORANGE);
	DrawModel(ForearmModel, {0, 0, 0}, 1.0f, PURPLE);
	DrawModel(WristModel, {0, 0, 0}, 1.0f, GREEN);
	DrawModel(GripperModel, {0, 0, 0}, 1.0f, MAROON);
}


void Simulator::DrawDHLinks() {

	TransfMatrix forward = Identity(); // initial frame

	// Draw first joint manually
	TransfMatrix firstJoint = forward * Translation(0, 0, IK::DHTable[0].d);
	Vector firstPos = firstJoint * Vector{0, 0, 0};
	Vector zPlus = firstJoint * Vector{0, 0, 3};
	Vector xPlus = firstJoint * Vector{3, 0, 0};
	DrawCube({firstPos.x, firstPos.y, firstPos.z}, 2, 2, 2, LIGHTGRAY);
	DrawLine3D({firstPos.x, firstPos.y, firstPos.z}, {zPlus.x, zPlus.y, zPlus.z}, BLUE);
	DrawLine3D({firstPos.x, firstPos.y, firstPos.z}, {xPlus.x, xPlus.y, xPlus.z}, RED);
	
	Vector prev = firstPos;

	for (const auto &link : IK::DHTable) {
		forward = forward * IK::TransformFromDH(link);
		Vector curr = forward * Vector{0, 0, 0};
		Vector start = forward * Vector{0, 0, 1};
		Vector end = forward * Vector{0, 0, -1};
		zPlus = forward * Vector{0, 0, 3};
		xPlus = forward * Vector{3, 0, 0};
		DrawLine3D({prev.x, prev.y, prev.z}, {curr.x, curr.y, curr.z}, BLACK);
		DrawCylinderEx(
			{start.x, start.y, start.z},
			{end.x, end.y, end.z},
			1, 1, 10, LIGHTGRAY);
		DrawLine3D({curr.x, curr.y, curr.z}, {zPlus.x, zPlus.y, zPlus.z}, BLUE);
		DrawLine3D({curr.x, curr.y, curr.z}, {xPlus.x, xPlus.y, xPlus.z}, RED);
		prev = curr;
	}

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
	if (IsCursorHidden()) {
		orbit += GetMouseDelta() * 0.01;
		if (IsKeyPressed(KEY_ESCAPE)) EnableCursor();
	} else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		DisableCursor();
	}
	camera.position = {cos(orbit.x)*cos(orbit.y)*100, sin(orbit.y)*100 + 10, sin(orbit.x)*cos(orbit.y)*100};
	camera.target = {0, 10, 0};
	camera.up = {0, 1, 0};
	

	if (IsGamepadAvailable(0)) {
		// std::cout << GetGamepadName(0) << ": " << IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
		axes[LEFT_STICK_X] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_X);
		axes[LEFT_STICK_Y] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_Y);
		axes[RIGHT_STICK_X] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_X);
		axes[RIGHT_STICK_Y] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_Y);
		axes[BUMPERS] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1);
		axes[TRIGGERS] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_TRIGGER) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_TRIGGER);
		axes[D_PAD_X] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_LEFT);
		axes[D_PAD_Y] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_UP) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_DOWN);
		axes[LEFT_BUMPER_TRIGGER] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_TRIGGER);
		axes[RIGHT_BUMPER_TRIGGER] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_TRIGGER);

		if (IsGamepadButtonPressed(selectedGamepad, GAMEPAD_BUTTON_MIDDLE_RIGHT)) ToggleModes();
		if(IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE)) Reset();

	} else {
		axes[LEFT_STICK_X] = IsKeyDown(KEY_D) - IsKeyDown(KEY_A);
		axes[LEFT_STICK_Y] = IsKeyDown(KEY_W) - IsKeyDown(KEY_S);
		axes[RIGHT_STICK_X] = IsKeyDown(KEY_L) - IsKeyDown(KEY_J);
		axes[RIGHT_STICK_Y] = IsKeyDown(KEY_I) - IsKeyDown(KEY_K);
		axes[BUMPERS] = IsKeyDown(KEY_U) - IsKeyDown(KEY_E);
		axes[TRIGGERS] = IsKeyDown(KEY_O) - IsKeyDown(KEY_Q);
		axes[D_PAD_X] = IsKeyDown(KEY_RIGHT) - IsKeyDown(KEY_LEFT);
		axes[D_PAD_Y] = IsKeyDown(KEY_UP) - IsKeyDown(KEY_DOWN);
		axes[LEFT_BUMPER_TRIGGER] = IsKeyDown(KEY_U) - IsKeyDown(KEY_O);
		axes[RIGHT_BUMPER_TRIGGER] = IsKeyDown(KEY_E) - IsKeyDown(KEY_Q);

		if (IsKeyPressed(KEY_M)) ToggleModes();
		if(IsKeyPressed(KEY_P)) Reset();
	}

}

void Simulator::Update(float delta) 
{	
	if (currentMode == OPEN_LOOP) {
		arm.driveOpenLoop(
			axes[RIGHT_STICK_X] * OPEN_LOOP_DUTY,
			axes[RIGHT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_X] * OPEN_LOOP_DUTY,
			axes[TRIGGERS] * OPEN_LOOP_DUTY,
			axes[BUMPERS] * OPEN_LOOP_DUTY
		);
	} else if (currentMode == INVERSE_KINEMATICS) {
		wristTarget.x += axes[RIGHT_STICK_Y] * IK_TARGET_SPEED * delta;
		wristTarget.y += axes[LEFT_STICK_Y] * IK_TARGET_SPEED * delta;
		wristTarget.z += axes[RIGHT_STICK_X] * IK_TARGET_SPEED * delta;
		targetAngles.J4 += axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J5 += axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J6 += axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		arm.driveInverseKinematics(
			wristTarget.x, 
			wristTarget.y,
			wristTarget.z,
			targetAngles.J4,
			targetAngles.J5,
			targetAngles.J6
		);
		
	} else if (currentMode == CLOSED_LOOP) {
		targetAngles.X += axes[RIGHT_STICK_X] * CLOSED_LOOP_LINEAR_SPEED * delta;
		targetAngles.J2 += axes[RIGHT_STICK_Y] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J3 += axes[LEFT_STICK_Y] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J4 += axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J5 += axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta;
		targetAngles.J6 += axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta;
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
	wristTarget.x = FOREARM_LENGTH;
	wristTarget.y = SHOULDER_LENGTH + BICEP_LENGTH + FOREARM_ROLL_LENGTH;
	wristTarget.z = -6.33;

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

void Simulator::ToggleModes() {
	if (currentMode == OPEN_LOOP) {
		currentMode = CLOSED_LOOP;
		targetAngles = arm.getJointPositions();
	} else if (currentMode == CLOSED_LOOP) {
		currentMode = INVERSE_KINEMATICS;
		wristTarget = arm.getGripperCoordinates();
	} else {
		currentMode = OPEN_LOOP;
	}
}
