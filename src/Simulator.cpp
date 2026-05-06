#include "Simulator.h"
#include "ArmParameters.h"
#include "InverseKinematics.h"
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

Simulator::Simulator() {
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "Arm Simulator v2.0");
	SetTargetFPS(60);

	camera.fovy = 70.0f;
	camera.projection = CAMERA_ORTHOGRAPHIC;
	orbit = {M_PI_2, 0};

	LoadModels();

	Reset();
}

Simulator::~Simulator() {
	UnloadModels();
	CloseWindow();
}

void Simulator::Draw() 
{
	JointPositions angles = arm.getJointPositions();
	Vector armTarget = arm.getTarget();

	ClearBackground(WHITE);
	BeginDrawing();
	BeginMode3D(camera);

	DrawArm(angles);
	if (arm.getCurrentMode() == ControlMode::IK_POSE || arm.getCurrentMode() == ControlMode::IK_WRIST) {
		rlDisableDepthTest();
		DrawDHLinks(IK::DHTable);
		// DrawDHLinks(IK::DHTable, GREEN, Translation(6, 0, 24));
		rlEnableDepthTest();
		DrawSphere({armTarget.x, armTarget.y, armTarget.z}, 0.5, YELLOW);
		targetHistory.push_back(armTarget);
		if (targetHistory.size() >= HISTORY_BUFFER_SIZE) targetHistory.pop_front();
		DrawHistoryBuffer(targetHistory, GREEN);
	}

	Vector gripperPos = arm.getGripperCoordinates();
	DrawSphere({gripperPos.x, gripperPos.y, gripperPos.z}, 0.5, RED);
	positionHistory.push_back(gripperPos);
	if (positionHistory.size() >= HISTORY_BUFFER_SIZE) positionHistory.pop_front();
	DrawHistoryBuffer(positionHistory, RED);
	
	DrawCube({0, 0, 0}, 1, 1, 1, LIGHTGRAY);
	DrawCube({1, 0, 0}, 0.2, 0.2, 0.2, RED);
	DrawCube({0, 1, 0}, 0.2, 0.2, 0.2, BLUE);
	DrawCube({0, 0, 1}, 0.2, 0.2, 0.2, LIME);
	DrawGrid(100, 10.0f);

	EndMode3D();
	
	DrawText("Athena Arm Simulator", 5, 5, 20, BLACK);
	DrawText("CM: ", 5, 700, 20, BLACK);

	switch (arm.getCurrentMode()) {
		case ControlMode::OPEN_LOOP: DrawText("O", 150, 700, 20, BLACK); break;
		case ControlMode::CLOSED_LOOP: DrawText("C", 150, 700, 20, BLACK); break;
		case ControlMode::IK_POSE:
			DrawText(TextFormat("POSE (%s, %s)", useToolPose ? "TOOL" : "WORLD", snappingEnabled ? "SNAP" : "NOSNAP"), 150, 700, 20, BLACK);
			break;
		case ControlMode::IK_WRIST: DrawText("WRIST", 150, 700, 20, BLACK); break;
	}
	
	DrawText("Joint Target:", 5, 650, 20,BLACK);
	DrawText(TextFormat("X:%.2f Y:%.2f Z:%.2f", armTarget.x, armTarget.y, armTarget.z), 250, 650, 20, BLACK);
	DrawText("Joint Angles:", 5, 670, 20, BLACK);
	int offset = 0;
	DrawText(TextFormat("X:%04.2f", angles.X), 250 + 110*offset++, 670, 20, BLACK);
	DrawText(TextFormat("J2:%04.2f", angles.J2), 250 + 110*offset++, 670, 20, BLACK);
	DrawText(TextFormat("J3:%04.2f", angles.J3), 250 + 110*offset++, 670, 20, BLACK);
	DrawText(TextFormat("J4:%04.2f", angles.J4), 250 + 110*offset++, 670, 20, BLACK);
	DrawText(TextFormat("J5:%04.2f", angles.J5), 250 + 110*offset++, 670, 20, BLACK);
	DrawText(TextFormat("J6:%04.2f", angles.J6), 250 + 110*offset++, 670, 20, BLACK);
	
	if (arm.getCurrentMode() == ControlMode::IK_POSE || arm.getCurrentMode() == ControlMode::IK_WRIST) {
		DrawDHTable(IK::DHTable, 800, 100, 500, 400);
	}

	EndDrawing();
}

void Simulator::DrawHistoryBuffer(const std::list<Vector> &buffer, Color lineColor) {
	auto it2 = buffer.begin();
	for (auto it1 = it2++; it2 != buffer.end(); ++it1, ++it2) {
		DrawLine3D({it1->x, it1->y, it1->z}, {it2->x, it2->y, it2->z}, lineColor);
	}
}

void Simulator::DrawArm(const JointPositions &angles) {
	XAxisModel.transform = MatrixTranslate(-SHOULDER_OVERHANG, 0, 0) * MatrixRotateY(M_PI_2);
	ShoulderModel.transform = MatrixTranslate(SHOULDER_OVERHANG, 0, angles.X) * XAxisModel.transform;
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


void Simulator::DrawDHLinks(const IK::DHParameters links[6], Color linkColor, const TransfMatrix &transform) {

	// EVIL CODE

	TransfMatrix forward = IK::BASE_FRAME * transform; // initial frame

	// Draw first joint manually
	TransfMatrix firstJoint = forward * Translation(0, 0, links[0].d);
	Vector firstPos = firstJoint * ORIGIN;
	Vector zPlus = firstJoint * (3*BASIS_Z);
	Vector xPlus = firstJoint * (3*BASIS_X);
	DrawCube({firstPos.x, firstPos.y, firstPos.z}, 2, 2, 2, linkColor);
	DrawLine3D({firstPos.x, firstPos.y, firstPos.z}, {zPlus.x, zPlus.y, zPlus.z}, BLUE);
	DrawLine3D({firstPos.x, firstPos.y, firstPos.z}, {xPlus.x, xPlus.y, xPlus.z}, RED);
	
	Vector prev = firstPos;

	for (int i = 0; i < 6; i++) {
		forward = forward * IK::TransformFromDH(links[i]);
		Vector curr = forward * ORIGIN;
		Vector start = forward * BASIS_X;
		Vector end = forward * -BASIS_X;
		zPlus = forward * (3 * BASIS_Z);
		xPlus = forward * (3 * BASIS_X);
		DrawLine3D({prev.x, prev.y, prev.z}, {curr.x, curr.y, curr.z}, BLACK);
		DrawCylinderEx(
			{start.x, start.y, start.z},
			{end.x, end.y, end.z},
			1, 1, 10, linkColor);
		DrawLine3D({curr.x, curr.y, curr.z}, {zPlus.x, zPlus.y, zPlus.z}, BLUE);
		DrawLine3D({curr.x, curr.y, curr.z}, {xPlus.x, xPlus.y, xPlus.z}, RED);
		prev = curr;
	}
}

void Simulator::DrawDHTable(const IK::DHParameters links[6], int x, int y, int width, int height) {
	int lineSpacing = height/6;
	int itemSpacing = width/4;
	DrawText("DH Frame Table", x, y-50, 28, BLACK);
	for (int i = 0; i < 6; i++) {
		DrawLine(x, y + lineSpacing * i, x + width, y + lineSpacing * i, BLACK);
		int textPos = y + lineSpacing * (i+1) - 50;
		DrawText(TextFormat("d%d: %04.2f", i+1, links[i].d), x + itemSpacing*0, textPos, 20, BLACK);
		DrawText(TextFormat("theta%d: %04.2f", i+1, links[i].theta), x + int(itemSpacing*0.75), textPos, 20, BLACK);
		DrawText(TextFormat("a%d: %04.2f", i+1, links[i].a), x + itemSpacing*2, textPos, 20, BLACK);
		DrawText(TextFormat("alpha%d: %04.2f", i+1, links[i].alpha), x + int(itemSpacing*2.75), textPos, 20, BLACK);
	}
	DrawLine(x, y + lineSpacing * 6, x + width, y + lineSpacing * 6, BLACK);
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

float removeDeadZone(float axis, float deadZone) {
	return abs(axis) > deadZone ? axis : 0;
}

void Simulator::ProcessInput() 
{
	if (IsCursorHidden()) {
		orbit += GetMouseDelta() * 0.01;
		if (IsKeyPressed(KEY_ESCAPE)) EnableCursor();
	} else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		DisableCursor();
	}
	camera.position = {-cosf(orbit.x)*cosf(orbit.y)*100, sinf(orbit.y)*100 + 10, -sinf(orbit.x)*cosf(orbit.y)*100};
	camera.target = {0, 10, 0};
	camera.up = {0, 1, 0};
	

	if (IsGamepadAvailable(0)) {
		// invert y axes because stick forward is negative for some reason
		axes[LEFT_STICK_X] = removeDeadZone(GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_X), CONTROLLER_DEAD_ZONE);
		axes[LEFT_STICK_Y] = removeDeadZone(-GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_Y), CONTROLLER_DEAD_ZONE);
		axes[RIGHT_STICK_X] = removeDeadZone(GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_X), CONTROLLER_DEAD_ZONE);
		axes[RIGHT_STICK_Y] = removeDeadZone(-GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_Y), CONTROLLER_DEAD_ZONE);
		axes[BUMPERS] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1);
		axes[TRIGGERS] = GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_TRIGGER) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_TRIGGER);
		axes[D_PAD_X] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_LEFT);
		axes[D_PAD_Y] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_UP) - IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_FACE_DOWN);
		axes[LEFT_BUMPER_TRIGGER] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_LEFT_TRIGGER);
		axes[RIGHT_BUMPER_TRIGGER] = IsGamepadButtonDown(selectedGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) - GetGamepadAxisMovement(selectedGamepad, GAMEPAD_AXIS_RIGHT_TRIGGER);

		if (IsGamepadButtonPressed(selectedGamepad, GAMEPAD_BUTTON_MIDDLE_RIGHT)) ToggleModes();
		if(IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_LEFT)) useToolPose = !useToolPose;

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
		if (IsKeyPressed(KEY_T)) useToolPose = !useToolPose;
		if (IsKeyPressed(KEY_Z)) {
			snappingEnabled = !snappingEnabled;
			arm.setYZSnappingThreshold(snappingEnabled ? 0.15 : 0);
		}
	}
	
	if (IsKeyPressed(KEY_P)) Reset();
}

void Simulator::Update(float delta) 
{
	switch (arm.getCurrentMode()) {
	case ControlMode::OPEN_LOOP:
		arm.driveOpenLoop(
			axes[RIGHT_STICK_X] * OPEN_LOOP_DUTY,
			axes[RIGHT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_Y] * OPEN_LOOP_DUTY,
			axes[LEFT_STICK_X] * OPEN_LOOP_DUTY,
			axes[TRIGGERS] * OPEN_LOOP_DUTY,
			axes[BUMPERS] * OPEN_LOOP_DUTY
		);
		break;
	case ControlMode::CLOSED_LOOP:
		arm.incrementTargetAngles(
			axes[RIGHT_STICK_X] * CLOSED_LOOP_LINEAR_SPEED * delta,
			axes[RIGHT_STICK_Y] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[LEFT_STICK_Y] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta
		);
		break;
	case ControlMode::IK_POSE:
		if (useToolPose) {
			arm.incrementInverseKinematicsToolPose(
				axes[RIGHT_STICK_X] * IK_TARGET_SPEED * delta,
				axes[RIGHT_STICK_Y] * IK_TARGET_SPEED * delta,
				axes[LEFT_STICK_Y] * IK_TARGET_SPEED * delta,
				axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta,
				axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta,
				axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta
			);
		} else {
			arm.incrementInverseKinematicsWorldPose(
				axes[RIGHT_STICK_X] * IK_TARGET_SPEED * delta,
				axes[RIGHT_STICK_Y] * IK_TARGET_SPEED * delta,
				axes[LEFT_STICK_Y] * IK_TARGET_SPEED * delta,
				axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta,
				axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta,
				axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta
			);
		}
		break;
	case ControlMode::IK_WRIST:
		arm.incrementInverseKinematicsPosition(
			axes[RIGHT_STICK_X] * IK_TARGET_SPEED * delta,
			axes[RIGHT_STICK_Y] * IK_TARGET_SPEED * delta,
			axes[LEFT_STICK_Y] * IK_TARGET_SPEED * delta,
			axes[LEFT_STICK_X] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[TRIGGERS] * CLOSED_LOOP_ANGULAR_SPEED * delta,
			axes[BUMPERS] * CLOSED_LOOP_ANGULAR_SPEED * delta
		);
		break;
	}
	arm.update(delta);
}

void Simulator::Reset() {
	arm.driveTargetAngles(0, 0, 0, 0, 0, 0);
} 

void Simulator::ToggleModes() {
	switch (arm.getCurrentMode()) {
	case ControlMode::OPEN_LOOP:
		arm.incrementTargetAngles(0, 0, 0, 0, 0, 0);
		break;
	case ControlMode::CLOSED_LOOP:
		arm.incrementInverseKinematicsWorldPose(0, 0, 0, 0, 0, 0);
		break;
	case ControlMode::IK_POSE:
		arm.incrementInverseKinematicsPosition(0, 0, 0, 0, 0, 0);
		break;
	case ControlMode::IK_WRIST:
		arm.driveOpenLoop(0, 0, 0, 0, 0, 0);
		break;
	}
}
