#include "Simulator.h"

void Simulator::Draw() 
{
	DrawModel(J2.model, (Vector3){0.0f, 0.0f, 0.0f }, 1.0f, RED);		
	DrawModel(J3.model, (Vector3){0, 0, 0}, 1.0f, BLUE);
	DrawModel(J4.model, (Vector3){0, 0, 0}, 1.0f, ORANGE);
	DrawModel(Pitch.model, (Vector3){0, 0, 0}, 1.0f, PURPLE);
	DrawModel(Valkyrie.model, (Vector3){0, 0, 0}, 1.0f, GREEN);
	DrawModel(Solenoid.model, (Vector3){0, 0, 0}, 1.0f, MAROON);
	DrawSphere({GripperPos.x, GripperPos.y, GripperPos.z}, 2, MAROON);
	DrawGrid(100, 10.0f);

	EndMode3D();

	DrawText("Arm Simulation", 5,5,20,BLACK);
	DrawText("CM: ", 5,700,20,BLACK);
	if (currentMode == OPEN_LOOP) DrawText("O", 150,700,20,BLACK);
	else if (currentMode == CLOSED_LOOP) DrawText("C", 150,700,20,BLACK);
	else DrawText("IK", 150,700,20,BLACK);
	if (lockMode) DrawText("LOCK", 80,700,20,BLACK);
	DrawText("X: ", 5,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.x), 40,650,20,BLACK);
	DrawText("Y: ", 150,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.y), 190,650,20,BLACK);
	DrawText("P: ", 5,600,20,BLACK);
	DrawText(TextFormat("%.2f", Pitch.qMotor), 60,600,20,BLACK);
	// DrawText("Valkyriet: ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*Valkyrie.qTarget), 210,600,20,BLACK);

	EndDrawing();
}

void Simulator::Unload() 
{
	UnloadModel(J2.model);
	UnloadModel(J3.model);
	UnloadModel(J4.model);
	UnloadModel(Pitch.model);
	UnloadModel(Valkyrie.model);
	UnloadModel(Solenoid.model);
}

void Simulator::TransformArm() 
{
	J2.transf = Rotate(0,0, -J2.qMotor*DEG2RAD) * Translate(0,0, INTOPIXELS*J1.qMotor);
	J3.transf = Rotate(0, 0, -J3.qMotor*DEG2RAD) * Translate(-J2_LENGTH*INTOPIXELS, 0, 0) * J2.transf;
	J4.transf = Rotate((-J4.qMotor)*DEG2RAD, 0,0) * Translate(-SHOULDER_LENGTH*INTOPIXELS, 0, 0) * J3.transf;
	Pitch.transf = Rotate(0,0, -Pitch.qMotor*DEG2RAD) * Translate(-(J3_LENGTH-SHOULDER_LENGTH)*INTOPIXELS, 0, 0) * J4.transf;
	Valkyrie.transf = Rotate(-Valkyrie.qMotor*DEG2RAD, 0,0) * Translate(-WRIST_RAD*INTOPIXELS, 0, 0) * Pitch.transf;
	Solenoid.transf = Rotate(0, M_PI, 0) * Translate(WRIST_RAD*INTOPIXELS, 0, 0) * Pitch.transf;

	UpdateRayLibMatrix(J2);
	UpdateRayLibMatrix(J3);
	UpdateRayLibMatrix(J4);
	UpdateRayLibMatrix(Pitch);
	UpdateRayLibMatrix(Valkyrie);
	UpdateRayLibMatrix(Solenoid);
}

void Simulator::UpdateRayLibMatrix(joint &J)
{
	J.model.transform.m0 = J.transf.m0;
	J.model.transform.m1 = J.transf.m1;
	J.model.transform.m2 = J.transf.m2;
	J.model.transform.m3 = J.transf.m3;
	J.model.transform.m4 = J.transf.m4;
	J.model.transform.m5 = J.transf.m5;
	J.model.transform.m6 = J.transf.m6;
	J.model.transform.m7 = J.transf.m7;
	J.model.transform.m8 = J.transf.m8;
	J.model.transform.m9 = J.transf.m9;
	J.model.transform.m10 = J.transf.m10;
	J.model.transform.m11 = J.transf.m11;
	J.model.transform.m12 = J.transf.m12;
	J.model.transform.m13 = J.transf.m13;
	J.model.transform.m14 = J.transf.m14;
	J.model.transform.m15 = J.transf.m15;
}

void Simulator::Keyboard() 
{
	if (IsKeyPressed(KEY_M)) {
		if (currentMode == OPEN_LOOP) currentMode = CLOSED_LOOP;
		else if (currentMode == CLOSED_LOOP) currentMode = INVERSE_KINEMATICS;
		else currentMode = OPEN_LOOP;
	}
	
	if (currentMode == OPEN_LOOP) {
		lockMode = false;
		if(IsKeyPressed(KEY_ZERO)) direction = direction? false : true;
		if(IsKeyDown(KEY_ONE)) buttonInput = 1; //X
		else if(IsKeyDown(KEY_TWO)) buttonInput = 2; //J2
		else if(IsKeyDown(KEY_THREE)) buttonInput = 3; //J3
		else if(IsKeyDown(KEY_FOUR)) buttonInput = 4; //J4
		else if(IsKeyDown(KEY_FIVE)) buttonInput = 5; //Pitch
		else if(IsKeyDown(KEY_SIX)) buttonInput = 6; //Roll Valkyrie
		else buttonInput = 0;
	} else if (currentMode == INVERSE_KINEMATICS) {
		if(IsKeyDown(KEY_UP)) WristPos.y += SPEED;
		if(IsKeyDown(KEY_DOWN)) WristPos.y -= SPEED;
		if(IsKeyDown(KEY_LEFT)) WristPos.x -= SPEED;
		if(IsKeyDown(KEY_RIGHT)) WristPos.x += SPEED;		
		if(IsKeyDown(KEY_Z)) WristPos.z -= SPEED;
		if(IsKeyDown(KEY_C)) WristPos.z += SPEED;
		if(IsKeyDown(KEY_R)) wrist.j4 += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_F)) wrist.j4 -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_E)) wrist.pitch += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_Q)) wrist.pitch -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_V)) wrist.valk += (SPEED*SPD_MOD4);
		if(IsKeyPressed(KEY_L)) lockMode = lockMode? false : true;
	} else if (currentMode == CLOSED_LOOP) {
		lockMode = false;
		if(IsKeyDown(KEY_UP)) J2.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_DOWN)) J2.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_LEFT)) J3.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_RIGHT)) J3.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_Z)) J1.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_C)) J1.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_R)) J4.qTarget -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_F)) J4.qTarget += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_E)) Pitch.qTarget += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_Q)) Pitch.qTarget -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_V)) Valkyrie.qTarget += (SPEED*SPD_MOD4);
	}
	
	if(IsKeyDown(KEY_P)) {
		WristPos.x = 10;
		WristPos.y = 0;
		WristPos.z = 0;
		wrist.j4 = 0;
		wrist.pitch = 0;
		wrist.valk = 0;
		underMode = false;
	}
}

bool Simulator::atFwdLim(joint J) 
{
	if (limsOverride) return false;
	if (J.RevLim > J.FwdLim) {
		return (J.qMotor >= J.FwdLim) && (J.qMotor <= (J.RevLim + J.FwdLim)/2);
	}
	return J.qMotor >= J.FwdLim;
}

bool Simulator::atRevLim(joint J) 
{
	if (limsOverride) return false;
	if (J.RevLim > J.FwdLim) {
		return (J.qMotor <= J.RevLim) && (J.qMotor >= (J.RevLim + J.FwdLim)/2);
	}
	return J.qMotor <= J.RevLim;
}

void Simulator::LimitJoint(joint &J) 
{
	if (atFwdLim(J)) J.qMotor = J.FwdLim;
	if (atRevLim(J)) J.qMotor = J.RevLim;
}

void Simulator::UpdateJoint(joint &J) 
{
	if (currentMode == CLOSED_LOOP) { //Limit closed loop target angles
		if (J.qTarget > J.FwdLim) J.qTarget = J.FwdLim;
		else if (J.qTarget < J.RevLim) J.qTarget = J.RevLim;
	}

	if (buttonInput == J.button) {
		direction? J.qMotor += (SPEED*2) : J.qMotor -= (SPEED*2); //Set decipercent
	} else if ((currentMode == CLOSED_LOOP) || (currentMode == INVERSE_KINEMATICS)) {
		J.qMotor = J.qTarget; //SetAngle()
	}
	LimitJoint(J);
}

void Simulator::Update() 
{
	if (((currentMode == CLOSED_LOOP) || (currentMode == INVERSE_KINEMATICS)) && (currentMode != prevMode)) HoldCurrentPosition();
	if (currentMode == INVERSE_KINEMATICS) {
		if (!CalculateInverseKinematics(Valkyrie.transf, WristPos, GripperPos, wrist.j4, wrist.pitch, wrist.valk, J1.qTarget, J2.qTarget, J3.qTarget, J4.qTarget, Pitch.qTarget, Valkyrie.qTarget, J3.FwdLim, J3.RevLim, lockMode)) HoldCurrentPosition();
	} else {
		J3.FwdLim = J3_POS_LIM;
		J3.RevLim = J3_NEG_LIM;
	}
	UpdateJoint(J1);
	UpdateJoint(J2);
	UpdateJoint(J3);
	UpdateJoint(J4);
	UpdateJoint(Pitch);
	UpdateJoint(Valkyrie);

	prevMode = currentMode;

}

void Simulator::HoldCurrentPosition() 
{
	WristPos = {0,0,0};
	WristPos = WristPos * (Translate(J3_LENGTH, 0, 0) * Rotate(0,0, J3.qMotor*DEG2RAD) * Translate(J2_LENGTH, 0, 0) * Rotate(0,0, J2.qMotor*DEG2RAD) * Translate(0,0, J1.qMotor));

	GripperPos = {0,0,0}; //pixels
	GripperPos = GripperPos * (Translate(-VALK_LENGTH*INTOPIXELS, 0, 0) * Valkyrie.transf);

	wrist.pitch = Pitch.qMotor + J2.qMotor + J3.qMotor;
	wrist.j4 = J4.qMotor;
	wrist.valk = Valkyrie.qMotor;

	J1.qTarget = J1.qMotor;
	J2.qTarget = J2.qMotor;
	J3.qTarget = J3.qMotor;
	J4.qTarget = J4.qMotor;
	Pitch.qTarget = Pitch.qMotor;
	Valkyrie.qTarget = Valkyrie.qMotor;
}
