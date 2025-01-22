#include "Simulator.h"

int main ()
{
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "ArmSimulation");
	SearchAndSetResourceDir("resources");
	SetTargetFPS(60);

	Camera camera;
	camera.position = (Vector3){ -200.0f, 100.0f, -200.0f };
	camera.target = (Vector3){ -200.0f, 100.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 90.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	DisableCursor();

	Simulator sim;
	
	while (!WindowShouldClose())
	{
		UpdateCamera(&camera, CAMERA_FREE);
		ClearBackground(WHITE);

		sim.TransformArm();
		sim.Keyboard();
		sim.Update();

		BeginDrawing();
		BeginMode3D(camera);
		sim.Draw();
	}

	sim.Unload();
	CloseWindow();
	return 0;
}

/*
TODO:
	Features:
		!! Lock mode
		Display angles and position
		Path of motion for keyboard typing?
		Need to add x axis and shoulder?

	Make xyz pos and joint angles input to IKcalc func, output desired joint angles
		Call in different file
		Make IK a library for both simulator and arm board
	Update arm software values for physical system

	Make emb software updated
	Make own vector struct instead
	Fix limits to be like real arm
*/