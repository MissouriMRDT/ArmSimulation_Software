#include "Simulator.h"

#include <chrono>

constexpr auto FRAME_TIME = std::chrono::nanoseconds(1'000'000'000/60);

int main ()
{
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "ArmSimulation");
	SetTargetFPS(60);

	Simulator sim;

	auto then = std::chrono::system_clock::now();
	
	while (!WindowShouldClose())
	{
		auto now = std::chrono::system_clock::now();
		if (now - then >= FRAME_TIME) {
			sim.ProcessInput();
			float delta = FRAME_TIME.count() / 1'000'00'000.0;
			sim.Update(delta);
			sim.Draw();
			then = now;
		}
	}
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
*/