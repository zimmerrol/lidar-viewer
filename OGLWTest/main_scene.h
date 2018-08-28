#pragma once
#include "../OGLW/scene.h"

class MainScene : public Scene
{
public:
	MainScene(Window* parentWindow);
	~MainScene();

	void render() const;
	void processKeyboardInput();
	void processMouseInput(double mousex, double mousey);
	void processScroll(double scrollx, double scrolly);
};

