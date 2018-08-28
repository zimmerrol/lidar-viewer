#include "stdafx.h"
#include "scene.h"
#include "window.h"
#include <functional>
#include "oglw.h"

Scene::Scene(Window* parentWindow)
{
	_pParentWindow = parentWindow;

	updateCallback = std::bind(&Scene::update, this, std::placeholders::_2);
	parentWindow->addUpdateCallback(&updateCallback);

	renderCallback = std::bind(&Scene::render, this);
	parentWindow->addRenderCallback(&renderCallback);

	processKeyInputCallback = std::bind(&Scene::processKeyboardInput, this, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
	parentWindow->addProcessKeyInputCallback(&processKeyInputCallback);

	processMouseCallback = std::bind(&Scene::processMouseInput, this, std::placeholders::_2, std::placeholders::_3);
	parentWindow->addProcessMouseCallback(&processMouseCallback);

	processScrollCallback = std::bind(&Scene::processScroll, this, std::placeholders::_2, std::placeholders::_3);
	parentWindow->addProcessScrollCallback(&processScrollCallback);
}

Scene::~Scene()
{
}

Window * Scene::getParentWindow()
{
	return _pParentWindow;
}
