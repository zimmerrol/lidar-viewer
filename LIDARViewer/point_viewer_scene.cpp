/*
MIT License

Copyright(c) 2018 Roland Zimmermann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "stdafx.h"
#include "point_viewer_scene.h"
#include "../OGLW/camera.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

PointViewerScene::PointViewerScene(Window* parentWindow, std::vector<float> pointData) : Scene(parentWindow)
{
	_pCamera = new Camera(0.0f, 0.0f, 3.0f);
	_pointData = pointData;

	glEnable(GL_PROGRAM_POINT_SIZE);

	_pShader = new Shader("shaders\\pointShader.vert", "shaders\\pointShader.frag");
	glUseProgram(_pShader->GetID());

	float vertices[] = {
		0.0f, 0.0f, 0.0f
	};

	unsigned int instanceVBO;
	glGenBuffers(1, &instanceVBO);
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * pointData.size(), &pointData[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &_vao);
	//glGenBuffers(1, &_vbo);

	glBindVertexArray(_vao);

	//glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	//glEnableVertexAttribArray(0);

	// also set instance data
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO); // this attribute comes from a different vertex buffer
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
	glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(6 * sizeof(float)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glVertexAttribDivisor(0, 1); // tell OpenGL this is an instanced vertex attribute.
	glVertexAttribDivisor(1, 1);
	glVertexAttribDivisor(2, 1);

	updateShader();
}

void PointViewerScene::updateShader()
{
	_viewMatrix = _pCamera->GetViewMatrix();
	_projectionMatrix = glm::perspective(glm::radians(_pCamera->GetZoom()), (float)500 / (float)500, 0.1f, 100.0f);

	_pShader->setFloat("saturation", 1.0f);
	_pShader->setMat4("viewMatrix", _viewMatrix);
	_pShader->setMat4("projectionMatrix", _projectionMatrix);
}

PointViewerScene::~PointViewerScene()
{
	delete _pCamera;
}

void PointViewerScene::update(float deltatime)
{
	if (_keys[GLFW_KEY_ESCAPE])
	{
		glfwSetWindowShouldClose(getParentWindow()->getInternalWindow(), true);
	}

	if (_keys[GLFW_KEY_W])
	{
		_pCamera->ProcessMovement(CameraMovement::Forward, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_S])
	{
		_pCamera->ProcessMovement(CameraMovement::Backward, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_A])
	{
		_pCamera->ProcessMovement(CameraMovement::Left, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_D])
	{
		_pCamera->ProcessMovement(CameraMovement::Right, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_SPACE])
	{
		_pCamera->ProcessMovement(CameraMovement::Up, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_LEFT_CONTROL])
	{
		_pCamera->ProcessMovement(CameraMovement::Down, deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_UP])
	{
		_pCamera->ProcessRotation(0, 500 * deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_DOWN])
	{
		_pCamera->ProcessRotation(0, -500 * deltatime);
		updateShader();
	}
	if (_keys[GLFW_KEY_LEFT])
	{
		_pCamera->ProcessRotation(-500 * deltatime, 0);
		updateShader();
	}
	if (_keys[GLFW_KEY_RIGHT])
	{
		_pCamera->ProcessRotation(500 * deltatime, 0);
		updateShader();
	}
}

void PointViewerScene::render() const
{
	/* Render here */
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // also clear the depth buffer now!

	_pShader->use();
	glBindVertexArray(_vao);
	glDrawArraysInstanced(GL_POINTS, 0, 1, _pointData.size() / 7); // NPOINTS points of 3 vertices each
	glBindVertexArray(0);
}

void PointViewerScene::processKeyboardInput(int key, int scancode, int action, int mode)
{
	if (action == GLFW_PRESS)
		_keys[key] = true;
	else if (action == GLFW_RELEASE)
		_keys[key] = false;
}

void PointViewerScene::processMouseInput(double mousex, double mousey)
{
	if (_firstMouseMovement)
	{
		_lastMouseX = mousex;
		_lastMouseY = mousey;
		_firstMouseMovement = false;
	}

	float xoffset = mousex - _lastMouseX;
	float yoffset = _lastMouseY - mousey; // reversed since y-coordinates go from bottom to top

	_lastMouseX = mousex;
	_lastMouseY = mousey;

	_pCamera->ProcessRotation(xoffset, yoffset);

	updateShader();
}

void PointViewerScene::processScroll(double scrollx, double scrolly)
{
	_pCamera->ProcessZoom(scrolly);

	updateShader();
}
