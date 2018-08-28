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
#include "surface_viewer_scene.h"
#include "../OGLW/camera.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

SurfaceViewerScene::SurfaceViewerScene(Window* parentWindow, boost::shared_ptr<std::vector<ColoredVertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices) : Scene(parentWindow)
{
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);

	_pCamera = new Camera(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);

	_pMesh = boost::shared_ptr<ColoredMesh>(new ColoredMesh(pVertices, pIndices));

	_pShader = new Shader("shaders\\surfaceShader.vert", "shaders\\surfaceShader.frag");
	_pShader->use();


	updateShader();
}

SurfaceViewerScene::SurfaceViewerScene(Window* parentWindow, boost::shared_ptr<std::vector<Vertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices) : Scene(parentWindow)
{
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);

	_pCamera = new Camera(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);

	_pMesh = boost::shared_ptr<Mesh>(new Mesh(pVertices, pIndices));

	_pShader = new Shader("shaders\\uncoloredSurfaceShader.vert", "shaders\\surfaceShader.frag");
	_pShader->use();

	updateShader();
}

SurfaceViewerScene::SurfaceViewerScene(Window * parentWindow, boost::shared_ptr<ColoredMesh> pMesh) : Scene(parentWindow)
{
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);

	_pCamera = new Camera(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);

	_pMesh = pMesh;

	_pShader = new Shader("shaders\\surfaceShader.vert", "shaders\\surfaceShader.frag");
	_pShader->use();

	updateShader();
}

void SurfaceViewerScene::updateShader()
{
	glm::vec3 pos = _pCamera->GetPositionVector();
	std::cout << pos.x << "," <<  pos.y << "," <<  pos.z << "; " << _pCamera->GetPitch() << "," << _pCamera->GetYaw() << std::endl;

	_viewMatrix = _pCamera->GetViewMatrix();
	_projectionMatrix = glm::perspective(glm::radians(_pCamera->GetZoom()), (float)getParentWindow()->GetWidth() / (float)getParentWindow()->GetHeight(), 0.1f, 100.0f);

	_pShader->setFloat("saturation", 1.0f);
	_pShader->setMat4("viewMatrix", _viewMatrix);
	_pShader->setMat4("projectionMatrix", _projectionMatrix);
}

void SurfaceViewerScene::update(float deltatime)
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

SurfaceViewerScene::~SurfaceViewerScene()
{
	delete _pCamera;
}

void SurfaceViewerScene::render() const
{
	/* Render here */
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // also clear the depth buffer now!

	_pShader->use();

	_pMesh->draw();
}

void SurfaceViewerScene::processKeyboardInput(int key, int scancode, int action, int mode)
{
	if (action == GLFW_PRESS)
		_keys[key] = true;
	else if (action == GLFW_RELEASE)
		_keys[key] = false;
}

void SurfaceViewerScene::processMouseInput(double mousex, double mousey)
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

void SurfaceViewerScene::processScroll(double scrollx, double scrolly)
{
	_pCamera->ProcessZoom(scrolly);

	updateShader();
}
