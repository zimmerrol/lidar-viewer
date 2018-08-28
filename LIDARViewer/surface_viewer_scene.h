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

#pragma once

#include "../OGLW/scene.h"
#include "../OGLW/shader.h"
#include <glm/glm.hpp>
#include "../OGLW/mesh.h"

class Camera;

class SurfaceViewerScene : public Scene
{
public:
	SurfaceViewerScene(Window* parentWindow, boost::shared_ptr<std::vector<ColoredVertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices);
	SurfaceViewerScene(Window* parentWindow, boost::shared_ptr<std::vector<Vertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices);
	SurfaceViewerScene(Window* parentWindow, boost::shared_ptr<ColoredMesh> pMesh);
	~SurfaceViewerScene();

	// Inherited via Scene
	virtual void render() const override;
	virtual void processKeyboardInput(int key, int scancode, int action, int mode) override;
	virtual void processMouseInput(double mousex, double mousey) override;
	virtual void processScroll(double scrollx, double scrolly) override;
	virtual void update(float deltatime) override;

private:
	Camera * _pCamera;
	float _lastFrame;

	bool _firstMouseMovement = true;
	float _lastMouseX;
	float _lastMouseY;

	Shader* _pShader;

	unsigned int _vao, _vbo;

	// std::vector<float> _pointData;

	void updateShader();

	glm::mat4 _viewMatrix;
	glm::mat4 _projectionMatrix;

	boost::shared_ptr<MeshBase> _pMesh;

	bool _keys[1024];
};

