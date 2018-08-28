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
#include <glm/glm.hpp>

enum CameraMovement
{
	Forward = 0,
	Backward = 1,
	Left = 2,
	Right = 4,
	Up = 8,
	Down = 16
};

class Camera
{
public:
	Camera(glm::vec3 positionVector = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 upVector = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = 0.0f, float pitch = 0.0f,
		float movementSpeed = 2.0, float mouseSensitivity = 0.1f, float zoom = 45.0f);
	Camera(float positionX, float positionY, float positionZ, float upX=0.0f, float upY=1.0f, float upZ=0.0f, float yaw=0.0f, float pitch=0.0f,
		float movementSpeed=2.0, float mouseSensitivity=0.1f, float zoom=45.0f);
	~Camera();
	glm::mat4 GetViewMatrix() const;
	float GetZoom() const;
	float GetYaw() const;
	float GetPitch() const;
	glm::vec3 GetPositionVector() const;

	void ProcessMovement(CameraMovement direction, float deltaTime);
	void ProcessRotation(float xOffset, float yOffset, bool constrainPitch = true);
	void ProcessZoom(float yoffset);

protected:
	glm::vec3 _positionVector;
	glm::vec3 _frontVector;
	glm::vec3 _upVector;
	glm::vec3 _rightVector;
	glm::vec3 _worldUpVector;

	float _yaw;
	float _pitch;
	float _movementSpeed;
	float _mouseSensitivity;
	float _zoom;

	void _updateCameraVectors();
};

