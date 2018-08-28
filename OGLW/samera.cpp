#include "stdafx.h"
#include "camera.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

Camera::Camera(glm::vec3 positionVector, glm::vec3 upVector, float yaw, float pitch, float movementSpeed, float mouseSensitivity, float zoom) : 
	_movementSpeed(movementSpeed), _mouseSensitivity(mouseSensitivity), _zoom(zoom), _frontVector(glm::vec3(0.0f, 0.0f, -1.0f))
{
	_positionVector = positionVector;
	_worldUpVector = upVector;
	_yaw = yaw;
	_pitch = pitch;

	_updateCameraVectors();
}

Camera::Camera(float positionX, float positionY, float positionZ, float upX, float upY, float upZ, float yaw, float pitch, float movementSpeed, float mouseSensitivity, float zoom) : 
	Camera(glm::vec3(positionX, positionY, positionZ), glm::vec3(upX, upY, upZ), yaw, pitch, movementSpeed, mouseSensitivity, zoom)
{
}

Camera::~Camera()
{
}

glm::mat4 Camera::GetViewMatrix() const
{
	return glm::lookAt(_positionVector, _positionVector + _frontVector, _upVector);
}

float Camera::GetZoom() const
{
	return _zoom;
}

float Camera::GetYaw() const
{
	return _yaw;
}

float Camera::GetPitch() const
{
	return _pitch;
}

glm::vec3 Camera::GetPositionVector() const
{
	return glm::vec3(_positionVector);
}

void Camera::ProcessMovement(CameraMovement direction, float deltaTime)
{
	float velocity = _movementSpeed * deltaTime;

	switch (direction)
	{
	case CameraMovement::Forward:
		_positionVector += _frontVector * velocity;
		break;
	case CameraMovement::Backward: 
		_positionVector -= _frontVector * velocity;
		break;
	case CameraMovement::Left:
		_positionVector -= _rightVector * velocity;
		break;
	case CameraMovement::Right:
		_positionVector += _rightVector * velocity;
		break;

	}
}

void Camera::ProcessRotation(float xOffset, float yOffset, bool constrainPitch)
{
	xOffset *= _mouseSensitivity;
	yOffset *= _mouseSensitivity;

	_yaw += xOffset;
	_pitch += yOffset;

	if (constrainPitch)
	{
		_pitch = std::max(std::min(_pitch, 89.0f), -89.0f);
	}

	_updateCameraVectors();
}

void Camera::ProcessZoom(float yoffset)
{
	if (_zoom >= 1.0f && _zoom <= 90.0f)
		_zoom -= yoffset;
	if (_zoom <= 1.0f)
		_zoom = 1.0f;
	if (_zoom >= 90.0f)
		_zoom = 90.0f;
}

void Camera::_updateCameraVectors()
{
	glm::vec3 front;
	front.x = cos(glm::radians(_yaw)) * cos(glm::radians(_pitch));
	front.y = sin(glm::radians(_pitch));
	front.z = sin(glm::radians(_yaw)) * cos(glm::radians(_pitch));
	_frontVector = glm::normalize(front);

	_rightVector = glm::normalize(glm::cross(_frontVector, _worldUpVector));
	_upVector = glm::normalize(glm::cross(_rightVector, _frontVector));
}
