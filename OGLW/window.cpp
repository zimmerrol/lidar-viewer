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
#include "window.h"
#include <iostream>
#include <functional>
#include "oglw.h"

Window::Window(int width, int height, const char* title, ProcessFramebufferSizeCallback* framebufferSizeCallback, ProcessKeyInputCallback* processKeyInputCallback, RenderCallback* renderCallback,
	ProcessMouseCallback* mouseCallback, ProcessScrollCallback* scrollCallback)
{
	_width = width;
	_height = height;

	addFramebufferSizeCallback(framebufferSizeCallback);
	addProcessKeyInputCallback(processKeyInputCallback);
	addRenderCallback(renderCallback);
	addProcessMouseCallback(mouseCallback);
	addProcessScrollCallback(scrollCallback);


	/* Create a windowed mode window and its OpenGL context */
	_internalWindow = glfwCreateWindow(width, height, title, NULL, NULL);
	if (!_internalWindow)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
	}

	glfwSetWindowUserPointer(_internalWindow, this);

	/* Make the window's context current */
	glfwMakeContextCurrent(_internalWindow);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
	}

	glfwSetFramebufferSizeCallback(_internalWindow, _processFrameBufferSizeChanged);
	glfwSetCursorPosCallback(_internalWindow, _processMouseMoved);
	glfwSetScrollCallback(_internalWindow, _processScrolled);
	glfwSetKeyCallback(_internalWindow, _processKeyInput);

	_framebufferSizeChangedCallback = ProcessFramebufferSizeCallback([this](Window* window, int width, int height)
	{
		glViewport(0, 0, width, height);
		_width = width;
		_height = height;
	});
	addFramebufferSizeCallback(&_framebufferSizeChangedCallback);

}


Window::~Window()
{
}

void Window::run()
{
	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(_internalWindow))
	{
		float currentFrame = glfwGetTime();
		_deltaTime = currentFrame - _lastFrameTime;
		_lastFrameTime = currentFrame;

		/* Poll for and process events */
		glfwPollEvents();

		for (auto callback : _updateCallbacks)
		{
			if (callback)
				(*callback)(this, _deltaTime);
		}

		for (auto callback : _renderCallbacks)
		{
			if (callback)
				(*callback)(this);
		}

		/* Swap front and back buffers */
		glfwSwapBuffers(_internalWindow);
	}
}

GLFWwindow * Window::getInternalWindow()
{
	return _internalWindow;
}

void Window::_processFrameBufferSizeChanged(GLFWwindow * window, int width, int height)
{
	Window* thisWindow = (Window*)glfwGetWindowUserPointer(window);
	for (auto callback : thisWindow->_framebufferSizeCallbacks)
	{
		(*callback)(thisWindow, width, height);
	}
}

void Window::_processMouseMoved(GLFWwindow * window, double xpos, double ypos)
{
	Window* thisWindow = (Window*)glfwGetWindowUserPointer(window);
	for (auto callback : thisWindow->_processMouseCallbacks)
	{
		(*callback)(thisWindow, xpos, ypos);
	}
}

void Window::_processKeyInput(GLFWwindow * window, int key, int scancode, int action, int mode)
{
	Window* thisWindow = (Window*)glfwGetWindowUserPointer(window);
	for (auto callback : thisWindow->_processKeyInputCallbacks)
	{
		(*callback)(thisWindow, key, scancode, action, mode);
	}
}

void Window::_processScrolled(GLFWwindow * window, double xoffset, double yoffset)
{
	Window* thisWindow = (Window*)glfwGetWindowUserPointer(window);
	for (auto callback : thisWindow->_processScrollCallbacks)
	{
		(*callback)(thisWindow, xoffset, yoffset);
	}
}

void Window::addFramebufferSizeCallback(ProcessFramebufferSizeCallback* callback)
{
	if (callback)
		this->_framebufferSizeCallbacks.push_back(callback);
}

void Window::addProcessKeyInputCallback(ProcessKeyInputCallback* callback)
{
	if (callback)
		this->_processKeyInputCallbacks.push_back(callback);
}

void Window::addUpdateCallback(UpdateCallback* callback)
{
	if (callback)
		this->_updateCallbacks.push_back(callback);
}

void Window::addRenderCallback(RenderCallback* callback)
{
	if (callback)
		this->_renderCallbacks.push_back(callback);
}

void Window::addProcessMouseCallback(ProcessMouseCallback* callback)
{
	if (callback)
		this->_processMouseCallbacks.push_back(callback);
}

void Window::addProcessScrollCallback(ProcessScrollCallback* callback)
{
	if (callback)
		this->_processScrollCallbacks.push_back(callback);
}

void Window::removeFramebufferSizeCallback(ProcessFramebufferSizeCallback* callback)
{
	auto position = std::find(this->_framebufferSizeCallbacks.begin(), this->_framebufferSizeCallbacks.end(), callback);
	if (position != this->_framebufferSizeCallbacks.end())
		this->_framebufferSizeCallbacks.erase(position, position + 1);
}

void Window::removeProcessKeyInputCallbacks(ProcessKeyInputCallback* callback)
{
	auto position = std::find(this->_processKeyInputCallbacks.begin(), this->_processKeyInputCallbacks.end(), callback);
	if (position != this->_processKeyInputCallbacks.end())
		this->_processKeyInputCallbacks.erase(position, position + 1);
}

void Window::removeRenderCallbacks(RenderCallback* callback)
{
	auto position = std::find(this->_renderCallbacks.begin(), this->_renderCallbacks.end(), callback);
	if (position != this->_renderCallbacks.end())
		this->_renderCallbacks.erase(position, position + 1);
}

void Window::removeProcessMouseCallbacks(ProcessMouseCallback* callback)
{
	auto position = std::find(this->_processMouseCallbacks.begin(), this->_processMouseCallbacks.end(), callback);
	if (position != this->_processMouseCallbacks.end())
		this->_processMouseCallbacks.erase(position, position + 1);
}

void Window::removeProcessScrollCallbacks(ProcessScrollCallback* callback)
{
	auto position = std::find(this->_processScrollCallbacks.begin(), this->_processScrollCallbacks.end(), callback);
	if (position != this->_processMouseCallbacks.end())
		this->_processScrollCallbacks.erase(position, position + 1);
}