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

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "oglw.h"
#include <vector>

class Window;

class Window
{
public:
	Window(int width, int height, const char* title, ProcessFramebufferSizeCallback* framebufferSizeCallback = NULL, ProcessKeyInputCallback* processKeyInputCallback = NULL, RenderCallback* renderCallback = NULL,
		ProcessMouseCallback* mouseCallback = NULL, ProcessScrollCallback* scrollCallback = NULL);
	~Window();
	void run();
	GLFWwindow* getInternalWindow();

	void addFramebufferSizeCallback(ProcessFramebufferSizeCallback* callback);
	void addProcessKeyInputCallback(ProcessKeyInputCallback* callback);
	void addUpdateCallback(UpdateCallback* callback);
	void addRenderCallback(RenderCallback* callback);
	void addProcessMouseCallback(ProcessMouseCallback* callback);
	void addProcessScrollCallback(ProcessScrollCallback* callback);

	void removeFramebufferSizeCallback(ProcessFramebufferSizeCallback* callback);
	void removeProcessKeyInputCallbacks(ProcessKeyInputCallback* callback);
	void removeRenderCallbacks(RenderCallback* callback);
	void removeProcessMouseCallbacks(ProcessMouseCallback* callback);
	void removeProcessScrollCallbacks(ProcessScrollCallback* callback);

	int GetWidth() { return _width; }
	int GetHeight() { return _height; }
	float GetDeltaTime() { return _deltaTime; }


protected:
	GLFWwindow * _internalWindow;

	std::vector<ProcessFramebufferSizeCallback*> _framebufferSizeCallbacks;
	std::vector<ProcessKeyInputCallback*> _processKeyInputCallbacks;
	std::vector<RenderCallback*> _renderCallbacks;
	std::vector<UpdateCallback*> _updateCallbacks;
	std::vector<ProcessMouseCallback*> _processMouseCallbacks;
	std::vector<ProcessScrollCallback*> _processScrollCallbacks;

private:
	static void _processFrameBufferSizeChanged(GLFWwindow* window, int width, int height);
	static void _processMouseMoved(GLFWwindow* window, double xpos, double ypos);
	static void _processScrolled(GLFWwindow* window, double xoffset, double yoffset);
	static void _processKeyInput(GLFWwindow* window, int key, int scancode, int action, int mode);

	int _width;
	int _height;
	float _deltaTime;
	float _lastFrameTime;
	ProcessFramebufferSizeCallback _framebufferSizeChangedCallback;
};

