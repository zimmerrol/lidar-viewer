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

#include "oglw.h"
#include "window.h"

class Scene
{
public:
	Scene(Window* parentWindow);
	~Scene();

	virtual void update(float deltatime) = 0;
	virtual void render() const = 0;
	virtual void processKeyboardInput(int key, int scancode, int action, int mode) = 0;
	virtual void processMouseInput(double mousex, double mousey) = 0;
	virtual void processScroll(double scrollx, double scrolly) = 0;

	Window* getParentWindow();

protected:
	Window * _pParentWindow;

private:
	RenderCallback renderCallback;
	UpdateCallback updateCallback;
	ProcessKeyInputCallback processKeyInputCallback;
	ProcessMouseCallback processMouseCallback;
	ProcessScrollCallback processScrollCallback;
};
