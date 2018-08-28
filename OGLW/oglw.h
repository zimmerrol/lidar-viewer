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
#include <functional>
#include <GLFW/glfw3.h>

class Window;

typedef std::function<void(Window*, int, int)> ProcessFramebufferSizeCallback;
typedef std::function<void(Window*, int key, int scancode, int action, int mode)> ProcessKeyInputCallback;
typedef std::function<void(Window*)> RenderCallback;
typedef std::function<void(Window*, float)> UpdateCallback;
typedef std::function<void(Window*, double, double)> ProcessMouseCallback;
typedef std::function<void(Window*, double, double)> ProcessScrollCallback;

namespace OGLW
{
	int init(int majorVersion = 4, int minorVersion = 6);
}