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
#include <vector>
#include <boost/shared_ptr.hpp>

struct Vertex
{
	glm::vec3 Position;
	glm::vec3 Normal;
};

struct ColoredVertex
{
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec3 Color;
	float Transparency;
};

class MeshBase
{
public:
	virtual void draw() const = 0;

protected:
	virtual void setupMesh() = 0;
};

class Mesh : public MeshBase
{
public:

	Mesh(std::vector<Vertex>* pVertices, std::vector<unsigned int>* pIndices);
	Mesh(boost::shared_ptr<std::vector<Vertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices);
	~Mesh();

	// Inherited via MeshBase
	virtual void draw() const override;
	virtual void setupMesh() override;

protected:
	unsigned int _vao, _vbo, _ebo;
	boost::shared_ptr<std::vector<Vertex>> _pVertices;
	boost::shared_ptr<std::vector<unsigned int>> _pIndices;
};

class ColoredMesh : public MeshBase
{
public:

	ColoredMesh(std::vector<ColoredVertex>* pVertices, std::vector<unsigned int>* pIndices);
	ColoredMesh(boost::shared_ptr<std::vector<ColoredVertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices);
	~ColoredMesh();

	// Inherited via MeshBase
	virtual void draw() const override;
	virtual void setupMesh() override;

protected:
	unsigned int _vao, _vbo, _ebo;
	boost::shared_ptr<std::vector<ColoredVertex>> _pVertices;
	boost::shared_ptr<std::vector<unsigned int>> _pIndices;
};
