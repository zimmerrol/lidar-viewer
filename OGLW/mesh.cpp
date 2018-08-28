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
#include "mesh.h"

#include "glad/glad.h"

Mesh::Mesh(std::vector<Vertex>* pVertices, std::vector<unsigned int>* pIndices) : _pVertices(pVertices), _pIndices(pIndices)
{
	setupMesh();
}

Mesh::Mesh(boost::shared_ptr<std::vector<Vertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices) : _pVertices(pVertices), _pIndices(pIndices)
{
	setupMesh();
}

Mesh::~Mesh()
{
}

void Mesh::draw() const
{
	glBindVertexArray(_vao);
	glDrawElements(GL_TRIANGLES, _pIndices->size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void Mesh::setupMesh()
{
	glGenVertexArrays(1, &_vao);
	glGenBuffers(1, &_vbo);
	glGenBuffers(1, &_ebo);

	glBindVertexArray(_vao);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);

	glBufferData(GL_ARRAY_BUFFER, _pVertices->size() * sizeof(Vertex), &_pVertices->front(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, _pIndices->size() * sizeof(unsigned int), &_pIndices->front(), GL_STATIC_DRAW);

	// vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

	glBindVertexArray(0);
}

ColoredMesh::ColoredMesh(std::vector<ColoredVertex>* pVertices, std::vector<unsigned int>* pIndices) : _pVertices(pVertices), _pIndices(pIndices)
{
	setupMesh();
}

ColoredMesh::ColoredMesh(boost::shared_ptr<std::vector<ColoredVertex>> pVertices, boost::shared_ptr<std::vector<unsigned int>> pIndices) : _pVertices(pVertices), _pIndices(pIndices)
{
	setupMesh();
}

ColoredMesh::~ColoredMesh()
{
}

void ColoredMesh::draw() const
{
	glBindVertexArray(_vao);
	glDrawElements(GL_TRIANGLES, _pIndices->size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void ColoredMesh::setupMesh()
{
	glGenVertexArrays(1, &_vao);
	glGenBuffers(1, &_vbo);
	glGenBuffers(1, &_ebo);

	glBindVertexArray(_vao);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);

	glBufferData(GL_ARRAY_BUFFER, _pVertices->size() * sizeof(ColoredVertex), &_pVertices->front(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, _pIndices->size() * sizeof(unsigned int), &_pIndices->front(), GL_STATIC_DRAW);

	// vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ColoredVertex), (void*)0);

	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ColoredVertex), (void*)offsetof(ColoredVertex, Normal));

	// color
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(ColoredVertex), (void*)offsetof(ColoredVertex, Color));

	// transparency
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(ColoredVertex), (void*)offsetof(ColoredVertex, Transparency));

	glBindVertexArray(0);
}
