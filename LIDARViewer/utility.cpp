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
#include "utility.h"
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <fstream>

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif

#ifdef USE_PCL
void utility::convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	assert(rawData.size() % 3 == 0);

	cloud->clear();
	cloud->height = 1;
	cloud->width = rawData.size() / 6;
	cloud->is_dense = false;

	for (size_t i = 0; i < rawData.size() / 6; i++)
	{
		pcl::PointXYZ point;
		point.x = rawData.at(i + 0);
		point.y = rawData.at(i + 1);
		point.z = rawData.at(i + 2);
		cloud->push_back(point);
	}
}

void utility::convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	assert(rawData.size() % 6 == 0);

	cloud->clear();
	cloud->height = 1;
	cloud->width = rawData.size() / 6;
	cloud->is_dense = false;

	for (size_t i = 0; i < rawData.size() / 6; i++)
	{
		pcl::PointXYZRGB point;
		point.x = rawData.at(i + 0);
		point.y = rawData.at(i + 1);
		point.z = rawData.at(i + 2);
		point.r = rawData.at(i + 3);
		point.g = rawData.at(i + 4);
		point.b = rawData.at(i + 5);
		cloud->push_back(point);
	}
}

void utility::convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	assert(rawData.size() % 7 == 0);

	cloud->clear();
	cloud->height = 1;
	cloud->width = rawData.size() / 7;
	cloud->is_dense = false;

	for (size_t i = 0; i < rawData.size() / 7; i++)
	{
		pcl::PointXYZRGBA point;
		point.x = rawData.at(i + 0);
		point.y = rawData.at(i + 1);
		point.z = rawData.at(i + 2);
		point.r = rawData.at(i + 3);
		point.g = rawData.at(i + 4);
		point.b = rawData.at(i + 5);
		point.a = rawData.at(i + 6);
		cloud->push_back(point);
	}
}


boost::shared_ptr<Mesh> utility::convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	int* vertexUsed = new int[cloud->size()];
	for (size_t i = 0; i < cloud->size(); i++)
	{
		vertexUsed[i] = -1;
	}

	boost::shared_ptr<std::vector<Vertex>> vertices(new std::vector<Vertex>());
	boost::shared_ptr<std::vector<unsigned int>> indices(new std::vector<unsigned int>());

	// create vertices/indices
	unsigned int index;
	for (size_t i = 0; i < polygonMesh.polygons.size(); i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			index = polygonMesh.polygons[i].vertices[j];
			if (vertexUsed[index] == -1)
			{
				auto xyz = cloud->at(index);
				vertices->push_back(Vertex{ glm::vec3(xyz.x, xyz.y, xyz.z), glm::vec3() });

				vertexUsed[index] = vertices->size() - 1;
			}
			indices->push_back(vertexUsed[index]);
		}
	}

	delete vertexUsed;

	return boost::shared_ptr<Mesh>(new Mesh(vertices, indices));
}

boost::shared_ptr<ColoredMesh> utility::convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	int* vertexUsed = new int[cloud->size()];
	for (size_t i = 0; i < cloud->size(); i++)
	{
		vertexUsed[i] = -1;
	}

	boost::shared_ptr<std::vector<ColoredVertex>> vertices(new std::vector<ColoredVertex>());
	boost::shared_ptr<std::vector<unsigned int>> indices(new std::vector<unsigned int>());

	// create vertices/indices
	unsigned int index;
	for (size_t i = 0; i < polygonMesh.polygons.size(); i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			index = polygonMesh.polygons[i].vertices[j];
			if (vertexUsed[index] == -1)
			{
				auto xyzrgb = cloud->at(index);
				vertices->push_back(ColoredVertex{ glm::vec3(xyzrgb.x, xyzrgb.y, xyzrgb.z), glm::vec3(), glm::vec3(xyzrgb.r / 255.0f, xyzrgb.g / 255.0f, xyzrgb.b / 255.0f) });

				vertexUsed[index] = vertices->size() - 1;
			}
			indices->push_back(vertexUsed[index]);
		}
	}

	delete vertexUsed;

	return boost::shared_ptr<ColoredMesh>(new ColoredMesh(vertices, indices));
}

boost::shared_ptr<ColoredMesh> utility::convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	int* vertexUsed = new int[cloud->size()];
	for (size_t i = 0; i < cloud->size(); i++)
	{
		vertexUsed[i] = -1;
	}

	boost::shared_ptr<std::vector<ColoredVertex>> vertices(new std::vector<ColoredVertex>());
	boost::shared_ptr<std::vector<unsigned int>> indices(new std::vector<unsigned int>());

	// create vertices/indices
	unsigned int index;
	for (size_t i = 0; i < polygonMesh.polygons.size(); i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			index = polygonMesh.polygons[i].vertices[j];
			if (vertexUsed[index] == -1)
			{
				auto xyzrgba = cloud->at(index);
				vertices->push_back(ColoredVertex{ glm::vec3(xyzrgba.x, xyzrgba.y, xyzrgba.z), glm::vec3(), glm::vec3(xyzrgba.r / 255.0f, xyzrgba.g / 255.0f, xyzrgba.b / 255.0f), xyzrgba.a / 255.0f });

				vertexUsed[index] = vertices->size() - 1;
			}
			indices->push_back(vertexUsed[index]);
		}
	}

	delete vertexUsed;

	return boost::shared_ptr<ColoredMesh>(new ColoredMesh(vertices, indices));
}
#endif

boost::shared_ptr<std::vector<float>> utility::loadRawData(const char* filename, int nPoints, float maximumIntensity)
{
	boost::shared_ptr<std::vector<float>> outputData(new std::vector<float>());
	outputData->reserve(nPoints * 7);

	std::ifstream fileStream(filename, std::ios::in);

	int currentLine = 0;
	float x, y, z;
	int intensity, r, g, b;
	if (fileStream.is_open())
	{
		while (!fileStream.eof() && currentLine < nPoints)
		{
			fileStream >> x >> y >> z >> intensity >> r >> g >> b;
			outputData->push_back(x);
			outputData->push_back(z);
			outputData->push_back(y);
			outputData->push_back(r / 255.0f);
			outputData->push_back(g / 255.0f);
			outputData->push_back(b / 255.0f);
			outputData->push_back(abs(intensity) / maximumIntensity);
			currentLine++;
		}
	}

	fileStream.close();

	return outputData;
}