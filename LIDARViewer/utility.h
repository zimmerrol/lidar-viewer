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

#include <vector>
#include <boost/shared_ptr.hpp>
#include <assert.h>
#include "../OGLW/mesh.h"

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif

namespace utility
{
#ifdef USE_PCL
	template <typename PointT>
	boost::shared_ptr<pcl::PointCloud<PointT>> loadPointCloid(const char* fileName)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(fileName, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		return cloud;
	}

	void convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void convertRawDataToPointCloud(std::vector<float> &rawData, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	boost::shared_ptr<Mesh> convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
	boost::shared_ptr<ColoredMesh> convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
	boost::shared_ptr<ColoredMesh> convertPolygonMeshToTriangleMesh(pcl::PolygonMesh polygonMesh, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
#endif
	boost::shared_ptr<std::vector<float>> loadRawData(const char* filename, int nPoints = -1, float maximumIntensity = 2000.0f);
}