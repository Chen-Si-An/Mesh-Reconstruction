#pragma once

#include "Struct.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr Mesh2PointCloud(const MESH_DATA &meshData);
PointCloud<Normal>::Ptr Mesh2Normals(const MESH_DATA &meshData);
bool NormalEstimation(MESH_DATA &meshData);
bool PoissonSurfaceReconstruction(const MESH_DATA &meshData, MESH_DATA &meshResult, double dReconstructDepth = 8., double dOctreeDepth = 5., 
	double dScale = 1.1, double dMinSamples = 1.5, double dInterpolateWeight = 4., int iGaussSeidelRelax = 8, bool bConfidence = false);
