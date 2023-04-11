#pragma once

#include "Struct.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>

using namespace pcl;

bool NormalEstimation(MESH_DATA &meshData);
