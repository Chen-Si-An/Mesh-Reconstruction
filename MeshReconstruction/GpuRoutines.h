/**
 * Author: rodrigo
 * 2015
 */
#pragma once

//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
/*
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
*/
#include <vector>
//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
#include "GpuStructs.h"
#include <cuda_runtime.h>

class GpuRoutines
{
public:
	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	/*
	static gpu::BallCenter findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius);
	static void buildInDeviceKDTree(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	*/
	static gpu::BallCenter findSeed(const std::vector<gpu::Point> *_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius);
	static void buildInDeviceKDTree(const std::vector<gpu::Point> *_cloud);
	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	static void releaseMemory();
	static void prepareStackSize();
	static bool radiusSearch(const gpu::Point &_target, const std::vector<gpu::Point> *_cloud, double radius, std::vector<int> &_idxs);	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
private:
	GpuRoutines()
	{
	}
	~GpuRoutines()
	{
	}

	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	/*
	static void allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static void allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed);
	*/
	static void allocPoints(const std::vector<gpu::Point> *_cloud);
	static void allocUsed(const std::vector<gpu::Point> *_cloud, const bool* _notUsed);
	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
};
