#include "PCLLib.h"

PointCloud<PointXYZ>::Ptr Mesh2PointCloud(const MESH_DATA &meshData)
{
	PointCloud<PointXYZ>::Ptr pCloud(new PointCloud<PointXYZ>);
	pCloud->width = meshData.NbNodes();
	pCloud->height = 1;
	pCloud->points.resize(pCloud->width * pCloud->height);

	CDot dotNode;
	for (size_t i = 0; i < pCloud->points.size(); i++)
	{
		dotNode = meshData.Node(i);

		pCloud->points[i].x = dotNode.m_x;
		pCloud->points[i].y = dotNode.m_y;
		pCloud->points[i].z = dotNode.m_z;
	}

	return pCloud;
}

PointCloud<Normal>::Ptr Mesh2Normals(const MESH_DATA &meshData)
{
	if (!meshData.HasNormals())
		return NULL;

	PointCloud<Normal>::Ptr pNormals(new PointCloud<Normal>);
	pNormals->resize(meshData.m_ayNormal.size());

	CDot dotNorm;
	for (size_t i = 0; i < pNormals->size(); i++)
	{
		dotNorm = meshData.m_ayNormal[i];

		(*pNormals)[i].normal_x = dotNorm.m_x;
		(*pNormals)[i].normal_y = dotNorm.m_y;
		(*pNormals)[i].normal_z = dotNorm.m_z;
	}

	return pNormals;
}

bool NormalEstimation(MESH_DATA &meshData)
{
	PointCloud<PointXYZ>::Ptr pCloud = Mesh2PointCloud(meshData);

	pcl::NormalEstimation<PointXYZ, Normal> normEst;
	PointCloud<Normal>::Ptr pNormals(new PointCloud<Normal>);
	normEst.setInputCloud(pCloud);
	normEst.setSearchMethod(search::KdTree<PointXYZ>::Ptr(new search::KdTree<PointXYZ>));
	normEst.setKSearch(20);
	normEst.compute(*pNormals);

	if (pNormals->size() == 0)
		return false;

	meshData.m_ayNormal.resize(pNormals->size());

	Normal norm;
	for (int i = 0; i < (int)pNormals->size(); i++)
	{
		norm = (*pNormals)[i];

		//目前測試檔案求出法向量都正好相反，暫時取反
		meshData.m_ayNormal[i] = CDot(-norm.normal_x, -norm.normal_y, -norm.normal_z);
	}

	return true;
}

bool PoissonSurfaceReconstruction(const MESH_DATA &meshData, MESH_DATA &meshResult, double dReconstructDepth, double dOctreeDepth, double dScale,
	double dMinSamples, double dInterpolateWeight, int iGaussSeidelRelax, bool bConfidence)
{
	PointCloud<PointXYZ>::Ptr pCloud = Mesh2PointCloud(meshData);
	PointCloud<Normal>::Ptr pNormals = Mesh2Normals(meshData);
	PointCloud<PointNormal>::Ptr pCloudwithNormals(new PointCloud<PointNormal>);
	concatenateFields(*pCloud, *pNormals, *pCloudwithNormals);

	Poisson<PointNormal> poisson;
	poisson.setInputCloud(pCloudwithNormals);
	poisson.setDepth(dReconstructDepth);
	poisson.setMinDepth(dOctreeDepth);
	poisson.setScale(dScale);
	poisson.setSamplesPerNode(dMinSamples);
	poisson.setPointWeight(dInterpolateWeight);
	poisson.setSolverDivide(iGaussSeidelRelax);
	poisson.setConfidence(bConfidence);

	PolygonMesh polyMesh;
	poisson.reconstruct(polyMesh);
	PointCloud<PointXYZ> cloudResult;
	fromPCLPointCloud2(polyMesh.cloud, cloudResult);

	if (cloudResult.size() == 0 || polyMesh.polygons.size() == 0)
		return false;

	meshResult.m_ayDot.resize(cloudResult.size());
	meshResult.m_ayFace.resize(polyMesh.polygons.size() * 3);
	PointXYZ point;
	for (int i = 0; i < (int)cloudResult.size(); i++)
	{
		point = cloudResult.points[i];

		meshResult.m_ayDot[i] = CDot(point.x, point.y, point.z);
	}
	for (int i = 0; i < (int)polyMesh.polygons.size(); i++)
		for (int j = 0; j < 3; j++)
			meshResult.m_ayFace[3 * i + j] = polyMesh.polygons[i].vertices[j];

	return true;
}