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

	/*
	Handle(TShort_HArray1OfShortReal) pAyNorms = new TShort_HArray1OfShortReal();
	pAyNorms->Resize(1, pNormals->size() * 3, Standard_False);
	pMesh->SetNormals(pAyNorms);

	Normal norm;
	for (Standard_Integer i = 1; i <= (Standard_Integer)(pMesh->Normals().Size() / 3); i++)
	{
		norm = (*pNormals)[i - 1];

		//目前測試檔案求出法向量都正好相反，暫時取反
		pMesh->SetNormal(i, gp_Dir(-norm.normal_x, -norm.normal_y, -norm.normal_z));
	}
	*/

	return false;
}