#include "BallPivoting.h"
#include "PCLLib.h"
#include "Pivoter.h"
#include "Front.h"

bool BPASurfaceReconstruction(const MESH_DATA &meshData, MESH_DATA &meshResult, double dBallRadius)
{
	PointCloud<PointXYZ>::Ptr pCloud = Mesh2PointCloud(meshData);
	PointCloud<Normal>::Ptr pNormals = Mesh2Normals(meshData);
	PointCloud<PointNormal>::Ptr pCloudwithNormals(new PointCloud<PointNormal>);
	concatenateFields(*pCloud, *pNormals, *pCloudwithNormals);

	Pivoter pivoter(pCloudwithNormals, dBallRadius);
	Front front;
	vector<TrianglePtr> mesh;

	while (true)
	{
		// Pivot from the current front
		EdgePtr edge;
		while ((edge = front.getActiveEdge()) != NULL)
		{
			// Testing edge
			pair<int, TrianglePtr> data = pivoter.pivot(edge);
			if (data.first != -1 && (!pivoter.isUsed(data.first) || front.inFront(data.first)))
			{
				mesh.push_back(data.second);
				front.joinAndFix(data, pivoter);
			}
			else
				front.setInactive(edge);
		}

		// Find a new seed
		TrianglePtr seed;
		if ((seed = pivoter.findSeed()) != NULL)
		{
			mesh.push_back(seed);
			front.addEdges(seed);
		}
		else
			break;
	}

	if (mesh.size() == 0)
		return false;

	meshResult.m_ayDot.resize(mesh.size() * 3);
	meshResult.m_ayFace.resize(mesh.size() * 3);
	for (int i = 0; i < mesh.size(); i++)
	{
		TrianglePtr pTri = mesh[i];
		for (int j = 0; j < 3; j++)
			meshResult.m_ayDot[3 * i + j] = CDot(pTri->getVertex(j).first->x, pTri->getVertex(j).first->y, pTri->getVertex(j).first->z);
	}
	for (int i = 0; i < mesh.size(); i++)
		for (int j = 0; j < 3; j++)
			meshResult.m_ayFace[3 * i + j] = 3 * i + j;

	return true;
}