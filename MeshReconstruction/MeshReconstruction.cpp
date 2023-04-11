// MeshReconstruction.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//

#include "ReadPLY.h"

struct CTriangle
{
	CDot m_dot[3];
	CDot m_dotNormal;
};

int main()
{
	MESH_DATA meshData;
	ReadPLY("D:\\Data\\ply\\bottle_ascii.ply", meshData);

	system("pause");
	return 0;
}
