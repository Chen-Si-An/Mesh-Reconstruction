#pragma once

#include <string>
#include <vector>

using namespace std;

enum
{
	FORMAT_UNKNOWN = 0,
	FORMAT_ASCII,
	FORMAT_BINARY_LITTLE_ENDIAN,
	FORMAT_BINARY_BIG_ENDIAN,
};

enum
{
	ELEMENT_UNKNOWN = 0,
	ELEMENT_VERTEX,
	ELEMENT_FACE,
};

typedef struct DATA_TYPE
{
	string m_strDataType;

	string GetDataType() const
	{
		return m_strDataType;
	}

	void SetDataType(string strDataType)
	{
		m_strDataType = strDataType;
	}

	int GetDataSize() const
	{
		if (m_strDataType == "char")
			return 1;
		else if (m_strDataType == "uchar")
			return 1;
		else if (m_strDataType == "short")
			return 2;
		else if (m_strDataType == "ushort")
			return 2;
		else if (m_strDataType == "int")
			return 4;
		else if (m_strDataType == "uint")
			return 4;
		else if (m_strDataType == "int8")
			return 1;
		else if (m_strDataType == "uint8")
			return 1;
		else if (m_strDataType == "int16")
			return 2;
		else if (m_strDataType == "uint16")
			return 2;
		else if (m_strDataType == "int32")
			return 4;
		else if (m_strDataType == "uint32")
			return 4;
		else if (m_strDataType == "float")
			return 4;
		else if (m_strDataType == "float32")
			return 4;
		else if (m_strDataType == "float64")
			return 8;
		else if (m_strDataType == "double")
			return 8;
		else
			return 0;
	}

	double TransformData(char *pData) const
	{
		if (m_strDataType == "char")
			return (double)*(char*)pData;
		else if (m_strDataType == "uchar")
			return (double)*(unsigned char*)pData;
		else if (m_strDataType == "short")
			return (double)*(short*)pData;
		else if (m_strDataType == "ushort")
			return (double)*(unsigned short*)pData;
		else if (m_strDataType == "int")
			return (double)*(int*)pData;
		else if (m_strDataType == "uint")
			return (double)*(unsigned int*)pData;
		else if (m_strDataType == "int8")
			return (double)*(int8_t*)pData;
		else if (m_strDataType == "uint8")
			return (double)*(uint8_t*)pData;
		else if (m_strDataType == "int16")
			return (double)*(int16_t*)pData;
		else if (m_strDataType == "uint16")
			return (double)*(uint16_t*)pData;
		else if (m_strDataType == "int32")
			return (double)*(int32_t*)pData;
		else if (m_strDataType == "uint32")
			return (double)*(uint32_t*)pData;
		else if (m_strDataType == "float")
			return *(float*)pData;
		else if (m_strDataType == "float32")
			return *(float*)pData;
		else if (m_strDataType == "float64")
			return *(double*)pData;
		else if (m_strDataType == "double")
			return *(double*)pData;
		else
			return 0.;
	}
} DATA_TYPE;

typedef struct VERTEX_PROPERTY
{
	DATA_TYPE m_dataType;
	string m_strName;

	VERTEX_PROPERTY()
	{

	}

	void SetDataType(string strDataType)
	{
		m_dataType.SetDataType(strDataType);
	}

	int GetDataSize() const
	{
		return m_dataType.GetDataSize();
	}

	double GetData(char *pData) const
	{
		return m_dataType.TransformData(pData);
	}

	string GetName() const
	{
		return m_strName;
	}

	void SetName(string strName)
	{
		m_strName = strName;
	}
} VTX_PROP;

typedef struct FACE_PROPERTY
{
	DATA_TYPE m_dataListNum;
	DATA_TYPE m_dataIndex;

	void SetListNumDataType(string strDataType)
	{
		m_dataListNum.SetDataType(strDataType);
	}

	void SetIndexDataType(string strDataType)
	{
		m_dataIndex.SetDataType(strDataType);
	}

	int GetListNumDataSize() const
	{
		return m_dataListNum.GetDataSize();
	}

	int GetIndexDataSize() const
	{
		return m_dataIndex.GetDataSize();
	}

	int GetListNum(char *pData) const
	{
		return (int)m_dataListNum.TransformData(pData);
	}

	int GetIndex(char *pData) const
	{
		return (int)m_dataIndex.TransformData(pData);
	}
} FACE_PROP;

typedef struct PLY_DATA
{
	int	m_iFormat;
	int	m_iNbVertex;
	vector<VTX_PROP> m_ayVtxProp;
	int	m_iNbFace;
	FACE_PROP m_faceProp;

	PLY_DATA()
	{
		m_iFormat = FORMAT_UNKNOWN;
		m_iNbVertex = 0;
		m_iNbFace = 0;
	}

	~PLY_DATA()
	{
		vector<VTX_PROP>().swap(m_ayVtxProp);
	}
} PLY_DATA;

struct CDot
{
	double	m_x;
	double	m_y;
	double	m_z;

	CDot(double dX = 0., double dY = 0., double dZ = 0.)
	{
		m_x = dX;
		m_y = dY;
		m_z = dZ;
	}

	CDot operator-(const CDot& dot)
	{
		return CDot(m_x - dot.m_x, m_y - dot.m_y, m_z - dot.m_z);
	}
};

typedef struct MESH_DATA
{
	vector<CDot> m_ayDot;
	vector<CDot> m_ayNormal;
	vector<int> m_ayFace;

	~MESH_DATA()
	{
		vector<CDot>().swap(m_ayDot);
		vector<CDot>().swap(m_ayNormal);
		vector<int>().swap(m_ayFace);
	}

	bool HasNormals() const
	{
		return m_ayNormal.size() > 0;
	}

	int NbNodes() const
	{
		return (int)m_ayDot.size();
	}

	CDot Node(int iIndex) const
	{
		return m_ayDot[iIndex];
	}

	int NbTriangles() const
	{
		return (int)m_ayFace.size() / 3;
	}
} MESH_DATA;
