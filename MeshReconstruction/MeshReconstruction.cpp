// MeshReconstruction.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//

#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <locale>
#include <vector>
using namespace std;

typedef unsigned char BYTE;

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
};

struct CTriangle
{
	CDot m_dot[3];
	CDot m_dotNormal;
};

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
	wstring m_strDataType;

	wstring GetDataType() const
	{
		return m_strDataType;
	}

	void SetDataType(wstring strDataType)
	{
		m_strDataType = strDataType;
	}

	int GetDataSize() const
	{
		if (m_strDataType == L"char")
			return 1;
		else if (m_strDataType == L"uchar")
			return 1;
		else if (m_strDataType == L"short")
			return 2;
		else if (m_strDataType == L"ushort")
			return 2;
		else if (m_strDataType == L"int")
			return 4;
		else if (m_strDataType == L"uint")
			return 4;
		else if (m_strDataType == L"int8")
			return 1;
		else if (m_strDataType == L"uint8")
			return 1;
		else if (m_strDataType == L"int16")
			return 2;
		else if (m_strDataType == L"uint16")
			return 2;
		else if (m_strDataType == L"int32")
			return 4;
		else if (m_strDataType == L"uint32")
			return 4;
		else if (m_strDataType == L"float")
			return 4;
		else if (m_strDataType == L"float32")
			return 4;
		else if (m_strDataType == L"float64")
			return 8;
		else if (m_strDataType == L"double")
			return 8;
		else
			return 0;
	}

	double TransformData(BYTE *pData) const
	{
		if (m_strDataType == L"char")
			return (double)*(char*)pData;
		else if (m_strDataType == L"uchar")
			return (double)*(unsigned char*)pData;
		else if (m_strDataType == L"short")
			return (double)*(short*)pData;
		else if (m_strDataType == L"ushort")
			return (double)*(unsigned short*)pData;
		else if (m_strDataType == L"int")
			return (double)*(int*)pData;
		else if (m_strDataType == L"uint")
			return (double)*(unsigned int*)pData;
		else if (m_strDataType == L"int8")
			return (double)*(int8_t*)pData;
		else if (m_strDataType == L"uint8")
			return (double)*(uint8_t*)pData;
		else if (m_strDataType == L"int16")
			return (double)*(int16_t*)pData;
		else if (m_strDataType == L"uint16")
			return (double)*(uint16_t*)pData;
		else if (m_strDataType == L"int32")
			return (double)*(int32_t*)pData;
		else if (m_strDataType == L"uint32")
			return (double)*(uint32_t*)pData;
		else if (m_strDataType == L"float")
			return *(float*)pData;
		else if (m_strDataType == L"float32")
			return *(float*)pData;
		else if (m_strDataType == L"float64")
			return *(double*)pData;
		else if (m_strDataType == L"double")
			return *(double*)pData;
		else
			return 0.;
	}
} DATA_TYPE;

typedef struct VERTEX_PROPERTY
{
	DATA_TYPE m_dataType;
	wstring m_strName;

	VERTEX_PROPERTY()
	{

	}

	void SetDataType(wstring strDataType)
	{
		m_dataType.SetDataType(strDataType);
	}

	int GetDataSize() const
	{
		return m_dataType.GetDataSize();
	}

	double GetData(BYTE *pData) const
	{
		return m_dataType.TransformData(pData);
	}

	wstring GetName() const
	{
		return m_strName;
	}

	void SetName(wstring strName)
	{
		m_strName = strName;
	}
} VTX_PROP;

typedef struct FACE_PROPERTY
{
	DATA_TYPE m_dataListNum;
	DATA_TYPE m_dataIndex;

	void SetListNumDataType(wstring strDataType)
	{
		m_dataListNum.SetDataType(strDataType);
	}

	void SetIndexDataType(wstring strDataType)
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

	int GetListNum(BYTE *pData) const
	{
		return (int)m_dataListNum.TransformData(pData);
	}

	int GetIndex(BYTE *pData) const
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

bool ReadPLY(const wstring &strPath, vector<CDot> &ayPoint, vector<CDot> &ayNorm);

int main()
{
	wstring str = L"ThIs iS a SaMpLe StRiNg";
	transform(str.begin(), str.end(), str.begin(),
		[](wchar_t c) -> wchar_t { return towlower(c); });
	wcout << str << '\n';

	system("pause");
	return 0;
}

bool ReadPLY_Header(wifstream &fin, PLY_DATA &plydata)
{
	wstring str;
	TCHAR szData[256];
	TCHAR *szTmp = NULL;
	TCHAR *context = NULL;

	if (getline(fin, str))
	{
		szTmp = wcstok_s(szData, L" \t\r\n", &context);
		str = szTmp;
		if (str != L"ply")
			return false;
	}
	else
		return false;

	if (getline(fin, str))
	{
		szTmp = wcstok_s(szData, L" \t", &context);
		str = szTmp;
		if (str != L"format")
			return false;

		szTmp = wcstok_s(szData, L" \t", &context);
		str = szTmp;
		if (str == L"ascii")
			plydata.m_iFormat = FORMAT_ASCII;
		else if (str == L"binary_little_endian")
			plydata.m_iFormat = FORMAT_BINARY_LITTLE_ENDIAN;
		else if (str == L"binary_big_endian")
			plydata.m_iFormat = FORMAT_BINARY_BIG_ENDIAN;
		else
			return false;

		szTmp = wcstok_s(szData, L" \t\r\n", &context);
		str = szTmp;
		if (str != L"1.0")
			return false;
	}
	else
		return false;

	int iElementCur = ELEMENT_UNKNOWN;
	while (getline(fin, str))
	{
		szTmp = wcstok_s(szData, L" \t\r\n", &context);
		str = szTmp;
		if (str == L"comment")
			;
		else if (str == L"element")
		{
			szTmp = wcstok_s(szData, L" \t", &context);
			str = szTmp;
			if (str == L"vertex")
			{
				iElementCur = ELEMENT_VERTEX;

				szTmp = wcstok_s(szData, L" \t\r\n", &context);
				plydata.m_iNbVertex = stoi(szTmp);
			}
			else if (str == L"face")
			{
				iElementCur = ELEMENT_FACE;

				szTmp = wcstok_s(szData, L" \t\r\n", &context);
				plydata.m_iNbFace = stoi(szTmp);
			}
		}
		else if (str == L"property")
		{
			if (iElementCur == ELEMENT_VERTEX)
			{
				VTX_PROP vtxProp;

				szTmp = wcstok_s(szData, L" \t", &context);
				str = szTmp;
				vtxProp.SetDataType(str);

				szTmp = wcstok_s(szData, L" \t\r\n", &context);
				str = szTmp;
				vtxProp.SetName(str);

				plydata.m_ayVtxProp.push_back(vtxProp);
			}
			else if (iElementCur == ELEMENT_FACE)
			{
				szTmp = wcstok_s(szData, L" \t", &context);
				str = szTmp;
				if (str != L"list")
					return false;

				szTmp = wcstok_s(szData, L" \t", &context);
				str = szTmp;
				plydata.m_faceProp.SetListNumDataType(str);

				szTmp = wcstok_s(szData, L" \t", &context);
				str = szTmp;
				plydata.m_faceProp.SetIndexDataType(str);
			}
		}
		else if (str == L"end_header")
		{
			return plydata.m_iNbVertex > 0;
		}
	}

	return false;
}

bool ReadPLY_ASCII(const wstring &strPath, vector<CDot> &ayPoint, vector<CDot> &ayNorm)
{
	return false;
}

bool ReadPLY_Binary(const wstring &strPath, vector<CDot> &ayPoint, vector<CDot> &ayNorm)
{
	return false;
}

bool ReadPLY(const wstring &strPath, vector<CDot> &ayPoint, vector<CDot> &ayNorm)
{
	return false;
}