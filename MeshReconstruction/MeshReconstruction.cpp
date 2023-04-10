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

	double TransformData(wchar_t *pData) const
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

	double GetData(wchar_t *pData) const
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

	int GetListNum(wchar_t *pData) const
	{
		return (int)m_dataListNum.TransformData(pData);
	}

	int GetIndex(wchar_t *pData) const
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
} MESH_DATA;

struct CTriangle
{
	CDot m_dot[3];
	CDot m_dotNormal;
};

bool ReadPLY(const wstring &strPath, MESH_DATA &meshData);

int main()
{
	MESH_DATA meshData;
	ReadPLY(L"D:\\Data\\ply\\bottle_binary.ply", meshData);

	system("pause");
	return 0;
}

bool is_number(const wstring& str) {
	try {
		// Try to convert the string to an integer or floating-point number
		size_t pos;
		int i = std::stoi(str, &pos);
		double d = std::stod(str, &pos);

		// If the conversion succeeded and the entire string was consumed, it's a valid number
		return pos == str.length();
	}
	catch (std::invalid_argument&) {
		// If the string could not be converted to a number, return false
		return false;
	}
	catch (std::out_of_range&) {
		// If the number is too large or small to fit in the output type, return false
		return false;
	}
}

bool ReadPLY_Header(wifstream &fin, PLY_DATA &plydata)
{
	wstring str;
	wchar_t szData[1024];
	wchar_t *context = NULL;

	if (getline(fin, str))
	{
		wcscpy_s(szData, str.c_str());
		str = wcstok_s(szData, L" \t\r\n", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](wchar_t c) -> wchar_t { return towlower(c); });
		if (str != L"ply")
			return false;
	}
	else
		return false;

	if (getline(fin, str))
	{
		wcscpy_s(szData, str.c_str());
		str = wcstok_s(szData, L" \t", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](wchar_t c) -> wchar_t { return towlower(c); });
		if (str != L"format")
			return false;

		str = wcstok_s(NULL, L" \t", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](wchar_t c) -> wchar_t { return towlower(c); });
		if (str == L"ascii")
			plydata.m_iFormat = FORMAT_ASCII;
		else if (str == L"binary_little_endian")
			plydata.m_iFormat = FORMAT_BINARY_LITTLE_ENDIAN;
		else if (str == L"binary_big_endian")
			plydata.m_iFormat = FORMAT_BINARY_BIG_ENDIAN;
		else
			return false;

		str = wcstok_s(NULL, L" \t\r\n", &context);
		if (str != L"1.0")
			return false;
	}
	else
		return false;

	int iElementCur = ELEMENT_UNKNOWN;
	while (getline(fin, str))
	{
		wcscpy_s(szData, str.c_str());
		str = wcstok_s(szData, L" \t\r\n", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](wchar_t c) -> wchar_t { return towlower(c); });
		if (str == L"comment")
			;
		else if (str == L"element")
		{
			str = wcstok_s(NULL, L" \t", &context);
			transform(str.begin(), str.end(), str.begin(),
				[](wchar_t c) -> wchar_t { return towlower(c); });
			if (str == L"vertex")
			{
				iElementCur = ELEMENT_VERTEX;

				str = wcstok_s(NULL, L" \t\r\n", &context);
				if (is_number(str))
					plydata.m_iNbVertex = stoi(str);
			}
			else if (str == L"face")
			{
				iElementCur = ELEMENT_FACE;

				str = wcstok_s(NULL, L" \t\r\n", &context);
				if (is_number(str))
					plydata.m_iNbFace = stoi(str);
			}
		}
		else if (str == L"property")
		{
			if (iElementCur == ELEMENT_VERTEX)
			{
				VTX_PROP vtxProp;

				str = wcstok_s(NULL, L" \t", &context);
				vtxProp.SetDataType(str);

				str = wcstok_s(NULL, L" \t\r\n", &context);
				transform(str.begin(), str.end(), str.begin(),
					[](wchar_t c) -> wchar_t { return towlower(c); });
				vtxProp.SetName(str);

				plydata.m_ayVtxProp.push_back(vtxProp);
			}
			else if (iElementCur == ELEMENT_FACE)
			{
				str = wcstok_s(NULL, L" \t", &context);
				transform(str.begin(), str.end(), str.begin(),
					[](wchar_t c) -> wchar_t { return towlower(c); });
				if (str != L"list")
					return false;

				str = wcstok_s(NULL, L" \t", &context);
				plydata.m_faceProp.SetListNumDataType(str);

				str = wcstok_s(NULL, L" \t", &context);
				plydata.m_faceProp.SetIndexDataType(str);
			}
		}
		else if (str == L"end_header")
			return plydata.m_iNbVertex > 0;
	}

	return false;
}

bool ReadPLY_ASCII(wifstream &fin, const PLY_DATA &plydata, MESH_DATA &meshData)
{
	bool bRet = false;

	bool bNx = false;
	bool bNy = false;
	bool bNz = false;
	for (UINT_PTR i = 0; i < plydata.m_ayVtxProp.size(); i++)
	{
		VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

		if (vtxProp.GetName() == L"nx")
			bNx = true;
		else if (vtxProp.GetName() == L"ny")
			bNy = true;
		else if (vtxProp.GetName() == L"nz")
			bNz = true;
	}
	bool bHasNorm = bNx && bNy && bNz;

	if (plydata.m_iNbVertex > 0)
	{
		meshData.m_ayDot.resize(plydata.m_iNbVertex);
		if (bHasNorm)
			meshData.m_ayNormal.resize(plydata.m_iNbVertex);
		if (plydata.m_iNbFace > 0)
			meshData.m_ayFace.resize(plydata.m_iNbFace * 3);
	}

	wstring str;
	wchar_t szData[1024];
	wchar_t *context = NULL;

	int iCounter = 0;
	while (iCounter < plydata.m_iNbVertex)
	{
		if (!getline(fin, str))
			goto READ_END;
		wcscpy_s(szData, str.c_str());

		for (UINT_PTR i = 0; i < plydata.m_ayVtxProp.size(); i++)
		{
			VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

			if (i == 0)
				str = wcstok_s(szData, L" \t,;", &context);
			else
				str = wcstok_s(NULL, L" \t,;\r\n", &context);

			if (!is_number(str))
				goto READ_END;

			if (vtxProp.GetName() == L"x")
				meshData.m_ayDot[iCounter].m_x = stod(str);
			else if (vtxProp.GetName() == L"y")
				meshData.m_ayDot[iCounter].m_y = stod(str);
			else if (vtxProp.GetName() == L"z")
				meshData.m_ayDot[iCounter].m_z = stod(str);
			else if (vtxProp.GetName() == L"nx")
				meshData.m_ayNormal[iCounter].m_x = stod(str);
			else if (vtxProp.GetName() == L"ny")
				meshData.m_ayNormal[iCounter].m_y = stod(str);
			else if (vtxProp.GetName() == L"nz")
				meshData.m_ayNormal[iCounter].m_z = stod(str);
		}

		iCounter++;
	}

	iCounter = 0;
	while (iCounter < plydata.m_iNbFace)
	{
		if (!getline(fin, str))
			goto READ_END;
		wcscpy_s(szData, str.c_str());

		str = wcstok_s(szData, L" \t,;\r\n", &context);
		int iListNum = is_number(str) ? stoi(str) : 0;
		if (iListNum != 3)
			goto READ_END;

		for (int i = 0; i < 3; i++)
		{
			str = wcstok_s(NULL, L" \t,;\r\n", &context);
			if (!is_number(str))
				goto READ_END;

			meshData.m_ayFace[3 * iCounter + i] = stoi(str);
		}

		iCounter++;
	}

	bRet = true;

READ_END:
	if (!bRet)
		meshData.~MESH_DATA();

	return bRet;
}

bool ReadPLY_Binary(wifstream &fin, const PLY_DATA &plydata, MESH_DATA meshData)
{
	bool bRet = false;

	bool bNx = false;
	bool bNy = false;
	bool bNz = false;
	for (UINT_PTR i = 0; i < plydata.m_ayVtxProp.size(); i++)
	{
		VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

		if (vtxProp.GetName() == L"nx")
			bNx = true;
		else if (vtxProp.GetName() == L"ny")
			bNy = true;
		else if (vtxProp.GetName() == L"nz")
			bNz = true;
	}
	bool bHasNorm = bNx && bNy && bNz;

	if (plydata.m_iNbVertex > 0)
	{
		meshData.m_ayDot.resize(plydata.m_iNbVertex);
		if (bHasNorm)
			meshData.m_ayNormal.resize(plydata.m_iNbVertex);
		if (plydata.m_iNbFace > 0)
			meshData.m_ayFace.resize(plydata.m_iNbFace * 3);
	}

	int iCounter = 0;
	while (iCounter < plydata.m_iNbVertex)
	{
		for (UINT_PTR i = 0; i < plydata.m_ayVtxProp.size(); i++)
		{
			VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

			int iDataSize = vtxProp.GetDataSize();
			wchar_t *pData = new wchar_t[iDataSize];

			fin.read(pData, iDataSize);
			BOOL bReadChk;
			if (bReadChk = fin.gcount() == iDataSize)
			{
				if (vtxProp.GetName() == L"x")
					meshData.m_ayDot[iCounter].m_x = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == L"y")
					meshData.m_ayDot[iCounter].m_y = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == L"z")
					meshData.m_ayDot[iCounter].m_z = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == L"nx")
					meshData.m_ayNormal[iCounter].m_x = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == L"ny")
					meshData.m_ayNormal[iCounter].m_y = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == L"nz")
					meshData.m_ayNormal[iCounter].m_z = vtxProp.GetData(pData);
			}

			delete[] pData;

			if (!bReadChk)
				goto READ_END;
		}

		iCounter++;
	}

	iCounter = 0;
	while (iCounter < plydata.m_iNbFace)
	{
		int iDataSize = plydata.m_faceProp.GetListNumDataSize();
		wchar_t *pData = new wchar_t[iDataSize];

		int iListNum = 0;
		fin.read(pData, iDataSize);
		BOOL bReadChk;
		if (bReadChk = fin.gcount() == iDataSize)
			iListNum = plydata.m_faceProp.GetListNum(pData);

		delete[] pData;

		if (!bReadChk || (iListNum != 3))
			goto READ_END;

		iDataSize = plydata.m_faceProp.GetIndexDataSize();
		pData = new wchar_t[iDataSize];

		for (int i = 0; i < 3; i++)
		{
			fin.read(pData, iDataSize);
			if (bReadChk = fin.gcount() == iDataSize)
				meshData.m_ayFace[3 * iCounter + i] = plydata.m_faceProp.GetIndex(pData) + 1;
			else
				break;
		}

		delete[] pData;

		if (!bReadChk)
			goto READ_END;
	}

	bRet = true;

READ_END:
	if (!bRet)
		meshData.~MESH_DATA();

	return bRet;
}

bool ReadPLY(const wstring &strPath, MESH_DATA &meshData)
{
	bool bRet = false;

	wifstream fin;
	fin.open(strPath, ios::binary);

	PLY_DATA plyData;
	if (ReadPLY_Header(fin, plyData))
	{
		if (plyData.m_iFormat == FORMAT_ASCII)
			bRet = ReadPLY_ASCII(fin, plyData, meshData);
		else if (plyData.m_iFormat == FORMAT_BINARY_LITTLE_ENDIAN || plyData.m_iFormat == FORMAT_BINARY_BIG_ENDIAN)
			bRet = ReadPLY_Binary(fin, plyData, meshData);
	}

	fin.close();

	return bRet;
}