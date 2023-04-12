#include "ReadPLY.h"
#include <fstream>
#include <algorithm>

bool is_number(const string& str) {
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

bool ReadPLY_Header(ifstream &fin, PLY_DATA &plydata)
{
	string str;
	char szData[1024];
	char *context = NULL;

	if (getline(fin, str))
	{
		strcpy(szData, str.c_str());
		str = strtok_s(szData, " \t\r\n", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](char c) -> char { return tolower(c); });
		if (str != "ply")
			return false;
	}
	else
		return false;

	if (getline(fin, str))
	{
		strcpy(szData, str.c_str());
		str = strtok_s(szData, " \t", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](char c) -> char { return tolower(c); });
		if (str != "format")
			return false;

		str = strtok_s(NULL, " \t", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](char c) -> char { return tolower(c); });
		if (str == "ascii")
			plydata.m_iFormat = FORMAT_ASCII;
		else if (str == "binary_little_endian")
			plydata.m_iFormat = FORMAT_BINARY_LITTLE_ENDIAN;
		else if (str == "binary_big_endian")
			plydata.m_iFormat = FORMAT_BINARY_BIG_ENDIAN;
		else
			return false;

		str = strtok_s(NULL, " \t\r\n", &context);
		if (str != "1.0")
			return false;
	}
	else
		return false;

	int iElementCur = ELEMENT_UNKNOWN;
	while (getline(fin, str))
	{
		strcpy(szData, str.c_str());
		str = strtok_s(szData, " \t\r\n", &context);
		transform(str.begin(), str.end(), str.begin(),
			[](char c) -> char { return tolower(c); });
		if (str == "comment")
			;
		else if (str == "element")
		{
			str = strtok_s(NULL, " \t", &context);
			transform(str.begin(), str.end(), str.begin(),
				[](char c) -> char { return tolower(c); });
			if (str == "vertex")
			{
				iElementCur = ELEMENT_VERTEX;

				str = strtok_s(NULL, " \t\r\n", &context);
				if (is_number(str))
					plydata.m_iNbVertex = stoi(str);
			}
			else if (str == "face")
			{
				iElementCur = ELEMENT_FACE;

				str = strtok_s(NULL, " \t\r\n", &context);
				if (is_number(str))
					plydata.m_iNbFace = stoi(str);
			}
		}
		else if (str == "property")
		{
			if (iElementCur == ELEMENT_VERTEX)
			{
				VTX_PROP vtxProp;

				str = strtok_s(NULL, " \t", &context);
				vtxProp.SetDataType(str);

				str = strtok_s(NULL, " \t\r\n", &context);
				transform(str.begin(), str.end(), str.begin(),
					[](char c) -> char { return tolower(c); });
				vtxProp.SetName(str);

				plydata.m_ayVtxProp.push_back(vtxProp);
			}
			else if (iElementCur == ELEMENT_FACE)
			{
				str = strtok_s(NULL, " \t", &context);
				transform(str.begin(), str.end(), str.begin(),
					[](char c) -> char { return tolower(c); });
				if (str != "list")
					return false;

				str = strtok_s(NULL, " \t", &context);
				plydata.m_faceProp.SetListNumDataType(str);

				str = strtok_s(NULL, " \t", &context);
				plydata.m_faceProp.SetIndexDataType(str);
			}
		}
		else if (str == "end_header")
			return plydata.m_iNbVertex > 0;
	}

	return false;
}

bool ReadPLY_ASCII(ifstream &fin, const PLY_DATA &plydata, MESH_DATA &meshData)
{
	bool bRet = false;

	bool bNx = false;
	bool bNy = false;
	bool bNz = false;
	for (int i = 0; i < (int)plydata.m_ayVtxProp.size(); i++)
	{
		VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

		if (vtxProp.GetName() == "nx")
			bNx = true;
		else if (vtxProp.GetName() == "ny")
			bNy = true;
		else if (vtxProp.GetName() == "nz")
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

	string str;
	char szData[1024];
	char *context = NULL;

	int iCounter = 0;
	while (iCounter < plydata.m_iNbVertex)
	{
		if (!getline(fin, str))
			goto READ_END;
		strcpy(szData, str.c_str());

		for (int i = 0; i < (int)plydata.m_ayVtxProp.size(); i++)
		{
			VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

			if (i == 0)
				str = strtok_s(szData, " \t,;", &context);
			else
				str = strtok_s(NULL, " \t,;\r\n", &context);

			if (!is_number(str))
				goto READ_END;

			if (vtxProp.GetName() == "x")
				meshData.m_ayDot[iCounter].m_x = stod(str);
			else if (vtxProp.GetName() == "y")
				meshData.m_ayDot[iCounter].m_y = stod(str);
			else if (vtxProp.GetName() == "z")
				meshData.m_ayDot[iCounter].m_z = stod(str);
			else if (vtxProp.GetName() == "nx")
				meshData.m_ayNormal[iCounter].m_x = stod(str);
			else if (vtxProp.GetName() == "ny")
				meshData.m_ayNormal[iCounter].m_y = stod(str);
			else if (vtxProp.GetName() == "nz")
				meshData.m_ayNormal[iCounter].m_z = stod(str);
		}

		iCounter++;
	}

	iCounter = 0;
	while (iCounter < plydata.m_iNbFace)
	{
		if (!getline(fin, str))
			goto READ_END;
		strcpy(szData, str.c_str());

		str = strtok_s(szData, " \t,;\r\n", &context);
		int iListNum = is_number(str) ? stoi(str) : 0;
		if (iListNum != 3)
			goto READ_END;

		for (int i = 0; i < 3; i++)
		{
			str = strtok_s(NULL, " \t,;\r\n", &context);
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

bool ReadPLY_Binary(ifstream &fin, const PLY_DATA &plydata, MESH_DATA &meshData)
{
	bool bRet = false;

	bool bNx = false;
	bool bNy = false;
	bool bNz = false;
	for (int i = 0; i < (int)plydata.m_ayVtxProp.size(); i++)
	{
		VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

		if (vtxProp.GetName() == "nx")
			bNx = true;
		else if (vtxProp.GetName() == "ny")
			bNy = true;
		else if (vtxProp.GetName() == "nz")
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
		for (int i = 0; i < (int)plydata.m_ayVtxProp.size(); i++)
		{
			VTX_PROP vtxProp = plydata.m_ayVtxProp[i];

			int iDataSize = vtxProp.GetDataSize();
			char *pData = new char[iDataSize];

			fin.read(pData, iDataSize);
			bool bReadChk;
			if (bReadChk = fin.gcount() == iDataSize)
			{
				if (vtxProp.GetName() == "x")
					meshData.m_ayDot[iCounter].m_x = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == "y")
					meshData.m_ayDot[iCounter].m_y = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == "z")
					meshData.m_ayDot[iCounter].m_z = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == "nx")
					meshData.m_ayNormal[iCounter].m_x = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == "ny")
					meshData.m_ayNormal[iCounter].m_y = vtxProp.GetData(pData);
				else if (vtxProp.GetName() == "nz")
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
		char *pData = new char[iDataSize];

		int iListNum = 0;
		fin.read(pData, iDataSize);
		bool bReadChk;
		if (bReadChk = fin.gcount() == iDataSize)
			iListNum = plydata.m_faceProp.GetListNum(pData);

		delete[] pData;

		if (!bReadChk || (iListNum != 3))
			goto READ_END;

		iDataSize = plydata.m_faceProp.GetIndexDataSize();
		pData = new char[iDataSize];

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

		iCounter++;
	}

	bRet = true;

READ_END:
	if (!bRet)
		meshData.~MESH_DATA();

	return bRet;
}

bool ReadPLY(const string &strPath, MESH_DATA &meshData)
{
	bool bRet = false;

	ifstream fin;
	fin.open(strPath, ios::binary);
	if (!fin.is_open())
		return false;

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