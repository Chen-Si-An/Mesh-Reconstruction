/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <cuda_runtime.h>

#define checkErrors(msg) \
	do { \
		cudaError_t __err = cudaGetLastError(); \
		if (__err != cudaSuccess) \
		{ \
			fprintf(stderr, "ERROR: %s (%s at %s:%d)\n", msg, cudaGetErrorString(__err), __FILE__, __LINE__); \
			fprintf(stderr, "*** FAILED - ABORTING\n"); \
			exit(1); \
		} \
	} while (0)

class GpuUtils
{
public:
	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	static bool setDevice(const int _device)
	{
		cudaError_t __err = cudaSetDevice(_device);
		return __err == cudaSuccess;
	}
	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t

	static size_t getAvailableMemory()
	{
		size_t freeMem, totalMem;
		cudaMemGetInfo(&freeMem, &totalMem);
		//checkErrors("memInfo failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
		return freeMem;
	}

	template<class T>
	static inline void allocMemory(T **_devDst, const int _size)
	{
		size_t bytes = sizeof(T) * _size;

		cudaMalloc((void **) _devDst, bytes);
		//checkErrors("cudaMalloc failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	}

	template<class T>
	static inline void createInDev(T **_devDst, const T *_hostSrc, const int _size)
	{
		size_t bytes = sizeof(T) * _size;

		cudaMalloc((void **) _devDst, bytes);
		//checkErrors("cudaMalloc failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t

		cudaMemcpy(*_devDst, _hostSrc, bytes, cudaMemcpyHostToDevice);
		//checkErrors("cudaMemcpy failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	}

	template<class T>
	static inline void setData(T **_devDst, const T *_hostSrc, const int _size)
	{
		size_t bytes = sizeof(T) * _size;

		cudaMemcpy(*_devDst, _hostSrc, bytes, cudaMemcpyHostToDevice);
		//checkErrors("cudaMemcpy failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	}

	template<class T>
	static inline void getData(T *_hostDst, const T *_devSrc, const int _size)
	{
		cudaMemcpy(_hostDst, _devSrc, sizeof(T) * _size, cudaMemcpyDeviceToHost);
		//checkErrors("cudaMemcpy failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	}

	template<class T>
	static inline void setSymbol(T &_devSymbol, const T *_hostSrc)
	{
		cudaMemcpyToSymbol(_devSymbol, _hostSrc, sizeof(T));
		//checkErrors("cudaMemcpyToSymbol failed");	//Allen_20221006A_Ball Pivoting�ϥ�GPU�[�t
	}

private:
	GpuUtils()
	{
	}

	~GpuUtils()
	{
	}
};
