/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <iostream>
#include <chrono>
#include <omp.h>

#include <immintrin.h>

void saxpy(size_t n, float a, float* x, float* y, float* z);
void verifySaxpy(size_t n, float* z);
void saxpy_openMP(size_t n, float a, float* x, float* y, float* z);
void saxpy_SSE(size_t n, float a, float* x, float* y, float* z);
void saxpy_AVX(size_t n, float a, float* x, float* y, float* z);

const float XVAL = rand() % 4096;
const float YVAL = rand() % 4096;
const float AVAL = rand() % 4096;
const size_t N = 1 << 26;


int main(int argc, char* argv[]) {
	
	//static const int ALIGN = 32;
	//alignas(ALIGN) float *x = new float[N], *y = new float[N];
    float *x = new float[N], *y = new float[N];

	for (size_t i = 0; i < N; i++) {
		x[i] = XVAL;
		y[i] = YVAL;
	}

	{
		float *cpu_z = new float[N];
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		// ˆ—
		saxpy(N, AVAL, x, y, cpu_z);
		//‚±‚±‚Ü‚Å
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "cpu : " << msec << " msec" << std::endl;
		verifySaxpy(N, cpu_z);
		delete[] cpu_z;
	}

	{
		float *omp_z = new float[N];
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		saxpy_openMP(N, AVAL, x, y, omp_z);
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "omp : " << msec << " msec" << std::endl;
		verifySaxpy(N, omp_z);
		delete[] omp_z;
	}

	{
		float *sse_z = new float[N];
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		saxpy_SSE(N, AVAL, x, y, sse_z);
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "sse : " << msec << " msec" << std::endl;
		verifySaxpy(N, sse_z);
		delete[] sse_z;
	}

	{
		float *avx_z = new float[N];
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		saxpy_AVX(N, AVAL, x, y, avx_z);
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "avx : " << msec << " msec" << std::endl;
		verifySaxpy(N, avx_z);
		delete[] avx_z;
	}
	delete[] x, y;

	return 0;
}

void saxpy(size_t n, float a, float* x, float* y, float* z)
{
	for (size_t i = 0; i < n; ++i) {
		z[i] = a * x[i] + y[i];
	}
}

void saxpy_openMP(size_t n, float a, float* x, float* y, float* z) {
	int numThread = omp_get_max_threads();
	size_t elemNum = n / numThread + 1;

#pragma omp parallel for num_threads(numThread)
	for (int i = 0; i < numThread; ++i) {
		for (size_t j = elemNum * i; j < elemNum * i + elemNum && j < n; ++j) {
		z[j] = a * x[j] + y[j];
		}
	}
}

void saxpy_SSE(size_t n, float a, float* x, float* y, float* z)
{
	__m128 va = _mm_load1_ps((const float*)&a);
	for (size_t i = 0; i < n; i+= sizeof(va) / sizeof(float)) {
		__m128 vx = _mm_loadu_ps(&x[i]);
		__m128 vy = _mm_loadu_ps(&y[i]);

		//‰‰ŽZ
		__m128 vz = _mm_add_ps(vy, _mm_mul_ps(vx, va));
		_mm_storeu_ps(&z[i], vz);
	}
}

void saxpy_AVX(size_t n, float a, float* x, float* y, float* z)
{
	__m256 va = _mm256_broadcast_ss((const float*)&a);
	for (size_t i = 0; i < n; i += sizeof(va) / sizeof(float)) {
		__m256 vx = _mm256_loadu_ps(&x[i]);
		__m256 vy = _mm256_loadu_ps(&y[i]);

		//‰‰ŽZ
		__m256 vz = _mm256_add_ps(vy, _mm256_mul_ps(vx, va));
		_mm256_storeu_ps(&z[i], vz);
	}
}

void verifySaxpy(size_t n, float* z) {
	float ZVAL = AVAL * XVAL +  YVAL;
	for (size_t i = 0; i < n; ++i) {
		if (ZVAL != z[i])
			std::cout << "error " << i << std::endl;
	}

}