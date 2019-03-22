/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <vector>
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
	const int n = 1 << 13;
	std::vector<std::vector<float>> a(n, std::vector<float>(n));
	std::vector<std::vector<float>> b(n, std::vector<float>(n));

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			a[j][i] = (float)rand();
			b[j][i] = a[j][i];
		}
	}

	{
		//access verticle
		float sum = 0.f;
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				sum += a[j][i];
			}
		}
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "verticle : " << msec << " msec" << std::endl;

		//Ban optimization!
		std::cout << "sum-> " << sum << std::endl;
	}

	{
		//access horizontal
		float sum = 0.f;
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				sum += a[i][j];
			}
		}
		end = std::chrono::system_clock::now();
		auto elapsed = end - start;
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
		std::cout << "horizontal : " << msec << " msec" << std::endl;

		//Ban optimization!
		std::cout << "sum-> " << sum << std::endl;
	}

}