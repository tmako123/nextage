
/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#define _USE_MATH_DEFINES
#include <iostream>
#include <opencv2/opencv.hpp>
#include <DBoW3.h>
#include <chrono>


#include "ioEuRoC.h"

int main()
{
	int numData = 1000;
	std::string dirName("C:/dataset/EuRoC/MH_01_easy/");
	std::string vocName("C:/lib/DBow3/orbvoc.dbow3");
	
	IoEuRoC ioEuRoC;
	ioEuRoC.initialize(dirName);

	cv::Ptr<cv::Feature2D> fdetector;
	fdetector = cv::ORB::create();

	std::vector<cv::Mat> descriptors;

	// calc descriptor
	for (int i = 0; i < numData; i++) {
		cv::Mat image;
		bool ret = ioEuRoC.loadNextImage(image);
		if (ret == false) break;
		cv::imshow("", image);
		cv::waitKey(1);

		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptor;
		fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptor);
		descriptors.push_back(descriptor);
	}

	// load voc
	auto t_start = std::chrono::high_resolution_clock::now();
	DBoW3::Vocabulary voc;
	voc.load(vocName);
	auto t_end = std::chrono::high_resolution_clock::now();
	std::cout << "voc load time=" << double(std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()) << " ms" << std::endl;

	// calc fbow features
	t_start = std::chrono::high_resolution_clock::now();
	std::vector<DBoW3::BowVector> bow3Vec;
	bow3Vec.reserve(descriptors.size());
	for (auto& descriptor : descriptors) {
		DBoW3::BowVector vec;
		voc.transform(descriptor, vec);
		bow3Vec.push_back(vec);
	}
	t_end = std::chrono::high_resolution_clock::now();
	std::cout << "transform time=" << double(std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()) << " ms" << std::endl;

	t_start = std::chrono::high_resolution_clock::now();
	DBoW3::BowVector& vec0 = bow3Vec[0];
	std::vector<double> scores;
	scores.reserve(bow3Vec.size());
	for (auto& vec1 : bow3Vec) {
		double score = voc.score(vec0, vec1);
		scores.push_back(score);
	}
	t_end = std::chrono::high_resolution_clock::now();
	std::cout << "image matching time=" << double(std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()) << " ms" << std::endl;
}

