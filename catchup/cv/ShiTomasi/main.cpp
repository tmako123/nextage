/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <iostream>
#include<opencv2/opencv.hpp>

float
shiTomasiScore(const cv::Mat& img, int u, int v)
{
	assert(img.type() == CV_8UC1);

	float dXX = 0.0;
	float dYY = 0.0;
	float dXY = 0.0;
	const int halfbox_size = 4;
	const int box_size = 2 * halfbox_size;
	const int box_area = box_size * box_size;
	const int x_min = u - halfbox_size;
	const int x_max = u + halfbox_size;
	const int y_min = v - halfbox_size;
	const int y_max = v + halfbox_size;

	if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
		return 0.0; // patch is too close to the boundary

	const int stride = img.step.p[0];
	for (int y = y_min; y < y_max; ++y)
	{
		const uint8_t* ptr_left = img.data + stride * y + x_min - 1;
		const uint8_t* ptr_right = img.data + stride * y + x_min + 1;
		const uint8_t* ptr_top = img.data + stride * (y - 1) + x_min;
		const uint8_t* ptr_bottom = img.data + stride * (y + 1) + x_min;
		for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
		{
			float dx = *ptr_right - *ptr_left;
			float dy = *ptr_bottom - *ptr_top;
			dXX += dx * dx;
			dYY += dy * dy;
			dXY += dx * dy;
			//std::cout << dx << "," << dy << std::endl;

		}
	}

	// Find and return smaller eigenvalue:
	dXX = dXX / (2.0 * box_area);
	dYY = dYY / (2.0 * box_area);
	dXY = dXY / (2.0 * box_area);

	std::cout << dXX << "," << dYY << "," << dXY << std::endl;
	return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
}

bool pairCompare(const std::pair<float, cv::Point2f>& first, const std::pair<float, cv::Point2f>& second)
{
	return first.first > second.first;
}


int main()
{
	cv::Mat img = cv::imread("Lena.png");
	cv::imshow("img", img);
	cv::waitKey(0);

	auto agast = cv::AgastFeatureDetector::create();
	//agast->setThreshold(70);
	std::vector<cv::KeyPoint> keyPoints;
	agast->detect(img, keyPoints);
	//agast->setNonmaxSuppression(true);

	std::vector<cv::Point2f> points;
	cv::KeyPoint::convert(keyPoints, points);

	cv::Mat imgRawAgast = img.clone();
	for (auto& pt : points) {
		cv::circle(imgRawAgast, pt, 3, cv::Scalar(0, 255, 0));
	}
	cv::imshow("rawAgast", imgRawAgast);
	cv::waitKey(0);

	cv::Mat grey;
	cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);

	std::vector<std::pair<float, cv::Point2f>> scoreLists;
	scoreLists.reserve(points.size());
	for (auto& pt : points) {
		float score = shiTomasiScore(grey, (int)pt.x, (int)pt.y);
		scoreLists.push_back(std::pair<float, cv::Point2f>(score, pt));
		std::cout << score << std::endl;
	}
	std::sort(scoreLists.begin(), scoreLists.end(), pairCompare);

	cv::Mat imgPoints50 = img.clone();
	std::vector<cv::Point2f> points50;
	points50.reserve(50);
	for (int i = 0; i < 50; i++) {
		if (scoreLists.size() <= i) break;
		points50.push_back(scoreLists[i].second);
		cv::circle(imgPoints50, scoreLists[i].second, 3, cv::Scalar(0, 255, 0));
	}
	cv::imshow("imgPoints50", imgPoints50);
	cv::waitKey(0);

	cv::Mat imgPointsTh = img.clone();
	std::vector<cv::Point2f> pointsTh;
	points50.reserve(50);
	for (int i = 0; i < scoreLists.size(); i++) {
		if (scoreLists[i].first < 1000) continue;
		pointsTh.push_back(scoreLists[i].second);
		cv::circle(imgPointsTh, scoreLists[i].second, 3, cv::Scalar(0, 255, 0));
	}
	cv::imshow("imgPointsTh", imgPointsTh);
	cv::waitKey(0);



	return 0;
}