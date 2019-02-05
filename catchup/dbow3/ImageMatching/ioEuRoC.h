#pragma once

#include <opencv2/opencv.hpp>
class IoEuRoC
{
public:
	IoEuRoC() :m_frameNum(980) {};
	~IoEuRoC() {};

	bool initialize(const std::string& path);
	bool loadNextImage(cv::Mat& image);

protected:
	std::vector<std::pair<double, std::string>> m_imagePathList;
	int m_frameNum;
	std::string m_dirName;
};

