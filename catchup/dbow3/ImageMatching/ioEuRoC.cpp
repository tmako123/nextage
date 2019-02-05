#include "ioEuRoC.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

bool IoEuRoC::initialize(const std::string &path)
{
	m_dirName = path;
	std::ifstream fi(m_dirName + "mav0/cam0/data.csv");
	if (fi.is_open() == false) {
		return false;
	}

	m_imagePathList.clear();

	std::string str;
	getline(fi, str);
	while (getline(fi, str)) {

		std::stringstream ss{ str };
		std::string buf;
		std::getline(ss, buf, ',');
		double timeStamp = std::stod(buf);
		std::getline(ss, buf, ',');
		std::string imagePathStr = buf;

		m_imagePathList.push_back(std::pair<double, std::string>(timeStamp, imagePathStr));

	}

	return true;
}

bool IoEuRoC::loadNextImage(cv::Mat& image) {
	if (m_frameNum >= m_imagePathList.size()) {
		return false;
	}

	std::ostringstream sout;
	sout << std::setfill('0') << std::setw(5) << m_frameNum;
	std::string strNum = sout.str();

	double imageTimeStamp = m_imagePathList[m_frameNum].first;
	std::string imageFileName = m_imagePathList[m_frameNum].second;
	std::string leftImgPath(m_dirName + std::string("mav0/cam0/data/"));
	image = cv::imread(leftImgPath + imageFileName);
	if (image.data == NULL) {
		return false;
	}
	m_frameNum++;
	return true;

}
