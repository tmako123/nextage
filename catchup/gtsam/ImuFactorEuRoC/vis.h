#pragma once

#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/core/eigen.hpp>

namespace Vis{
	void vizPoses(cv::viz::Viz3d& window, std::string& name, std::vector<std::pair<double, Eigen::Isometry3d>> poses/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255));
	void vizTrajectory(cv::viz::Viz3d& window, std::string& name, std::vector<std::pair<double, Eigen::Isometry3d>> poses/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255));

};

