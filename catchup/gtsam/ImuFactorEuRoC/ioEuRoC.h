#pragma once

#include <Eigen/Dense>
#include <memory>
#include <map>
#include <vector>

struct ImuData {
	double time;
	Eigen::Vector3d acc;
	Eigen::Vector3d gyro;
};

class IoEuRoC
{
public:
	IoEuRoC() {};
	~IoEuRoC() {};


	bool loadGroundTruth(const std::string &dirName, int startFrame = 0, int endFrameNum = -1);
	bool loadIMU(const std::string &dirName);
	
	void getInterframeImu(double beginTime, double endTime, std::vector<ImuData> &imu);

	std::map<double, Eigen::Isometry3d> m_gtPoseList;
	std::map<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>> m_imu;

	double m_gyroNoiseDensity;
	double m_gyroRandomWalk;
	double m_accNoiseDensity;
	double m_accRandomWalk;

};

