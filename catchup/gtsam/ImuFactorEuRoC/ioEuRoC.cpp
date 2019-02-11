#include "ioEuRoC.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <yaml-cpp/yaml.h>

bool IoEuRoC::loadGroundTruth(const std::string &dirName, int startFrame, int endFrameNum)
{
	std::string gtFilename = dirName + "mav0/state_groundtruth_estimate0/data.csv";
	std::ifstream fi(gtFilename);
	if (fi.is_open() == false) {
		return false;
	}

	//IMU->カメラオフセット読み込み

	std::string cam0FileName = dirName + "mav0/cam0/sensor.yaml";
	std::vector<double> yOffsetCam;
	try {
		YAML::Node yaml = YAML::LoadFile(cam0FileName);
		yOffsetCam = yaml["T_BS"]["data"].as<std::vector<double>>();
	}
	catch (YAML::Exception& e) {
		std::cerr << e.what() << std::endl;
	}
	Eigen::Matrix4d offsetCam = Eigen::Map<Eigen::Matrix4d>(&yOffsetCam[0]).transpose();


	std::string str;
	std::getline(fi, str);
	int count = 0;
	while (std::getline(fi, str)) {
		count++;
		if (count < startFrame) continue;
		if (endFrameNum > 0 && count > endFrameNum + startFrame) continue;
		double timeStamp;
		double vecT[3];
		double rot[4];
		sscanf(str.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &timeStamp, &vecT[0], &vecT[1], &vecT[2], &rot[0], &rot[1], &rot[2], &rot[3]);

		Eigen::Quaterniond quat;
		quat.w() = rot[0];
		quat.x() = rot[1];
		quat.y() = rot[2];
		quat.z() = rot[3];
		Eigen::Matrix3d rotation = quat.toRotationMatrix();

		Eigen::Vector3d trans(vecT[0], vecT[1], vecT[2]);

		Eigen::Isometry3d refPose;
		refPose.setIdentity();
		refPose.prerotate(rotation);
		refPose.pretranslate(Eigen::Vector3d(trans.x(), trans.y(), trans.z()));

		refPose = refPose * Eigen::Isometry3d(offsetCam);
		m_gtPoseList.insert(std::make_pair(timeStamp / 10e8, refPose.inverse()));
	}

	return true;
}

bool IoEuRoC::loadIMU(const std::string &dirName) {
	std::string imuFilename = dirName + "mav0/imu0/data.csv";
	std::ifstream fi(imuFilename);
	if (fi.is_open() == false) {
		return false;
	}

	std::string cam0FileName = dirName + "mav0/imu0/sensor.yaml";
	try {
		YAML::Node yaml = YAML::LoadFile(cam0FileName);
		m_gyroNoiseDensity = yaml["gyroscope_noise_density"].as<double>();
		m_gyroRandomWalk = yaml["gyroscope_random_walk"].as<double>();
		m_accNoiseDensity = yaml["accelerometer_noise_density"].as<double>();
		m_accRandomWalk = yaml["accelerometer_random_walk"].as<double>();
	}
	catch (YAML::Exception& e) {
		std::cerr << e.what() << std::endl;
	}

	std::string str;
	std::getline(fi, str);
	int count = 0;
	while (std::getline(fi, str)) {
		count++;
		double timeStamp;
		double vw[3];
		double acc[3];
		sscanf(str.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &timeStamp, &vw[0], &vw[1], &vw[2], &acc[0], &acc[1], &acc[2]);

		Eigen::Vector3d vwVec(vw[0], vw[1], vw[2]);
		Eigen::Vector3d accVec(acc[0], acc[1], acc[2]);
		std::pair<Eigen::Vector3d, Eigen::Vector3d> imu(vwVec, accVec);

		//std::cout << "gyro: " << vwVec.transpose() << std::endl;
		//std::cout << "acc: " << accVec.transpose() << std::endl;

		m_imu.insert(std::make_pair(timeStamp / 10e8, imu));
	}
}

void IoEuRoC::getInterframeImu(double beginTime, double endTime, std::vector<ImuData> &imu){
	auto it = m_imu.upper_bound(beginTime - 1);
	auto last = m_imu.lower_bound(endTime - 1);
	while (it != last) {
		ImuData d;
		d.time = it->first;
		d.gyro = it->second.first;
		d.acc = it->second.second;
		imu.push_back(d);
		it++;
	}
}

