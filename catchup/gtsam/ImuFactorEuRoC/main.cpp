/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <string>

#include "vis.h"
#include "ioEuRoC.h"
#include "imuPreintegration.h"

int main(int argc, char* argv[]) {
	//load euroc data
	//path
	std::string dirName("C:/dataset/EuRoC/MH_01_easy/");
	IoEuRoC ioEuRoC;
	ioEuRoC.loadGroundTruth(dirName, 10000, 300);
	ioEuRoC.loadIMU(dirName);

	std::vector<std::pair<double, Eigen::Isometry3d>> gtPoses;
	int count = 0;
	for (auto &ref : ioEuRoC.m_gtPoseList) {
		count++;
		if (count % 100 != 0) continue;
		gtPoses.push_back(std::pair<double, Eigen::Isometry3d>(ref.first, ref.second));
	}

	{
		std::vector<ImuData> imu;
		for (int i = 0; i < gtPoses.size() - 1; i++) {
			double startTime = gtPoses[i].first;
			double endTime = gtPoses[i + 1].first;
			std::vector<ImuData> imu;
			//printf("-- %.0f -> %.0f\n", startTime, endTime);
			ioEuRoC.getInterframeImu(startTime, endTime, imu);
			//for (auto it : imu) {
			//	printf("%.0f\n", it.first);
			//}
		}
	}
	//display
	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		Vis::vizPoses(myWindow, std::string("gtPoses"), gtPoses);
		Vis::vizTrajectory(myWindow, std::string("edge"), gtPoses);
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}

	//imu
	double startTime = gtPoses[0].first;
	double endTime = gtPoses[1].first;
	std::vector<ImuData> imu;
	ioEuRoC.getInterframeImu(startTime, endTime, imu);

	if (imu.size() == 0) return 0;

	double accBias = ioEuRoC.m_accNoiseDensity;
	double gyroBias = ioEuRoC.m_gyroNoiseDensity;
	std::cout << "initialize imu system" << std::endl;
	ImuPreintegration imuSystem;
	imuSystem.initialize(gtPoses[0].first, gtPoses[0].second, imu[0].gyro, accBias, gyroBias);

	std::cout << "start imu system" << std::endl;
	std::vector<std::pair<double, Eigen::Isometry3d>> estPoses;
	for (int i = 0; i < imu.size(); i++) {
		//std::cout << "----------------------" << i << "/" << imu.size() - 1 << std::endl;
		if (i == 0) continue;
		auto& obs = imu[i];
		double time = obs.time;
		Eigen::Vector3d gyro = obs.gyro;
		Eigen::Vector3d acc = obs.acc;
		bool bDebug = false;
		//if (i == imu.size() - 1) bDebug = true;
		Eigen::Isometry3d pose = imuSystem.updateImu(time, gyro, acc, bDebug).inverse();
		estPoses.push_back(std::pair<double, Eigen::Isometry3d>(time, pose));
	}

	//display
	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		Vis::vizPoses(myWindow, std::string("gtPoses"), gtPoses);
		//Vis::vizTrajectory(myWindow, std::string("edge"), gtPoses);
		Vis::vizTrajectory(myWindow, std::string("edge2"), estPoses, cv::viz::Color(0,255,0));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}



}
