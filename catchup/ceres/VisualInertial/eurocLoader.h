/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <string>
#include <yaml-cpp/yaml.h>

#include "happly/happly.h"

struct IMU {
    Eigen::Vector3d acc, gyro;
    double time;
};

namespace EuRoCLoader {
bool loadIum(const std::string& dirName, std::vector<IMU>& imus)
{
    std::string line;
    std::ifstream imu_file(dirName + "mav0/imu0/data.csv");
    if (imu_file.is_open() == false) {
        return false;
    }

    std::getline(imu_file, line);
    while (true) {
        if (!std::getline(imu_file, line)) {
            break;
        }

        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');

        double t = std::stod(s);

        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            gyr[j] = std::stof(s);
        }

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            acc[j] = std::stof(s);
        }

        IMU imu;
        imu.acc = acc;
        imu.gyro = gyr;
        imu.time = t * 1e-9;
        //std::cout << std::setprecision(15) << imu.t << std::endl;
        imus.push_back(imu);
    }
    return true;
}

bool loadGroundTruth(const std::string& dirName, std::map<double, Eigen::Isometry3d>& refPoseList)
{
    std::string gtFilename = dirName + "mav0/state_groundtruth_estimate0/data.csv";
    std::ifstream fi(gtFilename);
    if (fi.is_open() == false) {
        return false;
    }

#if 0
    std::string cam0FileName = dirName + "mav0/cam0/sensor.yaml";
    std::vector<double> yOffsetCam;
    try {
        YAML::Node yaml = YAML::LoadFile(cam0FileName);
        yOffsetCam = yaml["T_BS"]["data"].as<std::vector<double>>();
    } catch (YAML::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
    Eigen::Matrix4d offsetCam = Eigen::Map<Eigen::Matrix4d>(&yOffsetCam[0]).transpose();
#else
    Eigen::Matrix4d offsetCam = Eigen::Matrix4d::Identity();
#endif

    refPoseList.clear();

    std::string str;
    std::getline(fi, str);
    while (std::getline(fi, str)) {
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
        refPoseList.insert(std::make_pair(timeStamp, refPose.inverse()));
    }

    return true;
}

bool loadOffset(const std::string& dirName, Eigen::Isometry3d& icl, Eigen::Isometry3d& icr)
{
    std::string cam0FileName = dirName + "mav0/cam0/sensor.yaml";
    std::vector<double> yOffsetCamL;
    try {
        YAML::Node yaml = YAML::LoadFile(cam0FileName);
        yOffsetCamL = yaml["T_BS"]["data"].as<std::vector<double>>();
    } catch (YAML::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
    Eigen::Matrix4d offsetCamL = Eigen::Map<Eigen::Matrix4d>(&yOffsetCamL[0]).transpose();
    icl = offsetCamL.inverse();

    std::string cam1FileName = dirName + "mav0/cam1/sensor.yaml";
    std::vector<double> yOffsetCamR;
    try {
        YAML::Node yaml = YAML::LoadFile(cam1FileName);
        yOffsetCamR = yaml["T_BS"]["data"].as<std::vector<double>>();
    } catch (YAML::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
    Eigen::Matrix4d offsetCamR = Eigen::Map<Eigen::Matrix4d>(&yOffsetCamR[0]).transpose();
    icr = offsetCamR.inverse();
    return true;
}

bool loadPoints(const std::string& dirName, std::vector<Eigen::Vector3d>& points) {
    std::string pointFileName = dirName + "mav0/points.ply";
    happly::PLYData plyIn(pointFileName);
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();
    double minX = 0;
    points.reserve(vPos.size());
    for (auto& pos : vPos) {
        points.push_back(Eigen::Vector3d(pos[0], pos[1], pos[2]));
	}
    return true;
}
}