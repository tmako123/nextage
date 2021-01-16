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

bool loadGroundTruth(const std::string& dirName, std::map<double, Eigen::Isometry3f>& refPoseList)
{
    std::string gtFilename = dirName + "mav0/state_groundtruth_estimate0/data.csv";
    std::ifstream fi(gtFilename);
    if (fi.is_open() == false) {
        return false;
    }

    std::string cam0FileName = dirName + "mav0/cam0/sensor.yaml";
    std::vector<double> yOffsetCam;
    try {
        YAML::Node yaml = YAML::LoadFile(cam0FileName);
        yOffsetCam = yaml["T_BS"]["data"].as<std::vector<double>>();
    } catch (YAML::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
    Eigen::Matrix4d offsetCam = Eigen::Map<Eigen::Matrix4d>(&yOffsetCam[0]).transpose();

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
        refPoseList.insert(std::make_pair(timeStamp, refPose.inverse().cast<float>()));
    }

    return true;
}
}
