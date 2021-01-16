/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <random>

#include "eurocLoader.h"

#define DATA_EUROC "../../../ceres/VisualInertial/EuRoC_data/MH_01_easy/"

void eigen2cvPoint(const Eigen::Vector3d& eigen, cv::Point3d& cv)
{
    cv = cv::Vec3d(eigen.x(), eigen.y(), eigen.z());
}

void vizPoints(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Vector3d> points3d, cv::viz::Color color = cv::viz::Color(255, 255, 255))
{
    std::vector<cv::Vec3d> cvPoints3d(points3d.size());
    for (int i = 0; i < points3d.size(); i++) {
        cv::Vec3d pt3d(points3d[i].x(), points3d[i].y(), points3d[i].z());
        cvPoints3d[i] = pt3d;
    }
    cv::viz::WCloud wcloud(cvPoints3d, color);
    window.showWidget(name, wcloud);
}

void vizPoses(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Isometry3d> poses /*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255))
{
    for (int i = 0; i < poses.size(); i++) {
        cv::Affine3d camPose;
        cv::eigen2cv(Eigen::Affine3d(poses[i].inverse()).matrix(), camPose.matrix);
        std::string widgetFrustumName = name + std::to_string(i);
        cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), 0.1, color); // Camera frustum
        window.showWidget(widgetFrustumName, cpw_frustum, camPose);
    }
}

void vizLines(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Isometry3d> poses /*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255))
{

    std::vector<cv::Point3d> points;
    for (int i = 0; i < poses.size(); i++) {
        auto pos = poses[i].inverse().translation();
        cv::Point3d cvPoint3d;
        eigen2cvPoint(pos, cvPoint3d);
        points.push_back(cvPoint3d);
    }
    cv::viz::WPolyLine w_lines(points, color);
    window.showWidget(name, w_lines);
}

void visAxis(cv::viz::Viz3d& window, std::string& name, Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() /*w2c*/, double scale = 0.1)
{
    Eigen::Vector3d vecX(scale, 0, 0);
    Eigen::Vector3d vecY(0, scale, 0);
    Eigen::Vector3d vecZ(0, 0, scale);
    Eigen::Vector3d origin(0, 0, 0);
    cv::Point3d cvPoint3dX, cvPoint3dY, cvPoint3dZ, cvPoint3dOrigin;
    eigen2cvPoint(vecX, cvPoint3dX);
    eigen2cvPoint(vecY, cvPoint3dY);
    eigen2cvPoint(vecZ, cvPoint3dZ);
    eigen2cvPoint(origin, cvPoint3dOrigin);
    std::string widgetLineNameVexX = name + "vecX";
    std::string widgetLineNameVexY = name + "vecY";
    std::string widgetLineNameVexZ = name + "vecZ";
    cv::viz::WLine axisX(cvPoint3dOrigin, cvPoint3dX, cv::viz::Color::red());
    cv::viz::WLine axisY(cvPoint3dOrigin, cvPoint3dY, cv::viz::Color::green());
    cv::viz::WLine axisZ(cvPoint3dOrigin, cvPoint3dZ, cv::viz::Color::blue());
    window.showWidget(widgetLineNameVexX, axisX);
    window.showWidget(widgetLineNameVexY, axisY);
    window.showWidget(widgetLineNameVexZ, axisZ);
}

int main()
{
    std::vector<IMU> imus;
    std::map<double, Eigen::Isometry3f> poseLists;
    EuRoCLoader::loadGroundTruth(DATA_EUROC, poseLists);
    EuRoCLoader::loadIum(DATA_EUROC, imus);

    std::vector<Eigen::Isometry3d> poses; //w2c
    int count = 0;
    for (auto& pose : poseLists) {
        if (count++ % 300 == 0) {
            poses.push_back(pose.second.cast<double>());
        }
    }

    //gtPose generation

    ///show with viz
    cv::viz::Viz3d myWindow("Point Cloud");
    vizPoses(myWindow, std::string("g_camera"), poses, cv::viz::Color(0, 255, 0));
    vizLines(myWindow, std::string("g_camera_line"), poses);
    visAxis(myWindow, std::string("axis"), Eigen::Isometry3d::Identity(), 1.);
    myWindow.spinOnce(1, true);
    myWindow.spin();
    myWindow.removeAllWidgets();

    return 0;
}