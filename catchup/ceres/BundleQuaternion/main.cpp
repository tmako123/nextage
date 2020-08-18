/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#define _USE_MATH_DEFINES
#include "happly/happly.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

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
        cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, color); // Camera frustum
        window.showWidget(widgetFrustumName, cpw_frustum, camPose);
    }
}

int main()
{

    ///read ply
    happly::PLYData plyIn("../../../cv/VizPly/stanford/bun_zipper.ply");
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    std::vector<Eigen::Vector3d> points3d;
    int i = 0;
    for (auto& pt : vPos) {
        if (i++ % 30 != 0)
            continue;
        points3d.push_back(Eigen::Vector3d(pt.at(0), pt.at(1), pt.at(2)) * 50);
    }

    //generate camera
    int num_pose = 40;
    int radius = 10;
    std::vector<Eigen::Isometry3d> gtPoses; //w2c
    //gtPose generation
    for (int i = -3; i <= 3; i++) {
        double rad = 2 * M_PI / num_pose * i;
        Eigen::AngleAxisd rot(rad, Eigen::Vector3d(0, 1, 0));
        Eigen::Vector3d trans(sin(rad), 0.5, cos(rad));
        Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
        pose.prerotate(rot);
        pose.pretranslate(trans * radius);
        gtPoses.push_back(pose.inverse());
    }

    ///show with viz
    cv::viz::Viz3d myWindow("Point Cloud");
    vizPoints(myWindow, std::string("points3d"), points3d, cv::viz::Color(0, 0, 255));
    vizPoses(myWindow, std::string("camera"), gtPoses, cv::viz::Color(0, 0, 255));

    myWindow.spinOnce(1, true);
    myWindow.spin();
    myWindow.removeAllWidgets();

    //generate projection
    double f = 400;
    double w = 640;
    double h = 480;
    std::vector<std::vector<Eigen::Vector2d>> observations;
    std::vector<std::vector<Eigen::Vector2d>> observationsF;
    for (auto& pose : gtPoses) {
        std::vector<Eigen::Vector2d> observationCam;
        std::vector<Eigen::Vector2d> observationCamF;
        for (auto& pt : points3d) {
            Eigen::Vector3d pt3d_cam = pose * pt;
            Eigen::Vector2d pt2d_norm = pt3d_cam.head<2>() / pt3d_cam.z();
            Eigen::Vector2d pt2d = pt2d_norm * f + Eigen::Vector2d(w / 2, h / 2);
            observationCam.push_back(pt2d_norm);
            observationCamF.push_back(pt2d);
        }
        observations.push_back(observationCam);
        observationsF.push_back(observationCamF);
    }

    for (auto& obsCam : observationsF) {
        cv::Mat image = cv::Mat::zeros(h, w, CV_8UC3);
        for (auto& obs : obsCam) {
            cv::circle(image, cv::Point(obs.x(), obs.y()), 2, cv::Scalar(0, 255, 0), -1);
        }
        cv::imshow("", image);
        cv::waitKey(0);
    }

    //generate show image

    return 0;
}