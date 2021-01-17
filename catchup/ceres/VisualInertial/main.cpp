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
#include <algorithm>
#include <ceres/ceres.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <random>
#include <unordered_map>

#include "eurocLoader.h"

#define DATA_EUROC "../../../ceres/VisualInertial/EuRoC_data/MH_01_easy/"
const double f = 400;
const int width = 640;
const int height = 480;
const double cx = (width - 1) * 0.5;
const double cy = (height - 1) * 0.5;

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

void vizAxis(cv::viz::Viz3d& window, std::string& name, Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() /*w2c*/, double scale = 0.1)
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
    cv::Affine3d cvPose;
    cv::eigen2cv(Eigen::Affine3d(pose.inverse()).matrix(), cvPose.matrix);
    window.showWidget(widgetLineNameVexX, axisX, cvPose);
    window.showWidget(widgetLineNameVexY, axisY, cvPose);
    window.showWidget(widgetLineNameVexZ, axisZ, cvPose);
}

void vizAxis(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Isometry3d> poses /*w2c*/, double scale = 0.1)
{
    for (int i = 0; i < poses.size(); i++) {
        std::string widgetName = name + std::to_string(i);
        vizAxis(window, widgetName, poses[i]);
    }
}

struct Frame {
    std::unordered_map<int, Eigen::Vector2f> featuresL;
    std::unordered_map<int, Eigen::Vector2f> featuresR;
    std::unordered_map<int, Eigen::Vector2f> normfeaturesL;
    std::unordered_map<int, Eigen::Vector2f> normfeaturesR;
    double time;
};

void displayFrame(const std::string& win, Frame& frame, int LR = 0)
{
    std::unordered_map<int, Eigen::Vector2f>* features;
    if (LR == 0) {
        features = &frame.featuresL;
    } else {
        features = &frame.featuresR;
    }

    cv::Mat image = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    for (auto f : *features) {
        cv::circle(image, cv::Point2f(f.second.x(), f.second.y()), 2, cv::Scalar(0, 255, 0));
    }
    cv::imshow(win, image);
}

int main()
{
    //load poses
    std::vector<IMU> imus;
    std::map<double, Eigen::Isometry3d> poseLists;
    Eigen::Isometry3d icL, icR;
    std::vector<Eigen::Vector3d> points3d;
    EuRoCLoader::loadGroundTruth(DATA_EUROC, poseLists);
    EuRoCLoader::loadIum(DATA_EUROC, imus);
    EuRoCLoader::loadOffset(DATA_EUROC, icL, icR);
    EuRoCLoader::loadPoints(DATA_EUROC, points3d);

//show loaded data
#if 1
    {
        std::vector<Eigen::Isometry3d> imuPoses, cameraPosesL, cameraPosesR; //w2c
        int count = 0;
        for (auto& pose : poseLists) {
            if (count++ % 300 == 0) {
                imuPoses.push_back(pose.second);
                cameraPosesL.push_back(icL * pose.second);
                cameraPosesR.push_back(icR * pose.second);
            }
        }

        ///show all poses with viz
        cv::viz::Viz3d myWindow("Point Cloud");
        vizPoses(myWindow, std::string("cameraL"), cameraPosesL, cv::viz::Color(0, 255, 0));
        vizPoses(myWindow, std::string("cameraR"), cameraPosesR, cv::viz::Color(0, 255, 0));
        vizAxis(myWindow, std::string("axis"), imuPoses);
        vizLines(myWindow, std::string("imuTrajectory"), imuPoses);
        vizAxis(myWindow, std::string("axis"), Eigen::Isometry3d::Identity(), 1.);
        vizPoints(myWindow, std::string("point3d"), points3d, cv::viz::Color(255, 255, 255));
        myWindow.spinOnce(1, true);
        myWindow.spin();
        myWindow.removeAllWidgets();
    }
#endif

    std::vector<Frame> frames;

    //create frame
    {
        double prevTime = -1;
        for (auto& pose : poseLists) {
            double time = pose.first;
            //50ms interval
            if (time - prevTime < 0.05) {
                continue;
            }
            std::cout << std::setprecision(13) << time << std::endl;
            Eigen::Isometry3d poseCamL = icL * pose.second;
            Eigen::Isometry3d poseCamR = icR * pose.second;
            prevTime = time;

            Frame frame;
            frame.time = time;

            //projection
            int id = -1;
            for (auto& p_w : points3d) {
                id++;
                Eigen::Vector3d p_cl = poseCamL * p_w;
                Eigen::Vector3d p_cr = poseCamR * p_w;
                if (p_cl.z() > 10.0 || p_cl.z() < 0.010 || p_cr.z() > 10.0 || p_cr.z() < 0.010) {
                    continue;
                }

                //projection point
                double invZL = 1.0 / p_cl.z();
                double invZR = 1.0 / p_cl.z();
                Eigen::Vector2d normPos2dL(p_cl.x() * invZL, p_cl.y() * invZL);
                Eigen::Vector2d normPos2dR(p_cr.x() * invZR, p_cr.y() * invZR);
                Eigen::Vector2d pos2dL = normPos2dL * f + Eigen::Vector2d(cx, cy);
                Eigen::Vector2d pos2dR = normPos2dR * f + Eigen::Vector2d(cx, cy);

                if (pos2dL.x() < 1 || pos2dL.x() > width - 2 || pos2dL.y() < 1 || pos2dL.y() > height - 2) {
                    continue;
                }
                if (pos2dR.x() < 1 || pos2dR.x() > width - 2 || pos2dR.y() < 1 || pos2dR.y() > height - 2) {
                    continue;
                }

                //register
                frame.featuresL[id] = pos2dL.cast<float>();
                frame.featuresR[id] = pos2dR.cast<float>();
                frame.normfeaturesL[id] = normPos2dL.cast<float>();
                frame.normfeaturesR[id] = normPos2dR.cast<float>();
            }
            displayFrame("frameL", frame);
            displayFrame("frameR", frame, 1);
            cv::waitKey(0);
        }
    }

    return 0;
}