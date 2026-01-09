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
#include <random>

#include "factor.h"
#include "happly/happly.h"

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// 自作のヘッダーをインクルード
#include "Simple3DViewer.hpp"

// Eigen::Vector3d のリストを Point3f のリストに変換する補助関数
std::vector<cv::Point3f> eigenToCvPoints(const std::vector<Eigen::Vector3d> &pts)
{
    std::vector<cv::Point3f> cvPts;
    cvPts.reserve(pts.size());
    for (const auto &p : pts)
    {
        cvPts.emplace_back((float)p.x(), (float)p.y(), (float)p.z());
    }
    return cvPts;
}

// Eigen::Isometry3d を cv::Affine3d に変換する補助関数
cv::Affine3d eigenToCvAffine(const Eigen::Isometry3d &pose)
{
    cv::Mat cvMat;
    cv::eigen2cv(pose.matrix(), cvMat);
    return cv::Affine3d(cvMat);
}

int main()
{
    // --- データ読み込み・生成部分はそのまま ---
    happly::PLYData plyIn("../../../cv/VizPly/stanford/bun_zipper.ply");
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();

    std::vector<Eigen::Vector3d> gtPoints3d;
    int k = 0;
    for (auto &pt : vPos)
    {
        if (k++ % 30 != 0)
            continue;
        gtPoints3d.push_back(Eigen::Vector3d(pt.at(0), pt.at(1), pt.at(2)) * 50);
    }

    // カメラ生成（Ground Truth）
    int num_pose = 40;
    float radius = 10.0f;
    std::vector<Eigen::Isometry3d> gtPoses;
    for (int i = -3; i <= 3; i++)
    {
        double rad = 2 * M_PI / num_pose * i;
        Eigen::AngleAxisd rot(rad + M_PI, Eigen::Vector3d(0, 1, 0));
        Eigen::Vector3d trans(sin(rad), 0.5, cos(rad));
        Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
        pose.prerotate(rot);
        pose.pretranslate(trans * radius);
        gtPoses.push_back(pose);
    }

    // generate raugh points and camera
    double mu = 0., sigma = 1;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    std::vector<Eigen::Vector3d> points3d;
    std::vector<Eigen::Vector3d> noisedPoints3d;
    for (auto &pt : gtPoints3d)
    {
        Eigen::Vector3d noisePoint3d = pt + Eigen::Vector3d(dist(engine), dist(engine), dist(engine));
        points3d.push_back(noisePoint3d);
        noisedPoints3d.push_back(noisePoint3d);
    }

    std::vector<Eigen::Isometry3d> poses;       // c2w
    std::vector<Eigen::Isometry3d> noisedPoses; // c2w
    for (int i = 0; i < gtPoses.size(); i++)
    {
        Eigen::Isometry3d pose = gtPoses[i];
        if (i > 2)
        {
            pose.translation() += Eigen::Vector3d(dist(engine), dist(engine), dist(engine));
        }
        poses.push_back(pose);
        noisedPoses.push_back(pose);
    }

    Simple3DViewer viewer;

    // 1. Ground Truth (緑)
    viewer.addCloud(eigenToCvPoints(gtPoints3d), cv::Scalar(0, 255, 0), 1);
    for (const auto &p : gtPoses)
    {
        viewer.addCamera(eigenToCvAffine(p), cv::Scalar(0, 255, 0), 0.5f, 1);
    }

    // 2. Noised Data (水色)
    viewer.addCloud(eigenToCvPoints(noisedPoints3d), cv::Scalar(255, 255, 0), 1);
    for (const auto &p : noisedPoses)
    {
        viewer.addCamera(eigenToCvAffine(p), cv::Scalar(255, 255, 0), 0.5f, 1);
    }

    // ウィンドウを表示してループ
    std::cout << "Showing results. Press 'q' on the window to exit." << std::endl;
    viewer.show("Bundle Adjustment Result (VTK-free)");

    // generate projection
    double f = 400;
    double w = 640;
    double h = 480;
    std::vector<std::vector<Eigen::Vector2d>> observations;
    std::vector<std::vector<Eigen::Vector2d>> observationsF;
    for (auto &pose : gtPoses)
    {
        std::vector<Eigen::Vector2d> observationCam;
        std::vector<Eigen::Vector2d> observationCamF;
        for (auto &pt : gtPoints3d)
        {
            Eigen::Vector3d pt3d_cam = pose.inverse() * pt;
            Eigen::Vector2d pt2d_norm = pt3d_cam.head<2>() / pt3d_cam.z();
            Eigen::Vector2d pt2d = pt2d_norm * f + Eigen::Vector2d(w, h) * 0.5;
            observationCam.push_back(pt2d_norm);
            observationCamF.push_back(pt2d);
        }
        observations.push_back(observationCam);
        observationsF.push_back(observationCamF);
    }

#if 0
	//generate show image
    for (auto& obsCam : observationsF) {
        cv::Mat image = cv::Mat::zeros(h, w, CV_8UC3);
        for (auto& obs : obsCam) {
            cv::circle(image, cv::Point(obs.x(), obs.y()), 2, cv::Scalar(0, 255, 0), -1);
        }
        cv::imshow("", image);
        cv::waitKey(33);
    }
#endif

    // oprimize
    double POSE[7][7];
    double POINT[2000][3];

    for (int i = 0; i < poses.size(); i++)
    {
        Eigen::Isometry3d pose = poses[i];
        POSE[i][0] = pose.translation().x();
        POSE[i][1] = pose.translation().y();
        POSE[i][2] = pose.translation().z();
        Eigen::Quaterniond q{pose.rotation()};
        POSE[i][3] = q.w();
        POSE[i][4] = q.x();
        POSE[i][5] = q.y();
        POSE[i][6] = q.z();
    }

    for (int i = 0; i < points3d.size(); i++)
    {
        Eigen::Vector3d pt3d = points3d[i];
        POINT[i][0] = pt3d.x();
        POINT[i][1] = pt3d.y();
        POINT[i][2] = pt3d.z();
    }

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < 7; i++)
    {
        ceres::Manifold *manifold = new ceres::ProductManifold(
            new ceres::EuclideanManifold<3>(), // translation
            new ceres::QuaternionManifold()    // rotation
        );

        problem.AddParameterBlock(POSE[i], 7, manifold);
        if (i < 2)
        {
            problem.SetParameterBlockConstant(POSE[i]);
        }
    }

    for (int i = 0; i < points3d.size(); i++)
    {
        for (int j = 0; j < 7; j++)
        {
            Eigen::Vector2d obs = observations[j][i];

            // ProjectionFactor* f = new ProjectionFactor(obs);
            // problem.AddResidualBlock(f, loss_function, POSE[j], POINT[i]);

            // ceres::CostFunction *f = new ceres::NumericDiffCostFunction<ProjectionFactor, ceres::CENTRAL, 2, 7, 3>(
            //     new ProjectionFactor(obs), ceres::TAKE_OWNERSHIP);
            // problem.AddResidualBlock(f, loss_function, POSE[j], POINT[i]);

            // AutoDiffFunction
            ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<autoDiffProjectionFactor, 2, 7, 3>(
                    new autoDiffProjectionFactor(obs));
            problem.AddResidualBlock(cost_function, loss_function, POSE[j], POINT[i]);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = true;
    // options.num_threads = 2;
    // options.use_explicit_schur_complement = true;
    // options.use_nonmonotonic_steps = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    for (int i = 0; i < poses.size(); i++)
    {
        Eigen::Vector3d trans(POSE[i][0], POSE[i][1], POSE[i][2]);
        Eigen::Quaterniond q;
        q.w() = POSE[i][3];
        q.x() = POSE[i][4];
        q.y() = POSE[i][5];
        q.z() = POSE[i][6];

        Eigen::Isometry3d pose;
        pose.setIdentity();
        pose.prerotate(q.normalized().toRotationMatrix());
        pose.pretranslate(trans);
        poses[i] = pose;
    }

    for (int i = 0; i < points3d.size(); i++)
    {
        points3d[i].x() = POINT[i][0];
        points3d[i].y() = POINT[i][1];
        points3d[i].z() = POINT[i][2];
    }

    // --- Simple3DViewer を使った表示 ---

    viewer.clear();

    // 1. Ground Truth (緑)
    viewer.addCloud(eigenToCvPoints(gtPoints3d), cv::Scalar(0, 255, 0), 1);
    for (const auto &p : gtPoses)
    {
        viewer.addCamera(eigenToCvAffine(p), cv::Scalar(0, 255, 0), 0.5f, 1);
    }

    // 2. Noised Data (水色)
    viewer.addCloud(eigenToCvPoints(noisedPoints3d), cv::Scalar(255, 255, 0), 1);
    for (const auto &p : noisedPoses)
    {
        viewer.addCamera(eigenToCvAffine(p), cv::Scalar(255, 255, 0), 0.5f, 1);
    }

    // 3. Optimized Result (黄)
    viewer.addCloud(eigenToCvPoints(points3d), cv::Scalar(0, 255, 255), 1);
    for (const auto &p : poses)
    {
        viewer.addCamera(eigenToCvAffine(p), cv::Scalar(0, 255, 255), 0.5f, 1);
    }

    // ウィンドウを表示してループ
    std::cout << "Showing results. Press 'q' on the window to exit." << std::endl;
    viewer.show("Bundle Adjustment Result (VTK-free)");

    return 0;
}