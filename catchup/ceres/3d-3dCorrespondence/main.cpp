/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 *******************************************************/

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "../common/happly/happly.h"
#include "../common/Simple3DViewer.hpp"

constexpr bool USE_AUTO_DIFF = false;

// 自由度6の最適化のためのカスタムマニフォールド
class PoseManifold : public ceres::Manifold
{
public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override
    {
        Eigen::Map<const Eigen::Vector3d> p(x);
        Eigen::Map<const Eigen::Quaterniond> q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);
        Eigen::Map<const Eigen::Vector3d> d_theta(delta + 3);

        // 角度をノルムとした回転ベクトルからクォータニオンを生成 (近似ではなく厳密な指数写像)
        // もしくは提示の 0.5 * d_theta による近似（微小変化なら十分）
        Eigen::Quaterniond dq;
        double theta = d_theta.norm();
        if (theta < 1e-10)
        {
            dq = Eigen::Quaterniond(1.0, 0.5 * d_theta.x(), 0.5 * d_theta.y(), 0.5 * d_theta.z());
        }
        else
        {
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, d_theta / theta));
        }

        Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

        p_plus = p + dp;
        q_plus = (q * dq).normalized();
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
        J.setZero();
        // パススルー設定: Evaluateでの3x6ヤコビアンをそのまま使う
        J.topRows<6>().setIdentity();
        return true;
    }

    bool Minus(const double *y, const double *x, double *y_minus_x) const override { return false; }
    bool MinusJacobian(const double *x, double *jacobian) const override { return false; }
};

// --- Auto Diff 用 ---
struct AutoDiffPointFactor
{
    AutoDiffPointFactor(const Eigen::Vector3d &model, const Eigen::Vector3d &obs)
        : model_(model), obs_(obs) {}

    template <typename T>
    bool operator()(const T *const pose_ptr, T *residuals) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q(pose_ptr + 3);

        Eigen::Matrix<T, 3, 1> p_pred = (q * model_.cast<T>()) + t;
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
        res = obs_.cast<T>() - p_pred;
        return true;
    }
    const Eigen::Vector3d model_, obs_;
};

// --- 手動微分用 ---
class AnalyticPointFactor : public ceres::SizedCostFunction<3, 7>
{
public:
    AnalyticPointFactor(const Eigen::Vector3d &model, const Eigen::Vector3d &obs)
        : model_(model), obs_(obs) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
    {
        Eigen::Map<const Eigen::Vector3d> t(parameters[0]);
        Eigen::Map<const Eigen::Quaterniond> q(parameters[0] + 3);

        Eigen::Vector3d rotated_pt = q * model_;
        Eigen::Vector3d p_pred = rotated_pt + t;

        // 残差
        Eigen::Map<Eigen::Vector3d> res(residuals);
        res = p_pred - obs_;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();

            // 1. 並進微分: dr/dt = I (最初の3列)
            J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

            // 2. 回転微分: ^R*[model]_x (続く3列)
            Eigen::Matrix3d skew;
            skew << 0, -model_.z(), model_.y(),
                model_.z(), 0, -model_.x(),
                -model_.y(), model_.x(), 0;

            J.block<3, 3>(0, 3) = -q.toRotationMatrix() * skew;
        }
        return true;
    }
    const Eigen::Vector3d model_, obs_;
};

int main()
{
    happly::PLYData plyIn("../../../cv/VizPly/stanford/bun_zipper.ply");
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<Eigen::Vector3d> modelPoints3d;
    int k = 0;
    for (auto &pt : vPos)
    {
        if (k++ % 30 != 0)
            continue;
        modelPoints3d.push_back(Eigen::Vector3d(pt.at(0), pt.at(1), pt.at(2)) * 50);
    }

    double rad = 2.0 * M_PI / 10;
    Eigen::Isometry3d modelPose(Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY()));
    modelPose.translation() = Eigen::Vector3d(1, 2, 3);

    std::vector<Eigen::Vector3d> obsPoints3d;
    for (auto &pt : modelPoints3d)
    {
        obsPoints3d.push_back(modelPose * pt);
    }

    Simple3DViewer viewer;
    viewer.addCloud(eigenToCvPoints(modelPoints3d), cv::Scalar(0, 255, 0), 1);    // Green
    viewer.addCloud(eigenToCvPoints(obsPoints3d), cv::Scalar(255, 255, 0), 1); // Cyan
    viewer.show("3d-3d Registration Problem");

    Eigen::Matrix<double, 7, 1> pose_params;
    pose_params.head<3>().setZero();
    pose_params.tail<4>() = Eigen::Quaterniond::Identity().coeffs();

    ceres::Problem problem;
    for (size_t i = 0; i < modelPoints3d.size(); ++i)
    {
        ceres::CostFunction *cost_function;
        if (USE_AUTO_DIFF)
        {
            cost_function = new ceres::AutoDiffCostFunction<AutoDiffPointFactor, 3, 7>(
                new AutoDiffPointFactor(modelPoints3d[i], obsPoints3d[i]));
        }
        else
        {
            cost_function = new AnalyticPointFactor(modelPoints3d[i], obsPoints3d[i]);
        }
        problem.AddResidualBlock(cost_function, nullptr, pose_params.data());
    }

    ceres::Manifold *manifold;
    if (USE_AUTO_DIFF)
    {
        manifold = new ceres::ProductManifold(
            new ceres::EuclideanManifold<3>(),
            new ceres::EigenQuaternionManifold());
    }
    else
    {
        manifold = new PoseManifold();
    }
    problem.SetManifold(pose_params.data(), manifold);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    // 結果表示用
    Eigen::Vector3d t_opt = pose_params.head<3>();
    Eigen::Quaterniond q_opt(pose_params.tail<4>());
    Eigen::Isometry3d resultPose = Eigen::Isometry3d::Identity();
    resultPose.linear() = q_opt.toRotationMatrix();
    resultPose.translation() = t_opt;

    std::vector<Eigen::Vector3d> estPoints3d;
    for (auto &pt : modelPoints3d)
        estPoints3d.push_back(resultPose * pt);

    viewer.clear();
    viewer.addCloud(eigenToCvPoints(modelPoints3d), cv::Scalar(0, 255, 0), 1);    // Green
    viewer.addCloud(eigenToCvPoints(obsPoints3d), cv::Scalar(255, 255, 0), 1); // Cyan
    viewer.addCloud(eigenToCvPoints(estPoints3d), cv::Scalar(0, 0, 255), 1);   // Red
    viewer.show("3d-3d Registration Result");

    return 0;
}