/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
 *******************************************************/

#include "../common/plot_opencv.hpp"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <random>
#include <vector>

#define PI 3.14159265358979

// ceresのAutoDiffCostFunction用のFactor
struct Crosp2d2dAutoDiffFunctor
{
    Crosp2d2dAutoDiffFunctor(const Eigen::Vector2d &modelPoints,
                             const Eigen::Vector2d &obsPoints)
        : m_modelPoints(modelPoints), m_obsPoints(obsPoints)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        // pose = [tx, ty, theta]
        const T tx = pose[0];
        const T ty = pose[1];
        const T th = pose[2];

        T cos_th = ceres::cos(th);
        T sin_th = ceres::sin(th);

        T px = T(m_modelPoints.x());
        T py = T(m_modelPoints.y());

        // 回転 + 並進
        T wx = cos_th * px - sin_th * py + tx;
        T wy = sin_th * px + cos_th * py + ty;

        residuals[0] = wx - T(m_obsPoints.x());
        residuals[1] = wy - T(m_obsPoints.y());

        return true;
    }

    Eigen::Vector2d m_modelPoints;
    Eigen::Vector2d m_obsPoints;
};

// ceresのSizedCostFunction用のFactor
///< 残差の次元数:2, 推定パラメーターの次元数:3>
class Crosp2d2dFunctorFactor : public ceres::SizedCostFunction<2, 3>
{
public:
    Crosp2d2dFunctorFactor(const Eigen::Vector2d &modelPoints,
                           const Eigen::Vector2d &obsPoints)
        : m_modelPoints(modelPoints), m_obsPoints(obsPoints)
    {
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        double tx = parameters[0][0];
        double ty = parameters[0][1];
        double th = parameters[0][2];

        Eigen::Isometry2d deltaPose = Eigen::Isometry2d::Identity();
        Eigen::Rotation2D<double> rot(th);
        deltaPose.prerotate(rot);
        deltaPose.pretranslate(Eigen::Vector2d(tx, ty));

        Eigen::Vector2d w = deltaPose * m_modelPoints;

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual = w - m_obsPoints;

        if (!jacobians)
            return true;

        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[0]);

        double px = m_modelPoints.x();
        double py = m_modelPoints.y();

        double c = std::cos(th);
        double s = std::sin(th);

        // ∂r/∂tx, ∂r/∂ty
        J(0, 0) = 1.0;
        J(0, 1) = 0.0;
        J(1, 0) = 0.0;
        J(1, 1) = 1.0;

        // ∂r/∂θ（完全版）
        J(0, 2) = -s * px - c * py;
        J(1, 2) = c * px - s * py;

        return true;
    }

private:
    Eigen::Vector2d m_modelPoints, m_obsPoints;
};

// Eigen → std::vector 変換
void vectorEigen2Std(
    const std::vector<Eigen::Vector2d> &ePt,
    std::vector<double> &x, std::vector<double> &y)
{
    x.reserve(ePt.size());
    y.reserve(ePt.size());
    for (auto &pt : ePt)
    {
        x.emplace_back(pt.x());
        y.emplace_back(pt.y());
    }
}

int main()
{
    double maxX = 8;
    double maxY = 6;
    double step = 0.5;

    double mu = 0., sigma = 0.2;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    // モデル点（格子状）
    std::vector<Eigen::Vector2d> modelPoints;
    for (double i = -maxX; i <= maxX; i += step)
    {
        modelPoints.emplace_back(i, -maxY);
        modelPoints.emplace_back(i, maxY);
    }
    for (double i = -maxY; i <= maxY; i += step)
    {
        modelPoints.emplace_back(-maxX, i);
        modelPoints.emplace_back(maxX, i);
    }

    // 観測点生成（真の姿勢 + ノイズ）
    std::vector<Eigen::Vector2d> obsPoints;
    Eigen::Isometry2d gtMatrix = Eigen::Isometry2d::Identity();
    gtMatrix.prerotate(Eigen::Rotation2D<double>(10 * PI / 100));
    gtMatrix.pretranslate(Eigen::Vector2d(0.5, 1.5));

    obsPoints.reserve(modelPoints.size());
    for (auto &pt : modelPoints)
    {
        Eigen::Vector2d obs = gtMatrix * pt + Eigen::Vector2d(dist(engine), dist(engine));
        obsPoints.emplace_back(obs);
    }

    // Ceres 問題設定
    ceres::Problem problem;
    double vecPose[3] = {0, 0, 0};

    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    for (size_t i = 0; i < obsPoints.size(); i++)
    {
        const Eigen::Vector2d &obsPt = obsPoints[i];
        const Eigen::Vector2d &modelPt = modelPoints[i];

#if 1
        // AutoDiff
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<Crosp2d2dAutoDiffFunctor, 2, 3>(
                new Crosp2d2dAutoDiffFunctor(modelPt, obsPt));
        problem.AddResidualBlock(cost_function, loss_function, vecPose);
#else
        // SizedCostFunction
        Crosp2d2dFunctorFactor *f = new Crosp2d2dFunctorFactor(modelPt, obsPt);
        problem.AddResidualBlock(f, loss_function, vecPose);
#endif
    }

    // 最適化オプション
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // 最適化実行
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // 推定結果の適用
    Eigen::Isometry2d estMatrix = Eigen::Isometry2d::Identity();
    estMatrix.prerotate(Eigen::Rotation2D<double>(vecPose[2]));
    estMatrix.pretranslate(Eigen::Vector2d(vecPose[0], vecPose[1]));

    // 表示用変数に格納
    std::vector<Eigen::Vector2d> estPoints;
    estPoints.reserve(modelPoints.size());
    for (auto &pt : modelPoints)
    {
        estPoints.emplace_back(estMatrix * pt);
    }

    // 可視化
    std::vector<double> modelX, modelY, obsX, obsY, estX, estY;
    vectorEigen2Std(modelPoints, modelX, modelY);
    vectorEigen2Std(obsPoints, obsX, obsY);
    vectorEigen2Std(estPoints, estX, estY);

    std::vector<cv::Point2d> scatter_gt, scatter_obs, scatter_est;
    for (int i = 0; i < (int)modelPoints.size(); i++)
    {
        scatter_gt.emplace_back(modelX[i], modelY[i]);
        scatter_obs.emplace_back(obsX[i], obsY[i]);
        scatter_est.emplace_back(estX[i], estY[i]);
    }

    ocvplot::plot(
        {},
        {scatter_gt, scatter_obs, scatter_est},
        -maxX * 2, maxX * 2,
        -maxY * 2, maxY * 2);

    return 0;
}