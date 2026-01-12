
/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
 *******************************************************/

#include "../common/plot_opencv.hpp"
#include <ceres/ceres.h>
#include <iostream>
#include <random>
#include <vector>

// ceresのAutoDiffCostFunction用のFactor
struct CubicCostFunctor
{
    CubicCostFunctor(double x, double y)
        : m_x(x), m_y(y)
    {
    }
    template <typename T>
    bool operator()(const T *const x, T *residual) const
    {
        // e = (ax^3 + bx^2 + cx + d) - y
        residual[0] = static_cast<T>(x[0] * std::pow(m_x, 3) + x[1] * std::pow(m_x, 2) + x[2] * m_x + x[3]) - static_cast<T>(m_y);
        return true;
    }

private:
    const double m_x, m_y;
};

// ceresのSizedCostFunction用のFactor
///< 残渣の次元数:1, 推定パラメーターの次元数:4>
class CubicCostFunctionFactor : public ceres::SizedCostFunction<1, 4>
{
public:
    CubicCostFunctionFactor(double x, double y)
        : m_x(x), m_y(y)
    {
    }

    virtual bool Evaluate(
        double const *const *parameters,
        double *residuals,
        double **jacobians) const
    {
        double a = parameters[0][0];
        double b = parameters[0][1];
        double c = parameters[0][2];
        double d = parameters[0][3];

        double x2 = m_x * m_x;
        double x3 = m_x * x2;

        // e = (ax^3 + bx^2 + cx + d) - y
        residuals[0] = (a * x3 + b * x2 + c * m_x + d) - m_y;

        if (!jacobians)
        {
            return true;
        }
        // de/da
        jacobians[0][0] = x3;
        // de/db
        jacobians[0][1] = x2;
        // de/dc
        jacobians[0][2] = m_x;
        // de/dd
        jacobians[0][3] = 1;
        return true;
    }

private:
    const double m_x,
        m_y;
};

int main()
{
    /// 三次関数を生成
    double a = 0.5, b = -1.0, c = -0.5, d = 1.0;

    // 乱数発生器
    double mu = 0., sigma = 0.3;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    // データ生成
    double minX = -5.0, maxX = 5.0, resoX = 0.1;
    int numX = (maxX - minX) / resoX + 1;
    std::vector<double> v_X, v_gtY, v_obsY, v_estY;
    for (int i = 0; i < numX; i++)
    {
        double x = minX + resoX * i;
        double x2 = x * x;
        double x3 = x * x2;
        double gtY = a * x3 + b * x2 + c * x + d;
        double obsY = a * x3 + b * x2 + c * x + d + dist(engine);
        v_X.emplace_back(x);
        v_gtY.emplace_back(gtY);
        v_obsY.emplace_back(obsY);
    }

    ceres::Problem problem;
    std::vector<double> parameter(4, 0.0);
#if 1
    /// AutoDiff
    for (size_t i = 0; i < v_X.size(); i++)
    {
        double x = v_X[i];
        double obsY = v_obsY[i];
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CubicCostFunctor, 1, 4>(
            new CubicCostFunctor(x, obsY));
        problem.AddResidualBlock(cost_function, nullptr, parameter.data());
    }
#else
    /// SizedCostFunction
    // ceres::LossFunction* loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    for (size_t i = 0; i < v_X.size(); i++)
    {
        double x = v_X[i];
        double obsY = v_obsY[i];
        CubicCostFunctionFactor *f = new CubicCostFunctionFactor(x, obsY);
        problem.AddResidualBlock(f, nullptr, parameter.data());
    }
#endif

    // 最適化オプション
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // 最適化実行
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    for (int i = 0; i < numX; i++)
    {
        double x = minX + resoX * i;
        double x2 = x * x;
        double x3 = x * x2;
        double estY = parameter[0] * x3 + parameter[1] * x2 + parameter[2] * x + parameter[3];
        v_estY.emplace_back(estY);
    }

    // 表示用変数に格納
    std::vector<cv::Point2d> line_gt, line_est, scatter;
    for (int i = 0; i < numX; i++)
    {
        line_gt.emplace_back(v_X[i], v_gtY[i]);
        line_est.emplace_back(v_X[i], v_estY[i]);
        scatter.emplace_back(v_X[i], v_obsY[i]);
    }

    // グラフにして表示
    ocvplot::plot(
        {line_gt, line_est}, // 複数の線
        {scatter},           // 散布図
        minX, maxX,
        -5, 5 // Y 範囲
    );

    return 0;
}