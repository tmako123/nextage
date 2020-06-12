
/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include "../matplotlib-cpp/matplotlibcpp.h"
#include <ceres/ceres.h>
#include <iostream>
#include <random>
#include <vector>

namespace plt = matplotlibcpp;

struct CubicCostFunctor {
    CubicCostFunctor(double x, double y)
        : m_x(x)
        , m_y(y)
    {
    }
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = static_cast<T>(m_y) - static_cast<T>(x[0] * std::pow(m_x, 3) + x[1] * std::pow(m_x, 2) + x[2] * m_x + x[3]);
        return true;
    }

private:
    const double m_x, m_y;
};

///<Number of observation parameter, Num of estimation parameter>
class CubicCostFunctionFactor : public ceres::SizedCostFunction<1, 4> {
public:
    CubicCostFunctionFactor(double x, double y)
        : m_x(x)
        , m_y(y)
    {
    }

    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const
    {
        double a = parameters[0][0];
        double b = parameters[0][1];
        double c = parameters[0][2];
        double d = parameters[0][3];

        double x2 = m_x * m_x;
        double x3 = m_x * x2;

        residuals[0] = (a * x3 + b * x2 + c * m_x + d) - m_y;

        if (!jacobians) {
            return true;
        }
        jacobians[0][0] = x3;
        jacobians[0][1] = x2;
        jacobians[0][2] = m_x;
        jacobians[0][3] = 1;
        return true;
    }

private:
    const double m_x,
        m_y;
};

int main()
{
    ///問題のセットアップ
    double a = 0.5, b = -1.0, c = -0.5, d = 1.0;
    double mu = 0., sigma = 0.3;

    std::vector<double> vecX, vecY, vecY_, vecY__;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    double minX = -5.0, maxX = 5.0, resoX = 0.1;
    int numX = (maxX - minX) / resoX + 1;

    for (int i = 0; i < numX; i++) {
        double x = minX + resoX * i;
        double x2 = x * x;
        double x3 = x * x2;
        double y = a * x3 + b * x2 + c * x + d;
        double y_ = a * x3 + b * x2 + c * x + d + dist(engine);
        vecX.push_back(x);
        vecY.push_back(y);
        vecY_.push_back(y_);
    }

    ceres::Problem problem;
    std::vector<double> parameter(4, 0.0);
#if 0
    ///AutoDiff
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CubicCostFunctor, 1, 4>(
            new CubicCostFunctor(x, y_));
        problem.AddResidualBlock(cost_function, nullptr, parameter.data());
    }
#else
    ///SizedCostFunction
    //ceres::LossFunction* loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        CubicCostFunctionFactor* f = new CubicCostFunctionFactor(x, y_);
        //problem.AddResidualBlock(f, loss_function, parameter.data());
        problem.AddResidualBlock(f, nullptr, parameter.data());
    }
#endif

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    for (int i = 0; i < numX; i++) {
        double x = minX + resoX * i;
        double x2 = x * x;
        double x3 = x * x2;
        double y__ = parameter[0] * x3 + parameter[1] * x2 + parameter[2] * x + parameter[3];
        vecY__.push_back(y__);
    }

    plt::plot(vecX, vecY);
    plt::scatter(vecX, vecY_);
    plt::plot(vecX, vecY__);
    plt::show();

    return 0;
}