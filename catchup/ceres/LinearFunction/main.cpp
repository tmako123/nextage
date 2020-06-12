
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

struct LinearCostFunctor {
    LinearCostFunctor(double x, double y)
        : m_x(x)
        , m_y(y)
    {
    }
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = static_cast<T>(m_y) - static_cast<T>(x[0] * m_x + x[1]);
        return true;
    }

private:
    const double m_x, m_y;
};

///<Number of observation parameter, Num of estimation parameter>
class LinearCostFunctionFactor : public ceres::SizedCostFunction<1, 2> {
public:
    LinearCostFunctionFactor(double x, double y)
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

        residuals[0] = (a * m_x + b) - m_y;

        if (!jacobians) {
            return true;
        }
        jacobians[0][0] = m_x;
        jacobians[0][1] = 1;
        return true;
    }

private:
    const double m_x,
        m_y;
};

int main()
{
    ///問題のセットアップ
    ///y = 0.5 x + 1
    double a = 0.5, b = 1.0;
    double mu = 0., sigma = 0.3;

    std::vector<double> vecX, vecY, vecY_, vecY__;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    double minX = -5.0, maxX = 5.0, resoX = 0.1;
    int numX = (maxX - minX) / resoX + 1;

    for (int i = 0; i < numX; i++) {
        double x = minX + resoX * i;
        double y = a * x + b;
        double y_ = a * x + b + dist(engine);
        vecX.push_back(x);
        vecY.push_back(y);
        vecY_.push_back(y_);
    }

    ceres::Problem problem;
    std::vector<double> parameter(2, 0.0);
#if 0
	///AutoDiff
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<LinearCostFunctor, 1, 2>(
            new LinearCostFunctor(x, y_));
        problem.AddResidualBlock(cost_function, nullptr, parameter.data());
    }
#else
    ///SizedCostFunction
    //ceres::LossFunction* loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        LinearCostFunctionFactor* f = new LinearCostFunctionFactor(x, y_);
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
        double y__ = parameter[0] * x + parameter[1];
        vecY__.push_back(y__);
    }

    plt::plot(vecX, vecY);
    plt::scatter(vecX, vecY_);
    plt::plot(vecX, vecY__);
    plt::show();

    return 0;
}