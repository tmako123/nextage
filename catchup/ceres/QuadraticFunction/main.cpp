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

struct QuadraticCostFunctor {
    QuadraticCostFunctor(double x, double y)
        : m_x(x)
        , m_y(y)
    {
    }
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = static_cast<T>(m_y) - static_cast<T>(x[0] * m_x * m_x + x[1] * m_x + x[2]);
        return true;
    }

private:
    const double m_x, m_y;
};

///<Number of observation parameter, Num of estimation parameter>
class QuadraticCostFunctionFactor : public ceres::SizedCostFunction<1, 3> {
public:
    QuadraticCostFunctionFactor(double x, double y)
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

        residuals[0] = (a * m_x * m_x + b * m_x + c) - m_y;

        if (!jacobians) {
            return true;
        }
        jacobians[0][0] = m_x * m_x;
        jacobians[0][1] = m_x;
        jacobians[0][2] = 1;
        return true;
    }

private:
    const double m_x, m_y;
};

int main()
{
    ///問題のセットアップ
    ///y = 0.5 x + 1
    double a = 0.5, b = 1.2, c = 1.0;
    double mu = 0., sigma = 0.3;

    std::vector<double> vecX, vecY, vecY_, vecY__;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    double minX = -5.0, maxX = 5.0, resoX = 0.1;
    int numX = (maxX - minX) / resoX + 1;

    for (int i = 0; i < numX; i++) {
        double x = minX + resoX * i;
        double y = a * x * x + b * x + c;
        double y_ = a * x * x + b * x + c + dist(engine);
        vecX.push_back(x);
        vecY.push_back(y);
        vecY_.push_back(y_);
    }

    ceres::Problem problem;
    std::vector<double> parameter(3, 0.0);
#if 0
	///AutoDiff
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<QuadraticCostFunctor, 1, 3>(
            new QuadraticCostFunctor(x, y_));
        problem.AddResidualBlock(cost_function, nullptr, parameter.data());
    }
#else
    ///SizedCostFunction
    //ceres::LossFunction* loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    for (size_t i = 0; i < vecX.size(); i++) {
        double x = vecX[i];
        double y_ = vecY_[i];
        QuadraticCostFunctionFactor* f = new QuadraticCostFunctionFactor(x, y_);
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
        double y__ = parameter[0] * x * x + parameter[1] * x + parameter[2];
        vecY__.push_back(y__);
    }

    plt::plot(vecX, vecY);
    plt::scatter(vecX, vecY_);
    plt::plot(vecX, vecY__);
    plt::show();

    return 0;
}