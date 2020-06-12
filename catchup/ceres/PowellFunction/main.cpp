/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <random>
#include <vector>

struct PowellCostFunctor {
    template <typename T>
    bool operator()(
        const T* const x,
        T* residual) const
    {
        residual[0] = x[0] + T(10.0) * x[1];
        residual[1] = T(sqrt(5.0)) * (x[2] - x[3]);
        residual[2] = (x[1] - T(2.0) * x[2]) * (x[1] - T(2.) * x[2]);
        residual[3] = T(sqrt(10.0)) * (x[0] - x[3]) * (x[0] - x[3]);
        return true;
    }
};

///<Number of observation parameter, Num of estimation parameter>
class PowellCostFunctorFactor : public ceres::SizedCostFunction<4, 4> {
public:
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const
    {
        double x1 = parameters[0][0];
        double x2 = parameters[0][1];
        double x3 = parameters[0][2];
        double x4 = parameters[0][3];

        residuals[0] = x1 + 10.0 * x2;
        residuals[1] = sqrt(5.0) * x3 - x4;
        residuals[2] = (x2 - 2.0 * x3) * (x2 - 2.0 * x3);
        residuals[3] = sqrt(10.0) * (x1 - x4) * (x1 - x4);

        if (!jacobians) {
            return true;
        }
        ///eigen default major is col! set row major.
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> j(jacobians[0]);
        j << 1.0, 10.0, 0.0, 0.0,
            0.0, 0.0, sqrt(5.0), -1.0,
            0.0, 2 * x2 - 4 * x3, -4 * x2 + 8 * x3, 0.0,
            2 * sqrt(10.0) * (x1 - x4), 0.0, 0.0, -2 * sqrt(10.0) * (x1 - x4);
        return true;
    }
};

int main()
{
    ///問題のセットアップ
    Eigen::Vector4d x(3.0, 1.0, 2.0, 1.0);
    std::cout << "initial " << x.transpose() << std::endl;

    ceres::Problem problem;
#if 0
    ///AutoDiff
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PowellCostFunctor, 4, 4>(
        new PowellCostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, x.data());
#else
    ///SizedCostFunction
    //ceres::LossFunction* loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    PowellCostFunctorFactor* f = new PowellCostFunctorFactor();
    //problem.AddResidualBlock(f, loss_function, parameter.data());
    problem.AddResidualBlock(f, nullptr, x.data());
#endif

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    std::cout << "optimized " << x.transpose() << std::endl;

    return 0;
}