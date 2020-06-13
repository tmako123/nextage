/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include "../matplotlib-cpp/matplotlibcpp.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <random>
#include <vector>

#define PI 3.14159265358979

namespace plt = matplotlibcpp;

///<Number of observation parameter, Num of estimation parameter>
class Crosp2d2dFunctorFactor : public ceres::SizedCostFunction<2, 3> {
public:
    Crosp2d2dFunctorFactor(Eigen::Vector2d& modelPoints, Eigen::Vector2d& obsPoints)
        : m_modelPoints(modelPoints)
        , m_obsPoints(obsPoints)
    {
    }

public:
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const
    {
        Eigen::Isometry2d deltaPose;
        deltaPose.setIdentity();
        deltaPose.prerotate(parameters[0][2]);
        deltaPose.pretranslate(Eigen::Vector2d(parameters[0][0], parameters[0][1]));

        Eigen::Vector2d wModelPoints = deltaPose * m_modelPoints;
        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual = wModelPoints - m_obsPoints;

        if (!jacobians) {
            return true;
        }
        ///eigen default major is col! set row major.
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> j(jacobians[0]);
        j << 1.0, 0.0, -wModelPoints.y(),
            0.0, 1.0, wModelPoints.x(),
            0.0, 0.0, wModelPoints.x() * wModelPoints.x(), wModelPoints.y() * wModelPoints.y();
        return true;
    }

private:
    Eigen::Vector2d m_modelPoints, m_obsPoints;
};

void vectorEigen2Std(
    const std::vector<Eigen::Vector2d>& ePt,
    std::vector<double>& x, std::vector<double>& y)
{
    x.reserve(ePt.size());
    y.reserve(ePt.size());
    for (auto& pt : ePt) {
        x.push_back(pt.x());
        y.push_back(pt.y());
    }
}

int main()
{
    double maxX = 10;
    double maxY = 5;
    double step = 0.5;
    double mu = 0., sigma = 0.2;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    ///
    std::vector<Eigen::Vector2d> modelPoints;
    for (double i = 0; i <= maxX; i += step) {
        modelPoints.push_back(Eigen::Vector2d(i, 0));
        modelPoints.push_back(Eigen::Vector2d(i, maxY));
    }
    for (double i = 0; i <= maxY; i += step) {
        modelPoints.push_back(Eigen::Vector2d(0, i));
        modelPoints.push_back(Eigen::Vector2d(maxX, i));
    }

    ///
    std::vector<Eigen::Vector2d> obsPoints;
    Eigen::Isometry2d gtMatrix;
    gtMatrix.setIdentity();
    gtMatrix.prerotate(10 * PI / 100);
    gtMatrix.pretranslate(Eigen::Vector2d(0.5, 1.5));
    obsPoints.reserve(modelPoints.size());
    for (auto& pt : modelPoints) {
        Eigen::Vector2d obs = gtMatrix * pt + Eigen::Vector2d(dist(engine), dist(engine));
        obsPoints.push_back(obs);
    }

    ceres::Problem problem;
    double vecPose[3] = { 0, 0, 0 };

    ///SizedCostFunction
    ceres::LossFunction* loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    for (size_t i = 0; i < obsPoints.size(); i++) {
        Eigen::Vector2d obsPt = obsPoints[i];
        Eigen::Vector2d modelPt = modelPoints[i];
        Crosp2d2dFunctorFactor* f = new Crosp2d2dFunctorFactor(modelPt, obsPt);
        //problem.AddResidualBlock(f, nullptr, vecPose);
        problem.AddResidualBlock(f, loss_function, vecPose);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    Eigen::Isometry2d estMatrix;
    estMatrix.setIdentity();
    estMatrix.prerotate(vecPose[2]);
    estMatrix.pretranslate(Eigen::Vector2d(vecPose[0], vecPose[1]));

    std::vector<Eigen::Vector2d> estPoints;
    estPoints.reserve(modelPoints.size());
    for (auto& pt : modelPoints) {
        Eigen::Vector2d est = estMatrix * pt;
        estPoints.push_back(est);
    }

    std::vector<double> modelX, modelY, obsX, obsY, estX, estY;
    vectorEigen2Std(modelPoints, modelX, modelY);
    vectorEigen2Std(obsPoints, obsX, obsY);
    vectorEigen2Std(estPoints, estX, estY);
    plt::scatter(modelX, modelY);
    plt::scatter(obsX, obsY);
    plt::scatter(estX, estY);
    plt::show();

    return 0;
}