#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

//class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> 
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 3> {
  public:
    ProjectionFactor(const Eigen::Vector2d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    //void check(double **parameters);

    Eigen::Vector2d pts_j;
    static Eigen::Matrix2d sqrt_info;
};
