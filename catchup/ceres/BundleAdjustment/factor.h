#include <Eigen/Dense>
#include <ceres/ceres.h>

class PoseLocalParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 3> {
public:
    ProjectionFactor(const Eigen::Vector2d& _pts_i);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
    Eigen::Vector2d pt_j;
    Eigen::Matrix2d infoMatrix;
};
