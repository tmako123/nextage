#include "factor.h"

static Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta)
{
    Eigen::Quaterniond dq;
    Eigen::Vector3d harfTheta = theta;
    harfTheta *= 0.5;
    dq.x() = harfTheta.x();
    dq.y() = harfTheta.y();
    dq.z() = harfTheta.z();
    dq.w() = 1.0;
    return dq;
}

static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d R;
    R << 0, -w.z(), w.y(),
        w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0;
    return R;
}

bool PoseLocalParameterization::Plus(const double* x_old, const double* delta, double* x_new) const
{
    Eigen::Map<const Eigen::Vector3d> p_old(x_old);
    Eigen::Map<const Eigen::Quaterniond> q_old(x_old + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p_new(x_new);
    Eigen::Map<Eigen::Quaterniond> q_new(x_new + 3);

    p_new = p_old + dp;
    q_new = (q_old * dq).normalized();

    return true;
}

bool PoseLocalParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

ProjectionFactor::ProjectionFactor(const Eigen::Vector2d& _pt_j)
    : pt_j(_pt_j)
{
    infoMatrix = Eigen::Matrix2d::Identity() * 400;
};

bool ProjectionFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d X(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d pt_cam = Q.inverse() * (X - P);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    double inv_z = 1.0 / pt_cam.z();
    double inv_z2 = inv_z * inv_z;
    residual = (pt_cam * inv_z).head<2>() - pt_j;
    residual = infoMatrix * residual;

    if (jacobians) {
        Eigen::Matrix3d R = Q.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> jacob_proj(2, 3);
        jacob_proj << 1. * inv_z, 0, -pt_cam(0) * inv_z2,
            0, 1. * inv_z, -pt_cam(1) * inv_z2;
        jacob_proj = infoMatrix * jacob_proj;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_cam(jacobians[0]);

            Eigen::Matrix<double, 3, 6> j_cam;
            j_cam.leftCols<3>() = -R.transpose();
            j_cam.rightCols<3>() = skewSymmetric(pt_cam);

            jacobian_cam.leftCols<6>() = jacob_proj * j_cam;
            jacobian_cam.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
            Eigen::Matrix3d j_point;
            j_point = R.transpose();
            jacobian_point = jacob_proj * j_point;
        }
    }

    return true;
}