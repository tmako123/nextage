#include "projection_factor.h"
#include "utility.h"

Eigen::Matrix2d ProjectionFactor::sqrt_info;

ProjectionFactor::ProjectionFactor(const Eigen::Vector2d& _pts_j)
    : pts_j(_pts_j)
{
    sqrt_info = Eigen::Matrix2d::Identity() * 400;
};

bool ProjectionFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
	//std::cout << "pose" << Pj.transpose() << std::endl;

    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    //std::cout << "tic" << tic.transpose() << std::endl;

	Eigen::Vector3d pts_w(parameters[2][0], parameters[2][1], parameters[2][2]);
    //std::cout << "pts_w" << pts_w.transpose() << std::endl;


    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j;
    residual = sqrt_info * residual;
	//std::cout << "res" << residual << std::endl;

    if (jacobians) {
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
            0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        reduce = sqrt_info * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = -ric.transpose();
            Eigen::Matrix3d tmp_r = -ric.transpose();
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_imu_j);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_points(jacobians[2]);
            Eigen::Matrix<double, 3, 3> jaco_j;
            jaco_j = ric.transpose() * Rj.transpose();
            jacobian_points = reduce * jaco_j;
        }
    }

    return true;
}