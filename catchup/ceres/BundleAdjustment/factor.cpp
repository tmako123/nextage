#include "factor.h"

static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d R;
    R << 0, -w.z(), w.y(),
        w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0;
    return R;
}

ProjectionFactor::ProjectionFactor(const Eigen::Vector2d &_pt_j)
    : pt_j(_pt_j)
{
    infoMatrix = Eigen::Matrix2d::Identity() * 400;
};

bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // 1. パラメータ展開 (Eigen Mapを使用)
    Eigen::Map<const Eigen::Vector3d> P(parameters[0]);
    // EigenのMapでQを取り出す。メモリ並びが [x,y,z,w] の場合
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0] + 3);
    Eigen::Map<const Eigen::Vector3d> X(parameters[1]);

    // 2. 座標変換 (Rは Camera to World と仮定)
    Eigen::Matrix3d R = Q.toRotationMatrix();
    // pt_cam = R^T * (X - P)
    Eigen::Vector3d pt_cam = R.transpose() * (X - P);

    // 3. 残差計算
    double inv_z = 1.0 / pt_cam.z();
    Eigen::Vector2d projection(pt_cam.x() * inv_z, pt_cam.y() * inv_z);

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = infoMatrix * (projection - pt_j);

    if (jacobians)
    {
        // 投影微分 (2x3)
        Eigen::Matrix<double, 2, 3> jacob_proj;
        double inv_z2 = inv_z * inv_z;
        jacob_proj << inv_z, 0, -pt_cam.x() * inv_z2,
            0, inv_z, -pt_cam.y() * inv_z2;
        jacob_proj = infoMatrix * jacob_proj;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_cam(jacobians[0]);
            jacobian_cam.setZero();

            // --- 並進微分 ---
            // ∂(R^T(X-P))/∂P = -R^T
            jacobian_cam.leftCols<3>() = jacob_proj * (-R.transpose());

            // --- 回転微分 (ここが重要) ---
            // Manifoldで q * dq (右掛け) をしている場合、
            // 座標変換 pt_cam = R^T * (X - P) に対する微分は [pt_cam]x になる
            // ※もし左掛け (dq * q) なら -R^T * [X-P]x になる
            jacobian_cam.block<2, 3>(0, 3) = jacob_proj * skewSymmetric(pt_cam);
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
            // ∂(R^T(X-P))/∂X = R^T
            jacobian_point = jacob_proj * R.transpose();
        }
    }
    return true;
}