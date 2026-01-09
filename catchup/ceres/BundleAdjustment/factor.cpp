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
    Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q(parameters[0][3], parameters[0][4], parameters[0][5], parameters[0][6]);

    Eigen::Vector3d X(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d pt_cam = Q.inverse() * (X - P);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    double inv_z = 1.0 / pt_cam.z();
    double inv_z2 = inv_z * inv_z;
    residual = (pt_cam * inv_z).head<2>() - pt_j;
    residual = infoMatrix * residual;

    if (jacobians)
    {
        Eigen::Matrix3d R = Q.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> jacob_proj(2, 3);
        jacob_proj << 1. * inv_z, 0, -pt_cam(0) * inv_z2,
            0, 1. * inv_z, -pt_cam(1) * inv_z2;
        jacob_proj = infoMatrix * jacob_proj;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_cam(jacobians[0]);
            jacobian_cam.setZero(); // 7列すべて初期化

            // 1. 投影の微分 (2x3)
            Eigen::Matrix<double, 2, 3> jacob_proj;
            jacob_proj << inv_z, 0, -pt_cam.x() * inv_z2,
                0, inv_z, -pt_cam.y() * inv_z2;
            jacob_proj = infoMatrix * jacob_proj;

            // 2. 並進の微分 (2x3)
            jacobian_cam.leftCols<3>() = jacob_proj * (-R.transpose());

            // 3. 回転の微分 (2x4)
            // 接空間の微分 (2x3) = jacob_proj * skewSymmetric(pt_cam)
            // これを Ceres の QuaternionManifold が解釈できる 2x4 形式に変換します。
            // [接空間微分(2x3)] * [Local-to-Global 変換行列(3x4)] という形になります。

            // 回転の接空間微分 (2x3)
            Eigen::Matrix<double, 2, 3> jacob_rot_local = jacob_proj * skewSymmetric(pt_cam);

            // CeresのQuaternionManifold(w,x,y,z)において、
            // Local増分からGlobal(Quaternion)への微分は以下の形になります。
            Eigen::Matrix<double, 3, 4> j_local_to_global;
            j_local_to_global << -Q.x(), Q.w(), Q.z(), -Q.y(),
                -Q.y(), -Q.z(), Q.w(), Q.x(),
                -Q.z(), Q.y(), -Q.x(), Q.w();
            j_local_to_global *= 0.5;

            // 4列（index 3, 4, 5, 6）にセット
            // ここでは単純化のため、Ceresの内部射影と打ち消し合う形での「Global微分」をセットします
            // ※ 実際には Ceres がこの jacobian_cam (2x4) に Manifold の 4x3 を掛けて 2x3 に戻します。
            // そのため、ここでは 2x3 の local 微分を 2x4 に「展開」して渡します。

            // 最も確実な「手書きLocal微分」の渡し方：
            // CeresのQuaternionManifoldを使っている場合、
            // ここで 2x4 の行列として「接空間での動きをQuaternionの4成分に投影した値」を書き込みます。
            jacobian_cam.block<2, 4>(0, 3) = jacob_rot_local * j_local_to_global;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
            Eigen::Matrix3d j_point;
            j_point = R.transpose();
            jacobian_point = jacob_proj * j_point;
        }
    }

    return true;
}