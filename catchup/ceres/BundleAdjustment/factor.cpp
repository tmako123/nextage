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
    // 1. パラメータ
    Eigen::Map<const Eigen::Vector3d> P(parameters[0]);
    // Ceres/Eigenの四元数は w, x, y, z
    double w = parameters[0][3];
    double x = parameters[0][4];
    double y = parameters[0][5];
    double z = parameters[0][6];
    Eigen::Quaterniond Q(w, x, y, z);
    Eigen::Map<const Eigen::Vector3d> X(parameters[1]);

    // 2. 座標変換
    // p = R(q)^T * (X - P)
    // R(q)^T は q の共役 q* = (w, -x, -y, -z) による回転と同じです。
    // つまり、p = q* \times (X-P) \times q
    Eigen::Matrix3d R_cw = Q.toRotationMatrix().transpose();
    Eigen::Vector3d X_rel = X - P;
    Eigen::Vector3d pt_cam = R_cw * X_rel;

    // 3. 残差
    double weight = 400.0;
    double inv_z = 1.0 / pt_cam.z();
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = (pt_cam.head<2>() * inv_z - pt_j) * weight;

    if (jacobians)
    {
        // 4. 投影行列 (2x3)
        Eigen::Matrix<double, 2, 3> J_proj;
        double inv_z2 = inv_z * inv_z;
        J_proj << inv_z, 0, -pt_cam.x() * inv_z2,
                  0, inv_z, -pt_cam.y() * inv_z2;
        J_proj *= weight;

        // --- カメラパラメータ (jacobians[0]) ---
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_cam(jacobians[0]);
            J_cam.setZero();

            // A. 並進微分 (∂p / ∂P = -R_cw)
            J_cam.leftCols<3>() = J_proj * (-R_cw);

            // B. 回転微分 (∂p / ∂q) : Ambient Jacobian (3x4)を直接計算
            // 式: p = R(q)^T * u  (u = X_rel)
            // この偏微分は非常に間違いやすいため、以下の成分計算を用います。
            // R^T * u = (2w^2 - 1)u + 2(v*u)v - 2w(v x u)  (v=[x,y,z])
            // 注: 通常の回転 R*u の式に対し、クロス積の項の符号が反転します。

            double ux = X_rel.x(), uy = X_rel.y(), uz = X_rel.z();
            
            // 各成分の偏微分係数
            // Col 0: w
            // ∂/∂w = 4w*u - 2(v x u) = 2 * (2w*u - v x u) -> いや、係数が合わないことが多いので行列展開します
            
            Eigen::Matrix<double, 3, 4> J_rot_amb;

            // 以下の係数は R^T * u の厳密な展開結果です
            // w
            J_rot_amb(0, 0) = 2.0 * ( w * ux + z * uy - y * uz);
            J_rot_amb(1, 0) = 2.0 * (-z * ux + w * uy + x * uz);
            J_rot_amb(2, 0) = 2.0 * ( y * ux - x * uy + w * uz);

            // x
            J_rot_amb(0, 1) = 2.0 * ( x * ux + y * uy + z * uz);
            J_rot_amb(1, 1) = 2.0 * ( y * ux - x * uy - w * uz);
            J_rot_amb(2, 1) = 2.0 * ( z * ux + w * uy - x * uz);

            // y
            J_rot_amb(0, 2) = 2.0 * (-y * ux + x * uy + w * uz);
            J_rot_amb(1, 2) = 2.0 * ( x * ux + y * uy + z * uz);
            J_rot_amb(2, 2) = 2.0 * (-w * ux + z * uy - y * uz);

            // z
            J_rot_amb(0, 3) = 2.0 * (-z * ux - w * uy + y * uz);
            J_rot_amb(1, 3) = 2.0 * ( w * ux - z * uy + x * uz);
            J_rot_amb(2, 3) = 2.0 * ( x * ux + y * uy + z * uz);

            J_cam.rightCols<4>() = J_proj * J_rot_amb;
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt(jacobians[1]);
            J_pt = J_proj * R_cw;
        }
    }
    return true;
}