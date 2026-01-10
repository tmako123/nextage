#include <Eigen/Dense>
#include <ceres/ceres.h>

class PoseManifold : public ceres::Manifold
{
public:
    // メモリ上の次元数 (Pos:3 + Quat:4)
    int AmbientSize() const override { return 7; }

    // 実際の自由度 (Pos:3 + Rot:3)
    int TangentSize() const override { return 6; }

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override
    {
        Eigen::Map<const Eigen::Vector3d> p(x);
        // メモリ上の並びを[x, y, z, w]と仮定（Eigenのデフォルト）
        Eigen::Map<const Eigen::Quaterniond> q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);
        Eigen::Map<const Eigen::Vector3d> d_theta(delta + 3);

        // 微小回転 dq
        Eigen::Quaterniond dq = Eigen::Quaterniond(1.0, 0.5 * d_theta.x(), 0.5 * d_theta.y(), 0.5 * d_theta.z());

        Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

        p_plus = p + dp;
        q_plus = (q * dq).normalized(); // 右から掛ける＝カメラ座標系での回転
        return true;
    }

    // 2. PlusJacobian (∂(x ⊕ delta) / ∂delta)
    // ここがVINSの「パススルー」のキモ
    bool PlusJacobian(const double *x, double *jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
        J.setZero();

        // Evaluateで書いた最初の6列をそのまま抽出させるために単位行列を置く
        J.topRows<6>().setIdentity();

        return true;
    }

    // Minusはオプション（多くのソルバーでは実装不要でfalseを返せば良い）
    bool Minus(const double *y, const double *x, double *y_minus_x) const override { return false; }
    bool MinusJacobian(const double *x, double *jacobian) const override { return false; }
};

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    ProjectionFactor(const Eigen::Vector2d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    Eigen::Vector2d pt_j;
    Eigen::Matrix2d infoMatrix;
};

struct autoDiffProjectionFactor
{
    autoDiffProjectionFactor(const Eigen::Vector2d &_obs) : obs(_obs) {}

    template <typename T>
    bool operator()(const T *const pose, const T *const point, T *residuals) const
    {
        // 1. パラメータの展開
        // pose[0-2]: Translation, pose[3-6]: Quaternion (qw, qx, qy, qz)
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> P(pose);
        Eigen::Quaternion<T> Q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> X(point);

        // 2. カメラ座標系への変換: pt_cam = Q^-1 * (X - P)
        // AutoDiffなら inverse() の微分も自動で計算されます
        Eigen::Matrix<T, 3, 1> pt_cam = Q.inverse() * (X - P);

        // 3. 投影計算 (正規化座標系)
        T inv_z = T(1.0) / pt_cam.z();
        T predicted_u = pt_cam.x() * inv_z;
        T predicted_v = pt_cam.y() * inv_z;

        // 4. 残差の計算
        residuals[0] = predicted_u - T(obs.x());
        residuals[1] = predicted_v - T(obs.y());

        // 5. 重み付け (必要に応じて情報行列やスカラーを掛ける)
        T weight = T(20.0); // infoMatrix(400)のルートに相当
        residuals[0] *= weight;
        residuals[1] *= weight;

        return true;
    }

    Eigen::Vector2d obs;
};