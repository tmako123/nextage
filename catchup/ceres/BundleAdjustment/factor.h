#include <Eigen/Dense>
#include <ceres/ceres.h>

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 3> {
public:
    ProjectionFactor(const Eigen::Vector2d& _pts_i);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
    Eigen::Vector2d pt_j;
    Eigen::Matrix2d infoMatrix;
};

struct autoDiffProjectionFactor {
    autoDiffProjectionFactor(const Eigen::Vector2d& _obs) : obs(_obs) {}

    template <typename T>
    bool operator()(const T* const pose, const T* const point, T* residuals) const {
        // 1. パラメータの展開
        // pose[0-2]: Translation, pose[3-6]: Quaternion (qw, qx, qy, qz)
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> P(pose);
        Eigen::Quaternion<T> Q(pose[3], pose[4], pose[5], pose[6]);
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