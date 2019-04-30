#pragma once

#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>

namespace gtsam {
	class ISAM2;
	class NonlinearFactorGraph;
	class Values;
	class PreintegratedImuMeasurements;
	class NavState;
	namespace imuBias {
		class ConstantBias;
	}
}

class ImuPreintegration
{
public:
	ImuPreintegration();
	~ImuPreintegration();

	void initialize(double time, Eigen::Isometry3d& initialPose, Eigen::Vector3d& initialVelocity,
		double imuBiasNoise, double jyroBiasNoise);

	Eigen::Isometry3d updateImu(double time, Eigen::Vector3d& gyro, Eigen::Vector3d& acc, bool bDebug = false);

protected:
	gtsam::ISAM2* m_isam;
	gtsam::NonlinearFactorGraph* m_graph;
	gtsam::Values* m_initialEstimate;
	gtsam::PreintegratedImuMeasurements* m_preintImu;
	gtsam::NavState* m_prevState;
	gtsam::imuBias::ConstantBias* m_prevBias;

	double m_latestImuTime;
	
	size_t m_countV;
	size_t m_countX;
	size_t m_countB;
};

