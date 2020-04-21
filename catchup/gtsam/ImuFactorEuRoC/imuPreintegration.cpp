
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "imuPreintegration.h"
#include <stdio.h>

// Shorthand for velocity and pose variables
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;

const double kGravity = 9.81;

ImuPreintegration::ImuPreintegration():
	m_isam(NULL), m_graph(NULL), m_initialEstimate(NULL),
	m_countV(0), m_countX(0), m_countB(0)
{}

ImuPreintegration::~ImuPreintegration()
{
	if (m_isam != NULL) {
		delete m_isam;
	}
	if (m_graph != NULL) {
		delete m_graph;
	}
	if (m_initialEstimate != NULL) {
		delete m_initialEstimate;
	}
}

void ImuPreintegration::initialize(double time, Eigen::Isometry3d& initialPose, Eigen::Vector3d& initialVelocity,
	double imuBiasNoise, double jyroBiasNoise) {

	m_isam = new gtsam::ISAM2();
	m_initialEstimate = new gtsam::Values();
	m_graph = new gtsam::NonlinearFactorGraph();

	///////////////////
	initialPose.Identity();
	///////////////////

	gtsam::Pose3 gtsamInitialPose(initialPose.matrix());
	m_initialEstimate->insert(X(0), gtsamInitialPose);
	auto noise = gtsam::noiseModel::Diagonal::Sigmas(
		(gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
	m_graph->push_back(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsamInitialPose, noise));

	// Add imu bias
	auto biasnoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.1));
	m_initialEstimate->insert(B(0), gtsam::imuBias::ConstantBias());
	gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasprior(B(0), gtsam::imuBias::ConstantBias(),
		biasnoise);
	m_graph->push_back(biasprior);
	
	auto velnoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
	//gtsam::Vector3 n_velocity(initialVelocity);
	
	///////////////////
	gtsam::Vector3 n_velocity(0, 0, 0);
	///////////////////

	m_initialEstimate->insert(V(0), n_velocity);
	gtsam::PriorFactor<gtsam::Vector3> velprior(V(0), n_velocity, velnoise);
	m_graph->push_back(velprior);

	// genarate preintegration measurements
	auto params = gtsam::PreintegrationParams::MakeSharedU(kGravity);
	params->setAccelerometerCovariance(gtsam::I_3x3 * 0.1);
	params->setGyroscopeCovariance(gtsam::I_3x3 * 0.1);
	params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);
	params->setUse2ndOrderCoriolis(false);
	params->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
	m_preintImu = new gtsam::PreintegratedImuMeasurements(params);

	m_prevState = new gtsam::NavState(gtsamInitialPose, n_velocity);
	m_prevBias = new gtsam::imuBias::ConstantBias();
	m_latestImuTime = time;
}

Eigen::Isometry3d ImuPreintegration::updateImu(double time, Eigen::Vector3d& gyro, Eigen::Vector3d& acc, bool bDebug) {
	m_countX++;
	m_countV++;
	m_countB++;

	double deltaT = time - m_latestImuTime;

	///////////////////
	gyro = Eigen::Vector3d(0, 0, 0);
	acc = Eigen::Vector3d(0, 0, 0);
	///////////////////

	if (bDebug) {
		printf("%.3lf, %.3lf, %.3lf\n", time, m_latestImuTime, deltaT);
		std::cout << "gyro: " << gyro.transpose() << std::endl;
		std::cout << "acc: " << acc.transpose() << std::endl;
	}
	//predict
	gtsam::NavState prop_state = m_preintImu->predict(*m_prevState, *m_prevBias);

	//prop_state.pose().print();
	//std::cout << prop_state.v() << std::endl;

	m_initialEstimate->insert(X(m_countX), prop_state.pose());
	m_initialEstimate->insert(V(m_countV), prop_state.v());
	//m_initialEstimate->insert(B(m_countB), *m_prevBias);

	// Predict acceleration and gyro measurements in (actual) body frame
	gtsam::Vector3 measuredAcc(acc);
	gtsam::Vector3 measuredOmega(gyro);
	m_preintImu->integrateMeasurement(measuredAcc, measuredOmega, deltaT);

	// Add Imu Factor
	gtsam::ImuFactor imufac(X(m_countX - 1), V(m_countV - 1), X(m_countX), V(m_countV), B(0), *m_preintImu);
	m_graph->add(imufac);

	if (bDebug) {
		m_initialEstimate->print();
		m_graph->print();
	}
	//std::cout << "update" << std::endl;
	// Incremental solution
	m_isam->update(*m_graph, *m_initialEstimate);

	auto result = m_isam->calculateEstimate();
	if (bDebug) {
		std::cout << "result" << std::endl;
		result.print();
	}

	//update time
	m_latestImuTime = time;

	// Overwrite the beginning of the preintegration for the next step.
	*m_prevState = gtsam::NavState(result.at<gtsam::Pose3>(X(m_countX)),
		result.at<gtsam::Vector3>(V(m_countV)));
	//*m_prevBias = result.at<gtsam::imuBias::ConstantBias>(B(m_countB));

	m_preintImu->resetIntegration();
	m_graph->resize(0);
	m_initialEstimate->clear();

	//result.at<gtsam::Pose3>(X(m_countX)).print();
	Eigen::Matrix3d mat = result.at<gtsam::Pose3>(X(m_countX)).rotation().matrix();
	Eigen::Vector3d pos = result.at<gtsam::Pose3>(X(m_countX)).translation();
	Eigen::Isometry3d emat;
	emat.pretranslate(pos);
	emat.prerotate(mat);

	std::cout << emat.matrix() << std::endl;

	return emat;

}