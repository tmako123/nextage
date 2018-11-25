#include <opencv2/opencv.hpp>
#include <opencv_lib.hpp>
#include <opencv2/viz.hpp>
#include <iostream>

#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/core/eigen.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"

int main()
{

	std::vector<cv::Vec3f> pos;
	std::vector<cv::Vec3b> color;

	//posegeneration
	int num_pose = 20;
	int radius = 10;
	std::vector<Eigen::Isometry3d> gtPose; //w2c
	for (int i = 0; i < num_pose; i++) {
		double rad = 2 * M_PI / num_pose * i;
		Eigen::AngleAxisd rot(rad, Eigen::Vector3d(0, 1, 0));
		Eigen::Vector3d trans(sin(rad), 0, cos(rad));
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(rot);
		pose.pretranslate(trans * radius);
		gtPose.push_back(pose.inverse());
	}

	std::vector<Eigen::Isometry3d> obsPose; //w2c
	for (int i = 0; i < num_pose; i++) {
		double rad = 2 * M_PI / num_pose * i * 0.95;
		Eigen::AngleAxisd rot(rad, Eigen::Vector3d(0, 1, 0));
		Eigen::Vector3d trans(sin(rad), 0, cos(rad));
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(rot);
		pose.pretranslate(trans * radius);
		obsPose.push_back(pose.inverse());
	}

	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		for (int i = 0; i < num_pose; i++) {
			cv::Affine3d camPose;
			cv::eigen2cv(Eigen::Affine3d(gtPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetFrustumName = "CPW_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(255, 255, 255)); // Camera frustum
			myWindow.showWidget(widgetFrustumName, cpw_frustum, camPose);

			cv::eigen2cv(Eigen::Affine3d(obsPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetEstFrustumName = "CPE_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpe_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(0, 255, 0)); // Camera frustum
			myWindow.showWidget(widgetEstFrustumName, cpe_frustum, camPose);
		}
		myWindow.spinOnce(1, true);
		myWindow.spin();

		//•\Ž¦‚ðíœ
		myWindow.removeAllWidgets();
	}

	//make edge
	struct Edge {
		Eigen::Isometry3d relativePose;
		std::pair<size_t, size_t> id;
	};

	std::vector<Edge> edges;
	for (int i = 0; i < num_pose - 1; i++) {
		Edge edge;
		edge.relativePose = obsPose[i + 1] * obsPose[i].inverse();
		edge.id = std::pair<size_t, size_t>(i, i + 1);
		edges.push_back(edge);
	}
	for (int i = 0; i < num_pose - 2; i++) {
		Edge edge;
		edge.relativePose = gtPose[i + 2] * gtPose[i].inverse();
		edge.id = std::pair<size_t, size_t>(i, i + 2);
		edges.push_back(edge);
	}

	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		for (int i = 0; i < num_pose; i++) {
			cv::Affine3d camPose;
			//cv::eigen2cv(Eigen::Affine3d(gtPose[i]).matrix(), camPose.matrix);
			//std::string widgetFrustumName = "CPW_FRUSTUM" + std::to_string(i);
			//cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(255, 255, 255)); // Camera frustum
			//myWindow.showWidget(widgetFrustumName, cpw_frustum, camPose);

			cv::eigen2cv(Eigen::Affine3d(obsPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetEstFrustumName = "CPE_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpe_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(0, 255, 0)); // Camera frustum
			myWindow.showWidget(widgetEstFrustumName, cpe_frustum, camPose);
		}

		for (int i = 0; i < edges.size(); i++) {
			Edge edge = edges[i];
			Eigen::Vector3d first = obsPose[edge.id.first].inverse().translation();
			Eigen::Vector3d second = obsPose[edge.id.second].inverse().translation();
			cv::Point3d cvFirst(first.x(), first.y(), first.z());
			cv::Point3d cvSecond(second.x(), second.y(), second.z());
			cv::viz::WLine cvEdge(cvFirst, cvSecond);
			std::string widgetEdgeName = "EDGES" + std::to_string(i);
			myWindow.showWidget(widgetEdgeName, cvEdge);
		}

		myWindow.spinOnce(1, true);
		myWindow.spin();

		//•\Ž¦‚ðíœ
		myWindow.removeAllWidgets();
	}

	//g2o
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
		new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
	g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(num_pose);
	//std::vector<g2o::VertexSim3Expmap*> vpVertices(num_pose);

	// Set KeyFrame vertices
	for (size_t i = 0, iend = num_pose; i < iend; i++) {
		g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

		Eigen::Matrix<double, 3, 3> Rcw = obsPose[i].rotation();
		Eigen::Matrix<double, 3, 1> tcw = obsPose[i].translation();
		g2o::Sim3 Scw(Rcw, tcw, 1.0);
		//vScw[i] = Siw;
		VSim3->setEstimate(Scw);

		if (i == 0) VSim3->setFixed(true);

		VSim3->setId(i);
		VSim3->setMarginalized(false);
		VSim3->_fix_scale = true;
		optimizer.addVertex(VSim3);
		//vpVertices[i] = VSim3;
	}

	std::set<std::pair<long unsigned int, long unsigned int> > sInsertedEdges;
	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	for (int i = 0; i < edges.size(); i++) {
		Edge edge = edges[i];
		Eigen::Matrix<double, 3, 3> Rji = edge.relativePose.rotation();
		Eigen::Matrix<double, 3, 1> tji = edge.relativePose.translation();
		const g2o::Sim3 Sji(Rji, tji, 1.0);

		g2o::EdgeSim3* e = new g2o::EdgeSim3();
		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edge.id.first)));
		e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edge.id.second)));
		e->setMeasurement(Sji);
		e->information() = matLambda;
		optimizer.addEdge(e);
	}

	{
		Eigen::Isometry3d loopPose;
		Eigen::Isometry3d relPose = gtPose[num_pose - 1] * gtPose[0].inverse();
		Eigen::Matrix<double, 3, 3> Rji = relPose.rotation();
		Eigen::Matrix<double, 3, 1> tji = relPose.translation();
		const g2o::Sim3 Sji(Rji, tji, 1.0);

		g2o::EdgeSim3* e = new g2o::EdgeSim3();
		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(num_pose - 1)));
		e->setMeasurement(Sji);
		e->information() = matLambda;
		optimizer.addEdge(e);
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(20);

	std::vector<Eigen::Isometry3d> estPose; //w2c

	for (int i = 0; i < num_pose; i++) {
		g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(i));
		g2o::Sim3 CorrectedSiw = VSim3->estimate();
		double s = CorrectedSiw.scale();

		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(CorrectedSiw.rotation().toRotationMatrix());
		pose.pretranslate(CorrectedSiw.translation() * (1. / s));
		estPose.push_back(pose);
	}

	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		for (int i = 0; i < num_pose; i++) {
			cv::Affine3d camPose;
			cv::eigen2cv(Eigen::Affine3d(gtPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetFrustumName = "CPW_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(255, 255, 255)); // Camera frustum
			myWindow.showWidget(widgetFrustumName, cpw_frustum, camPose);

			cv::eigen2cv(Eigen::Affine3d(obsPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetObsFrustumName = "CPO_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpo_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(0, 255, 0)); // Camera frustum
			myWindow.showWidget(widgetObsFrustumName, cpo_frustum, camPose);

			cv::eigen2cv(Eigen::Affine3d(estPose[i].inverse()).matrix(), camPose.matrix);
			std::string widgetEstFrustumName = "CPE_FRUSTUM" + std::to_string(i);
			cv::viz::WCameraPosition cpe_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, cv::viz::Color(0, 0, 255)); // Camera frustum
			myWindow.showWidget(widgetEstFrustumName, cpe_frustum, camPose);
		}
		myWindow.spinOnce(1, true);
		myWindow.spin();

		//•\Ž¦‚ðíœ
		myWindow.removeAllWidgets();

	}

	return 0;
}