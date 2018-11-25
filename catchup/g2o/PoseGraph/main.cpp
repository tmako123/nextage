/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

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

//make edge
struct Edge {
	Eigen::Isometry3d relativePose;
	std::pair<size_t, size_t> id;
};

void vizPoses(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Isometry3d > poses/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255)) {
	for (int i = 0; i < poses.size(); i++) {
		cv::Affine3d camPose;
		cv::eigen2cv(Eigen::Affine3d(poses[i].inverse()).matrix(), camPose.matrix);
		std::string widgetFrustumName = name + std::to_string(i);
		cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -1.0, color); // Camera frustum
		window.showWidget(widgetFrustumName, cpw_frustum, camPose);
	}
}

void vizEdges(cv::viz::Viz3d& window, std::string& name, std::vector<Edge>& edges, std::vector<Eigen::Isometry3d > poses/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255)) {
	for (int i = 0; i < edges.size(); i++) {
		Edge edge = edges[i];
		Eigen::Vector3d first = poses[edge.id.first].inverse().translation();
		Eigen::Vector3d second = poses[edge.id.second].inverse().translation();
		cv::Point3d cvFirst(first.x(), first.y(), first.z());
		cv::Point3d cvSecond(second.x(), second.y(), second.z());
		cv::viz::WLine cvEdge(cvFirst, cvSecond, color);
		std::string widgetEdgeName = "EDGES" + std::to_string(i);
		window.showWidget(widgetEdgeName, cvEdge);
	}
}

int main()
{
	std::vector<cv::Vec3f> pos;
	std::vector<cv::Vec3b> color;

	int num_pose = 20;
	int radius = 10;
	std::vector<Eigen::Isometry3d> gtPoses; //w2c
	//gtPose generation
	for (int i = 0; i < num_pose; i++) {
		double rad = 2 * M_PI / num_pose * i;
		Eigen::AngleAxisd rot(rad, Eigen::Vector3d(0, 1, 0));
		Eigen::Vector3d trans(sin(rad), 0, cos(rad));
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(rot);
		pose.pretranslate(trans * radius);
		gtPoses.push_back(pose.inverse());
	}

	//obsPose generation
	std::vector<Eigen::Isometry3d> obsPoses; //w2c
	for (int i = 0; i < num_pose; i++) {
		double rad = 2 * M_PI / num_pose * i * 0.95;
		Eigen::AngleAxisd rot(rad, Eigen::Vector3d(0, 1, 0));
		Eigen::Vector3d trans(sin(rad), 0, cos(rad));
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(rot);
		pose.pretranslate(trans * radius);
		obsPoses.push_back(pose.inverse());
	}

	//display
	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizPoses(myWindow, std::string("gtPoses"), gtPoses);
		vizPoses(myWindow, std::string("obsPoses"), obsPoses, cv::viz::Color(0, 255, 0));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}

	//obsEdge generation
	std::vector<Edge> edges;
	for (int i = 0; i < num_pose - 1; i++) {
		Edge edge;
		edge.relativePose = obsPoses[i + 1] * obsPoses[i].inverse();
		edge.id = std::pair<size_t, size_t>(i, i + 1);
		edges.push_back(edge);
	}
	//some edge is estimated correctly
	for (int i = 0; i < num_pose - 2; i++) {
		Edge edge;
		edge.relativePose = gtPoses[i + 2] * gtPoses[i].inverse();
		edge.id = std::pair<size_t, size_t>(i, i + 2);
		edges.push_back(edge);
	}

	//loop edge is estimated correctly
	std::vector<Edge> loopEdges;
	{
		Edge edge;
		edge.relativePose = gtPoses[num_pose-1] * gtPoses[0].inverse();
		edge.id = std::pair<size_t, size_t>(0, num_pose-1);
		loopEdges.push_back(edge);
	}

	//display
	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizPoses(myWindow, std::string("obsPoses"), obsPoses, cv::viz::Color(0, 255, 0));
		vizEdges(myWindow, std::string("obsEdges"), edges, obsPoses);
		vizEdges(myWindow, std::string("loopEdges"), loopEdges, obsPoses, cv::viz::Color(0, 0, 255));
		myWindow.spinOnce(1, true);
		myWindow.spin();
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

	// set vertices
	for (size_t i = 0, iend = num_pose; i < iend; i++) {
		g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
		Eigen::Matrix<double, 3, 3> Rcw = obsPoses[i].rotation();
		Eigen::Matrix<double, 3, 1> tcw = obsPoses[i].translation();
		g2o::Sim3 Scw(Rcw, tcw, 1.0);
		VSim3->setEstimate(Scw);
		if (i == 0) VSim3->setFixed(true);
		VSim3->setId(i);
		VSim3->setMarginalized(false);
		VSim3->_fix_scale = true;
		optimizer.addVertex(VSim3);
	}

	std::set<std::pair<long unsigned int, long unsigned int> > sInsertedEdges;
	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	// set edgess
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

	// set loop edge
	for (int i = 0; i < loopEdges.size(); i++) {
		Edge edge = loopEdges[i];
		Eigen::Matrix<double, 3, 3> Rji = edge.relativePose.rotation();
		Eigen::Matrix<double, 3, 1> tji = edge.relativePose.translation();
		const g2o::Sim3 Sji(Rji, tji, 1.0);
		g2o::EdgeSim3* e = new g2o::EdgeSim3();
		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(num_pose - 1)));
		e->setMeasurement(Sji);
		e->information() = matLambda;
		optimizer.addEdge(e);
	}

	// optimize
	optimizer.initializeOptimization();
	optimizer.optimize(20);

	// expand estPoses
	std::vector<Eigen::Isometry3d> estPoses; //w2c
	for (int i = 0; i < num_pose; i++) {
		g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(i));
		g2o::Sim3 CorrectedSiw = VSim3->estimate();
		double s = CorrectedSiw.scale();
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(CorrectedSiw.rotation().toRotationMatrix());
		pose.pretranslate(CorrectedSiw.translation() * (1. / s));
		estPoses.push_back(pose);
	}

	//display
	{
		cv::viz::Viz3d myWindow("VIZ");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizPoses(myWindow, std::string("gtPoses"), gtPoses);
		vizPoses(myWindow, std::string("obsPoses"), obsPoses, cv::viz::Color(0, 255, 0));
		vizPoses(myWindow, std::string("estPoses"), estPoses, cv::viz::Color(0, 0, 255));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}
	return 0;
}