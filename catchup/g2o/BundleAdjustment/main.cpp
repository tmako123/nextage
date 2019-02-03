
/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv_lib.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/eigen.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

struct ObsPoint2d {
	int pointId;
	int cameraId;
	Eigen::Vector2d pt;
};

struct Camera {
	Eigen::Isometry3d pose; //w2c
	double focal;
	double d1;
	double d2;
};

class Loader {
public:
	Loader() {};
	~Loader() {};

	bool LoadFile(const char* filename) {
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			return false;
		};
		int num_cameras;
		int num_points;
		int num_observations;
		FscanfOrDie(fptr, "%d", &num_cameras);
		FscanfOrDie(fptr, "%d", &num_points);
		FscanfOrDie(fptr, "%d", &num_observations);

		int num_parameters_ = 9 * num_cameras + 3 * num_points;
		obsPoints2d.reserve(num_observations);
		for (int i = 0; i < num_observations; ++i) {
			ObsPoint2d pt2d;
			FscanfOrDie(fptr, "%d", &(pt2d.cameraId));
			FscanfOrDie(fptr, "%d", &(pt2d.pointId));
			FscanfOrDie(fptr, "%lf", &(pt2d.pt.x()));
			FscanfOrDie(fptr, "%lf", &(pt2d.pt.y()));
			obsPoints2d.push_back(pt2d);
			//std::cout << pt2d.pt << std::endl;
		}
		cameras.reserve(num_cameras);
		for (int i = 0; i < num_cameras; ++i) {
			Eigen::Vector3d rotVec;
			Eigen::Vector3d translation;
			Camera cam;
			cam.pose = Eigen::Isometry3d::Identity();
			//dataset contains w2c translation
			FscanfOrDie(fptr, "%lf", &(rotVec.x()));
			FscanfOrDie(fptr, "%lf", &(rotVec.y()));
			FscanfOrDie(fptr, "%lf", &(rotVec.z()));
			FscanfOrDie(fptr, "%lf", &(translation.x()));
			FscanfOrDie(fptr, "%lf", &(translation.y()));
			FscanfOrDie(fptr, "%lf", &(translation.z()));
			double angle = rotVec.norm();
			rotVec.normalize();
			cam.pose.prerotate(Eigen::AngleAxisd(angle, rotVec));
			cam.pose.pretranslate(translation);
			cam.pose = cam.pose;
			FscanfOrDie(fptr, "%lf", &(cam.focal));
			FscanfOrDie(fptr, "%lf", &(cam.d1));
			FscanfOrDie(fptr, "%lf", &(cam.d2));
			cameras.push_back(cam);
		}
		points3d.reserve(num_points);
		for (int i = 0; i < num_points; i++) {
			Eigen::Vector3d point3d;
			FscanfOrDie(fptr, "%lf", &(point3d.x()));
			FscanfOrDie(fptr, "%lf", &(point3d.y()));
			FscanfOrDie(fptr, "%lf", &(point3d.z()));
			points3d.push_back(point3d);
		}

		return true;
	}

private:
	template<typename T>
	bool FscanfOrDie(FILE *fptr, const char *format, T *value) {
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1) {
			return false;
		}
		return true;
	}

public:
	std::vector<ObsPoint2d> obsPoints2d;
	std::vector<Camera> cameras;
	std::vector<Eigen::Vector3d> points3d;
};

void vizCameras(cv::viz::Viz3d& window, std::string& name, std::vector<Camera> cameras/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255)) {
	for (int i = 0; i < cameras.size(); i++) {
		cv::Affine3d camPose;
		cv::eigen2cv(Eigen::Affine3d(cameras[i].pose.inverse()).matrix(), camPose.matrix);
		std::string widgetFrustumName = name + std::to_string(i);
		cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -0.1, color); // Camera frustum
		window.showWidget(widgetFrustumName, cpw_frustum, camPose);
	}
}

void vizPoints(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Vector3d> points3d, cv::viz::Color color = cv::viz::Color(255, 255, 255)) {
	std::vector<cv::Vec3d> cvPoints3d(points3d.size());
	for (int i = 0; i < points3d.size(); i++) {
		cv::Vec3d pt3d(points3d[i].x(), points3d[i].y(), points3d[i].z());
		cvPoints3d[i] = pt3d;
	}
	cv::viz::WCloud wcloud(cvPoints3d, color);
	window.showWidget(name, wcloud);
}

void vizPoses(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Isometry3d > poses/*w2c*/, cv::viz::Color color = cv::viz::Color(255, 255, 255)) {
	for (int i = 0; i < poses.size(); i++) {
		cv::Affine3d camPose;
		cv::eigen2cv(Eigen::Affine3d(poses[i].inverse()).matrix(), camPose.matrix);
		std::string widgetFrustumName = name + std::to_string(i);
		cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -0.1, color); // Camera frustum
		window.showWidget(widgetFrustumName, cpw_frustum, camPose);
	}
}

void addNoise(std::vector<Eigen::Vector3d>& points3d, double mu = 0.0, double sigma = 0.1) {
	std::mt19937 rand_src(12345);
	for (int i = 0; i < points3d.size(); i++) {
		std::normal_distribution<double> rand_dist(mu, sigma);
		points3d[i].x() += rand_dist(rand_src);
		points3d[i].y() += rand_dist(rand_src);
		points3d[i].z() += rand_dist(rand_src);
	}
}

int main()
{
	Loader loader;
	loader.LoadFile("C:/dataset/bundle/problem-394-100368-pre.txt");
	std::vector<ObsPoint2d> &obsPoints2d = loader.obsPoints2d;
	std::vector<Camera> &cameras = loader.cameras;
	std::vector<Eigen::Vector3d> &points3d = loader.points3d;

	//visualize raw data
	{
		cv::viz::Viz3d myWindow("Point Cloud");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizCameras(myWindow, std::string("camera"), cameras);
		vizPoints(myWindow, std::string("points3d"), points3d, cv::viz::Color(0, 0, 255));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}

	addNoise(points3d);

	//visualize noized data
	{
		cv::viz::Viz3d myWindow("Point Cloud");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizCameras(myWindow, std::string("camera"), cameras);
		vizPoints(myWindow, std::string("points3d"), points3d, cv::viz::Color(0, 0, 255));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
		g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

	optimizer.setAlgorithm(solver);

	const float thHuber2D = sqrt(5.99);

	// Set vertices
	for (size_t i = 0; i < cameras.size(); i++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		Eigen::Matrix<double, 3, 3> Rcw = cameras[i].pose.rotation();
		Eigen::Matrix<double, 3, 1> tcw = cameras[i].pose.translation();
		g2o::SE3Quat Scw(Rcw, tcw);
		vSE3->setEstimate(Scw);
		vSE3->setId(i);
		vSE3->setFixed(i == 0);
		optimizer.addVertex(vSE3);
	}

	//set points
	for (size_t i = 0; i < points3d.size(); i++) {
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(points3d[i]);
		const int id = i + cameras.size();
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
	}

	//set edges : obserbation
	for (size_t i = 0; i < obsPoints2d.size(); i++) {
		ObsPoint2d& obsPoint2d = obsPoints2d[i];
		g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(obsPoint2d.pointId + cameras.size())));
		e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(obsPoint2d.cameraId)));
		e->setInformation(Eigen::Matrix2d::Identity());

		{
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuber2D);
		}
     	
		e->fx = cameras[obsPoint2d.cameraId].focal;
		e->fy = cameras[obsPoint2d.cameraId].focal;
		e->cx = 0;
		e->cy = 0;
		double d1 = cameras[obsPoint2d.cameraId].d1;
		double d2 = cameras[obsPoint2d.cameraId].d2;

		//obserbed points are distorted, so computes the ideal point coordinates here.
		cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
			e->fx, 0., 0.,
			0., e->fy, 0.,
			0., 0., 0);
		cv::Mat distCoeff = (cv::Mat_<double>(1, 4) << d1, d2, 0., 0.);
		std::vector<cv::Point2d> src;
		//dataset's camera model is y-up.
		src.push_back(cv::Point2d(-obsPoint2d.pt.x(), -obsPoint2d.pt.y()));
		std::vector<cv::Point2d> dst;
		cv::undistortPoints(src, dst, cameraMatrix, distCoeff);
		dst[0].x *= e->fx;
		dst[0].y *= e->fy;
		e->setMeasurement(Eigen::Vector2d(dst[0].x, dst[0].y));
		//double undistortion = -1.-
		//e->setMeasurement(obsPoint2d.pt * undistortion);
		optimizer.addEdge(e);
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(20);

	std::vector<Eigen::Vector3d> estPoints3d;
	std::vector<Eigen::Isometry3d> estPoses;

	// extruct optimized poses
	for (size_t i = 0; i < cameras.size(); i++)
	{
		g2o::VertexSE3Expmap* VSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
		g2o::SE3Quat CorrectedSiw = VSE3->estimate();
		Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		pose.prerotate(CorrectedSiw.rotation());
		pose.pretranslate(CorrectedSiw.translation());
		estPoses.push_back(pose); //w2c
	}

	// extruct optimized points
	for (size_t i = 0; i < points3d.size(); i++) {
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + cameras.size()));
		estPoints3d.push_back(vPoint->estimate());
	}

	// visualize optimized result
	{
		cv::viz::Viz3d myWindow("Point Cloud");
		myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
		vizPoses(myWindow, std::string("camera"), estPoses);
		vizPoints(myWindow, std::string("points3d"), estPoints3d, cv::viz::Color(0, 0, 255));
		myWindow.spinOnce(1, true);
		myWindow.spin();
		myWindow.removeAllWidgets();
	}
}

