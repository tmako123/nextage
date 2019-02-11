#include "vis.h"

void Vis::vizPoses(cv::viz::Viz3d& window, std::string& name, std::vector<std::pair<double, Eigen::Isometry3d>> poses/*w2c*/, cv::viz::Color color) {
	for (int i = 0; i < poses.size(); i++) {
		cv::Affine3d camPose;
		cv::eigen2cv(Eigen::Affine3d(poses[i].second.inverse()).matrix(), camPose.matrix);
		std::string widgetFrustumName = name + std::to_string(i);
		cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.6, 0.4), 0.1, color); // Camera frustum
		window.showWidget(widgetFrustumName, cpw_frustum, camPose);
	}
}

void Vis::vizTrajectory(cv::viz::Viz3d& window, std::string& name, std::vector<std::pair<double, Eigen::Isometry3d>> poses/*w2c*/, cv::viz::Color color) {
	for (int i = 1; i < poses.size(); i++) {
		std::string widgetFrustumName = name + std::to_string(i);
		Eigen::Vector3d first = poses[i-1].second.inverse().translation();
		Eigen::Vector3d second = poses[i].second.inverse().translation();
		cv::Point3d cvFirst(first.x(), first.y(), first.z());
		cv::Point3d cvSecond(second.x(), second.y(), second.z());
		cv::viz::WLine cvEdge(cvFirst, cvSecond, color);
		std::string widgetEdgeName = "EDGES" + std::to_string(i);
		window.showWidget(widgetEdgeName, cvEdge);
	}
}