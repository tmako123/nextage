
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

#include "PlyGenerator.h"

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

int main()
{
	Loader loader;
	loader.LoadFile("C:/dataset/bundle/problem-394-100368-pre.txt");
	std::vector<ObsPoint2d> &obsPoints2d = loader.obsPoints2d;
	std::vector<Camera> &cameras = loader.cameras;
	std::vector<Eigen::Vector3d> &points3d = loader.points3d;

	PlyGenerator ply;
	ply.initialize();

	for (auto &pt : points3d) {
		ply.registerPoints3d(pt.cast<float>());
	}
	ply.outputPly(std::string("points.ply"));

	ply.reset();
	for (auto &cam : cameras) {
		ply.registerCamera(cam.pose.cast<float>(), 0.1, Eigen::Vector3i(0, 0, 255));
	}
	ply.outputPly(std::string("camera.ply"));


}

