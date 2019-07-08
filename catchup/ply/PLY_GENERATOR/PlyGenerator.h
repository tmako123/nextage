
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



class PlyGenerator {
public:
	PlyGenerator() {};
	~PlyGenerator() {};

	void initialize() {
		reset();
	};

	void reset() {
		m_outputBuf.colors.clear();
		m_outputBuf.faceIndexes.clear();
		m_outputBuf.points3d.clear();
	}

	void registerPoints3d(Eigen::Vector3f point, Eigen::Vector3i color = Eigen::Vector3i(0, 0, 0));

	void registerCamera(Eigen::Isometry3f pose, float scale = 1.0, Eigen::Vector3i color = Eigen::Vector3i(0, 0, 0));

	void registerCube(Eigen::Vector3f pos = Eigen::Vector3f(0, 0, 0), float scale = 1.0, Eigen::Vector3i color = Eigen::Vector3i(0, 0, 0));

	bool outputPly(std::string& fname);
	

protected:

	struct OutputBuf{
		std::vector<Eigen::Vector3f> points3d;
		std::vector<Eigen::Vector3i> colors;
		std::vector<std::vector<int>> faceIndexes;
	} m_outputBuf;
	
};