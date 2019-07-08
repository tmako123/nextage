
/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "plyGenerator.h"

void PlyGenerator::registerPoints3d(Eigen::Vector3f point, Eigen::Vector3i color) {
	m_outputBuf.points3d.push_back(point);
	m_outputBuf.colors.push_back(color);
}

void PlyGenerator::registerCamera(Eigen::Isometry3f pose, float scale, Eigen::Vector3i color) {
	std::vector<Eigen::Vector3f> camVertexes;

	//pyramid
	camVertexes.push_back(Eigen::Vector3f(0, 0, 0));
	camVertexes.push_back(Eigen::Vector3f(-1, -1, 2));
	camVertexes.push_back(Eigen::Vector3f(-1, 1, 2));
	camVertexes.push_back(Eigen::Vector3f(1, 1, 2));
	camVertexes.push_back(Eigen::Vector3f(1, -1, 2));

	//apply pose
	Eigen::Isometry3f invPose = pose.inverse();
	for (auto&pt : camVertexes) {
		pt = invPose * (pt * scale);
	}

	int offset = m_outputBuf.points3d.size();
	//add vertex
	for (auto&pt : camVertexes) {
		m_outputBuf.points3d.push_back(pt);
		m_outputBuf.colors.push_back(color);
	}
	//add faces
	std::vector<std::vector<int>> faces;
	faces.push_back(std::vector<int>{ 0, 1, 2 }); //c++11
	faces.push_back(std::vector<int>{ 0, 2, 3 });
	faces.push_back(std::vector<int>{ 0, 3, 4 });
	faces.push_back(std::vector<int>{ 0, 4, 1 });
	faces.push_back(std::vector<int>{ 4, 3, 2, 1 });
	for (auto& f : faces) {
		for (auto&e : f) {
			e += offset;
		}
		m_outputBuf.faceIndexes.push_back(f);
	}
}

void PlyGenerator::registerCube(Eigen::Vector3f pos, float scale, Eigen::Vector3i color) {
}


bool PlyGenerator::outputPly(std::string& fname) {

	const int num_vertex = m_outputBuf.points3d.size();
	const int num_faces = m_outputBuf.faceIndexes.size();
	if (num_vertex != m_outputBuf.colors.size()) return false;

	std::ofstream ofs(fname, std::ios::out);
	if (!ofs) return false;

	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex " << num_vertex << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "property uchar red" << std::endl;
	ofs << "property uchar green" << std::endl;
	ofs << "property uchar blue" << std::endl;
	ofs << "element face " << num_faces << std::endl;
	ofs << "property list uchar int vertex_indices" << std::endl;
	ofs << "end_header" << std::endl;

	//vertex
	for (int i = 0; i < num_vertex; i++) {
		Eigen::Vector3f& pt = m_outputBuf.points3d[i];
		Eigen::Vector3i& cl = m_outputBuf.colors[i];
		ofs << pt.x() << " " 
			<< pt.y() << " " 
			<< pt.z() << " " 
			<< cl.x() << " " 
			<< cl.y() << " " 
			<< cl.z() << std::endl;
	}

	//faces
	for (auto &fs : m_outputBuf.faceIndexes) {
		int num_face_vertexes = fs.size();
		if (num_face_vertexes < 3) continue;
		ofs << num_face_vertexes << " " ;
		for (int i = 0; i < num_face_vertexes; i++) {
			int id = fs[i];
			if (i == num_face_vertexes - 1) {
				ofs << id << std::endl;
			}
			else {
				ofs << id << " ";
			}
		}
	}
	return true;
}
