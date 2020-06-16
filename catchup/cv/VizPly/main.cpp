/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include "happly/happly.h"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

void vizPoints(cv::viz::Viz3d& window, std::string& name, std::vector<Eigen::Vector3d> points3d, cv::viz::Color color = cv::viz::Color(255, 255, 255))
{
    std::vector<cv::Vec3d> cvPoints3d(points3d.size());
    for (int i = 0; i < points3d.size(); i++) {
        cv::Vec3d pt3d(points3d[i].x(), points3d[i].y(), points3d[i].z());
        cvPoints3d[i] = pt3d;
    }
    cv::viz::WCloud wcloud(cvPoints3d, color);
    window.showWidget(name, wcloud);
}

int main()
{
    ///read ply
    happly::PLYData plyIn("../../../cv/VizPly/stanford/bun_zipper.ply");
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    std::vector<Eigen::Vector3d> points3d;
    for (auto& pt : vPos) {
        points3d.push_back(Eigen::Vector3d(pt.at(0), pt.at(1), pt.at(2)));
    }

    ///show with viz
    cv::viz::Viz3d myWindow("Point Cloud");
    vizPoints(myWindow, std::string("points3d"), points3d, cv::viz::Color(0, 0, 255));

    myWindow.spinOnce(1, true);
    myWindow.spin();
    myWindow.removeAllWidgets();

    return 0;
}