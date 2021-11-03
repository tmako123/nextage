/*******************************************************
 * Copyright(c) 2021, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include "../ThirdParty/svo_pro/feature_alignment.h"
#include "../ThirdParty/svo_pro/patch_utils.h"
#include "../ThirdParty/svo_pro/patch_warp.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

int main()
{
    std::string file0("../../../svo/ThirdParty/data/EuRoC_img0.png");
    cv::Mat src;
    src = cv::imread(file0, -1);

    cv::Mat affine = cv::Mat::eye(2, 3, CV_32F);
    affine.at<float>(0, 2) = 4.f;
    affine.at<float>(1, 2) = 5.f;
    cv::Mat dst;
    cv::warpAffine(src, dst, affine, src.size());
    dst = dst + 50;

    //feature detection
    std::vector<cv::Point2f> srcPts;
    cv::goodFeaturesToTrack(src, srcPts, 500, 0.01, 10);

    //deature gt pos
    std::vector<cv::Point2f> dstPts;
    dstPts.reserve(srcPts.size());
    {
        Eigen::Matrix<float, 2, 3> affineE = Eigen::Map<Eigen::Matrix<float, 2, 3, Eigen::RowMajor>>(reinterpret_cast<float*>(affine.ptr()));
        //std::cout << affineE << std::endl;
        for (auto& srcPt : srcPts) {
            Eigen::Vector2f dstPt = affineE * Eigen::Vector3f(srcPt.x, srcPt.y, 1.f);
            dstPts.push_back(cv::Point2f(dstPt.x(), dstPt.y()));
            //std::cout << "--------------------" << std::endl;
            //std::cout << srcPt << std::endl;
            //std::cout << dstPt << std::endl;
        }
    }

    {
        //LK
        std::vector<cv::Point2f> estPts;
        //feature tracking
        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(src, dst, srcPts, estPts, status, err, cv::Size(15, 15), 2, criteria);

        //count good points
        int count = 0;
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                if ((Eigen::Vector2f(estPts[i].x, estPts[i].y)
                        - Eigen::Vector2f(dstPts[i].x, dstPts[i].y))
                        .norm()
                    < 1.f) {
                    count++;
                }
            }
        }
        std::cout << "calcOpticalFlowPyrLK : " << count << std::endl;

        cv::Mat showImg;
        cv::RNG rng;
        cv::cvtColor(dst, showImg, cv::COLOR_GRAY2BGR);
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                // draw the tracks
                cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                cv::line(showImg, srcPts[i], estPts[i], color, 1);
                cv::circle(showImg, estPts[i], 2, color, -1);
            }
        }
        cv::imshow("src", src);
        cv::imshow("dst", dst);
        cv::imshow("klt", showImg);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    {
        //svo arign2d
        std::vector<cv::Point2f> estPts;
        std::vector<uchar> status;
        estPts.reserve(srcPts.size());
        status.reserve(srcPts.size());
        const int harfPatchSize = 8;
        const int patchSize = harfPatchSize * 2;
        const int patchArea = patchSize * patchSize;
        const int patchWithBorderSize = patchSize + 2;
        const int patchWithBorderArea = patchWithBorderSize * patchWithBorderSize;

        alignas(16) svo::uint8_t patchWithBorder[patchWithBorderArea];
        alignas(16) svo::uint8_t patch[patchArea];
        for (int i = 0; i < srcPts.size(); i++) {
            svo::warp::createPatchNoWarp(src, Eigen::Vector2i(srcPts[i].x, srcPts[i].y), harfPatchSize + 1, patchWithBorder);
            svo::patch_utils::createPatchFromPatchWithBorder(patchWithBorder, patchSize, patch);
            svo::Keypoint pt(srcPts[i].x, srcPts[i].y);
            //svo::GradientVector grad(1, 0);
            //bool ok = svo::feature_alignment::align1D(dst, grad, patchWithBorder, patch, 30, false, false, &pt);
            bool ok = svo::feature_alignment::align2D(dst, patchWithBorder, patch, 30, true, true, pt);
            estPts.push_back(cv::Point2f(pt.x(), pt.y()));
            status.push_back(ok == true);
        }

        //count good points
        int count = 0;
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                if ((Eigen::Vector2f(estPts[i].x, estPts[i].y)
                        - Eigen::Vector2f(dstPts[i].x, dstPts[i].y))
                        .norm()
                    < 1.f) {
                    count++;
                }
            }
        }
        std::cout << "align2D : " << count << std::endl;

        cv::Mat showImg;
        cv::RNG rng;
        cv::cvtColor(dst, showImg, cv::COLOR_GRAY2BGR);
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                // draw the tracks
                cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                cv::line(showImg, srcPts[i], estPts[i], color, 1);
                cv::circle(showImg, estPts[i], 2, color, -1);
            }
        }
        cv::imshow("src", src);
        cv::imshow("dst", dst);
        cv::imshow("align2D", showImg);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    {
        //svo arignPyr2d
        int klt_max_level = 4;
        int klt_min_level = 0;
        std::vector<int> klt_patch_sizes = { 16, 16, 16, 8, 8 };
        double klt_min_update_squared = 0.001;
        int klt_max_iter = 30;
        std::vector<cv::Mat> srcPyr, dstPyr;
        srcPyr.push_back(src);
        dstPyr.push_back(dst);
        for (int i = 1; i <= klt_max_level; i++) {
            cv::Mat tmpSrc, tmpDst;
            cv::pyrDown(srcPyr.back(), tmpSrc, cv::Size(srcPyr.back().cols / 2, srcPyr.back().rows / 2));
            srcPyr.push_back(tmpSrc);
            cv::pyrDown(dstPyr.back(), tmpDst, cv::Size(dstPyr.back().cols / 2, dstPyr.back().rows / 2));
            dstPyr.push_back(tmpDst);
        }

        std::vector<cv::Point2f> estPts;
        std::vector<uchar> status;
        status.resize(srcPts.size());
        estPts.reserve(srcPts.size());
        for (auto& pt : srcPts) {
            estPts.push_back(pt);
        }
        svo::feature_alignment::alignPyr2DVec(srcPyr, dstPyr,
            klt_max_level, klt_min_level, klt_patch_sizes,
            klt_max_iter, klt_min_update_squared,
            srcPts, estPts, status);

        //count good points
        int count = 0;
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                if ((Eigen::Vector2f(estPts[i].x, estPts[i].y)
                        - Eigen::Vector2f(dstPts[i].x, dstPts[i].y))
                        .norm()
                    < 1.f) {
                    count++;
                }
            }
        }
        std::cout << "alignPyr2DVec : " << count << std::endl;

        cv::Mat showImg;
        cv::RNG rng;
        cv::cvtColor(dst, showImg, cv::COLOR_GRAY2BGR);
        for (uint i = 0; i < srcPts.size(); i++) {
            if (status[i] == 1) {
                // draw the tracks
                cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                cv::line(showImg, srcPts[i], estPts[i], color, 1);
                cv::circle(showImg, estPts[i], 2, color, -1);
            }
        }
        cv::imshow("src", src);
        cv::imshow("dst", dst);
        cv::imshow("alignPyr2DVec", showImg);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return 0;
}
