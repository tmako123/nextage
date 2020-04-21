/*******************************************************
 * Copyright(c) 2018, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Dense>

#define SUBPIX_VERBOSE 0

void generateRefPatchNoWarpInterpolate(const cv::Mat& img, const Eigen::Vector2d& px,
	cv::Mat& ref_patch_with_border)
{
	// compute interpolation weights
	const int u_r = floor(px[0]);
	const int v_r = floor(px[1]);
	const float subpix_u = px[0] - u_r;
	const float subpix_v = px[1] - v_r;
	const float wTL = (1.0 - subpix_u)*(1.0 - subpix_v);
	const float wTR = subpix_u * (1.0 - subpix_v);
	const float wBL = (1.0 - subpix_u)*subpix_v;
	const float wBR = subpix_u * subpix_v;

	// loop through search_patch, interpolate
	ref_patch_with_border = cv::Mat(10, 10, CV_8UC1);
	uint8_t* patch_ptr = ref_patch_with_border.data;
	const int stride = img.step.p[0];
	for (int y = 0; y < 10; ++y)
	{
		uint8_t* img_ptr = (uint8_t*)img.data + (v_r + y - 4 - 1)*stride + u_r - 4 - 1;
		for (int x = 0; x < 10; ++x, ++patch_ptr, ++img_ptr)
			*patch_ptr = wTL * img_ptr[0] + wTR * img_ptr[1] + wBL * img_ptr[stride] + wBR * img_ptr[stride + 1];
	}
}


bool align1D(
	const cv::Mat& cur_img,
	const Eigen::Vector2f& dir,                  // direction in which the patch is allowed to move
	uint8_t* ref_patch_with_border,
	uint8_t* ref_patch,
	const int n_iter,
	Eigen::Vector2d& cur_px_estimate,
	double& h_inv)
{
	const int halfpatch_size_ = 4;
	const int patch_size = 8;
	const int patch_area = 64;
	bool converged = false;

	// compute derivative of template and prepare inverse compositional
	float ref_patch_dv[patch_area];
	Eigen::Matrix2f H; H.setZero();

	// compute gradient and hessian
	const int ref_step = patch_size + 2;
	float* it_dv = ref_patch_dv;
	for (int y = 0; y < patch_size; ++y)
	{
		uint8_t* it = ref_patch_with_border + (y + 1)*ref_step + 1;
		for (int x = 0; x < patch_size; ++x, ++it, ++it_dv)
		{
			Eigen::Vector2f J;
			J[0] = 0.5*(dir[0] * (it[1] - it[-1]) + dir[1] * (it[ref_step] - it[-ref_step]));
			J[1] = 1;
			*it_dv = J[0];
			H += J * J.transpose();
		}
	}
	h_inv = 1.0 / H(0, 0)*patch_size*patch_size;
	Eigen::Matrix2f Hinv = H.inverse();
	float mean_diff = 0;

	// Compute pixel location in new image:
	float u = cur_px_estimate.x();
	float v = cur_px_estimate.y();

	// termination condition
	const float min_update_squared = 0.03*0.03;
	const int cur_step = cur_img.step.p[0];
	float chi2 = 0;
	Eigen::Vector2f update; update.setZero();
	for (int iter = 0; iter < n_iter; ++iter)
	{
		int u_r = floor(u);
		int v_r = floor(v);
		if (u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols - halfpatch_size_ || v_r >= cur_img.rows - halfpatch_size_)
			break;

		if (isnan(u) || isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
			return false;

		// compute interpolation weights
		float subpix_x = u - u_r;
		float subpix_y = v - v_r;
		float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
		float wTR = subpix_x * (1.0 - subpix_y);
		float wBL = (1.0 - subpix_x)*subpix_y;
		float wBR = subpix_x * subpix_y;

		// loop through search_patch, interpolate
		uint8_t* it_ref = ref_patch;
		float* it_ref_dv = ref_patch_dv;
		float new_chi2 = 0.0;
		Eigen::Vector2f Jres; Jres.setZero();
		for (int y = 0; y < patch_size; ++y)
		{
			uint8_t* it = (uint8_t*)cur_img.data + (v_r + y - halfpatch_size_)*cur_step + u_r - halfpatch_size_;
			for (int x = 0; x < patch_size; ++x, ++it, ++it_ref, ++it_ref_dv)
			{
				float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step + 1];
				float res = search_pixel - *it_ref + mean_diff;
				Jres[0] -= res * (*it_ref_dv);
				Jres[1] -= res;
				new_chi2 += res * res;
			}
		}

		if (iter > 0 && new_chi2 > chi2)
		{
#if SUBPIX_VERBOSE
			std::cout << "error increased." << std::endl;
#endif
			u -= update[0];
			v -= update[1];
			break;
		}

		chi2 = new_chi2;
		update = Hinv * Jres;
		u += update[0] * dir[0];
		v += update[0] * dir[1];
		mean_diff += update[1];

#if SUBPIX_VERBOSE
		std::cout << "Iter " << iter << ":"
			<< "\t u=" << u << ", v=" << v
			<< "\t update = " << update[0] << ", " << update[1]
			<< "\t new chi2 = " << new_chi2 << std::endl;
#endif

		if (update[0] * update[0] + update[1] * update[1] < min_update_squared)
		{
#if SUBPIX_VERBOSE
			std::cout << "converged." << std::endl;
#endif
			converged = true;
			break;
		}
	}

	cur_px_estimate << u, v;
	return converged;
}

bool align2D(
	const cv::Mat& cur_img,
	uint8_t* ref_patch_with_border,
	uint8_t* ref_patch,
	const int n_iter,
	Eigen::Vector2d& cur_px_estimate,
	bool no_simd)
{
	const int halfpatch_size_ = 4;
	const int patch_size_ = 8;
	const int patch_area_ = 64;
	bool converged = false;

	// compute derivative of template and prepare inverse compositional
	float ref_patch_dx[patch_area_];
	float ref_patch_dy[patch_area_];
	Eigen::Matrix3f H; H.setZero();

	// compute gradient and hessian
	const int ref_step = patch_size_ + 2;
	float* it_dx = ref_patch_dx;
	float* it_dy = ref_patch_dy;
	for (int y = 0; y < patch_size_; ++y)
	{
		uint8_t* it = ref_patch_with_border + (y + 1)*ref_step + 1;
		for (int x = 0; x < patch_size_; ++x, ++it, ++it_dx, ++it_dy)
		{
			Eigen::Vector3f J;
			J[0] = 0.5 * (it[1] - it[-1]);
			J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
			J[2] = 1;
			*it_dx = J[0];
			*it_dy = J[1];
			H += J * J.transpose();
		}
	}
	Eigen::Matrix3f Hinv = H.inverse();
	float mean_diff = 0;

	// Compute pixel location in new image:
	float u = cur_px_estimate.x();
	float v = cur_px_estimate.y();

	// termination condition
	const float min_update_squared = 0.03*0.03;
	const int cur_step = cur_img.step.p[0];
	//  float chi2 = 0;
	Eigen::Vector3f update; update.setZero();
	for (int iter = 0; iter < n_iter; ++iter)
	{
		int u_r = floor(u);
		int v_r = floor(v);
		if (u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols - halfpatch_size_ || v_r >= cur_img.rows - halfpatch_size_)
			break;

		if (isnan(u) || isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
			return false;

		// compute interpolation weights
		float subpix_x = u - u_r;
		float subpix_y = v - v_r;
		float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
		float wTR = subpix_x * (1.0 - subpix_y);
		float wBL = (1.0 - subpix_x)*subpix_y;
		float wBR = subpix_x * subpix_y;

		// loop through search_patch, interpolate
		uint8_t* it_ref = ref_patch;
		float* it_ref_dx = ref_patch_dx;
		float* it_ref_dy = ref_patch_dy;
		//    float new_chi2 = 0.0;
		Eigen::Vector3f Jres; Jres.setZero();
		for (int y = 0; y < patch_size_; ++y)
		{
			uint8_t* it = (uint8_t*)cur_img.data + (v_r + y - halfpatch_size_)*cur_step + u_r - halfpatch_size_;
			for (int x = 0; x < patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
			{
				float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step + 1];
				float res = search_pixel - *it_ref + mean_diff;
				Jres[0] -= res * (*it_ref_dx);
				Jres[1] -= res * (*it_ref_dy);
				Jres[2] -= res;
				//        new_chi2 += res*res;
			}
		}


		/*
			if(iter > 0 && new_chi2 > chi2)
			{
		#if SUBPIX_VERBOSE
			  cout << "error increased." << endl;
		#endif
			  u -= update[0];
			  v -= update[1];
			  break;
			}
			chi2 = new_chi2;
		*/
		update = Hinv * Jres;
		u += update[0];
		v += update[1];
		mean_diff += update[2];

#if SUBPIX_VERBOSE
		std::cout << "Iter " << iter << ":"
			<< "\t u=" << u << ", v=" << v
			<< "\t update = " << update[0] << ", " << update[1]
			//         << "\t new chi2 = " << new_chi2 << endl;
			<< std::endl;
#endif

			if (update[0] * update[0] + update[1] * update[1] < min_update_squared)
			{
#if SUBPIX_VERBOSE
				std::cout << "converged." << std::endl;
#endif
				converged = true;
				break;
			}
	}

	cur_px_estimate << u, v;
	return converged;
}

int main() {
	cv::Mat image0 = cv::imread("001.png", 0);
	cv::Mat image1 = cv::imread("002.png", 0);

	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	int maxCorners = 23;

	cv::imshow("win", image0);
	cv::waitKey(0);

	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(image0, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector,	k);

	cv::Mat copy;
	cv::cvtColor(image0, copy, cv::COLOR_GRAY2BGR);
	int r = 4;
	for (int i = 0; i < corners.size(); i++)
	{
		circle(copy, corners[i], r, cv::Scalar(0, 255, 0), -1, 8, 0);
	}

	/// Show what you got
	cv::imshow("win", copy);
	cv::waitKey(0);

	std::vector<Eigen::Vector2d> ePoint;
	for (auto& pt : corners) {
		ePoint.push_back(Eigen::Vector2d(pt.x, pt.y));
	}

	std::vector<Eigen::Vector2d> next;
	for (auto& pt : ePoint) {
		// create reference patch with border
		cv::Mat ref_patch_with_border;
		generateRefPatchNoWarpInterpolate(image0, pt, ref_patch_with_border);

		//cv::imshow("path", ref_patch_with_border);
		//cv::waitKey(10);

		// create reference patch, aligned
		Eigen::Vector2d px_est = pt;
		bool ret = align2D(image1, ref_patch_with_border.data, ref_patch_with_border.data, 1000, px_est, true);
		if (ret == false) continue;
		next.push_back(px_est);
		//std::cout << px_est << std::endl;
	}

	cv::Mat copy2;
	cv::cvtColor(image1, copy2, cv::COLOR_GRAY2BGR);
	for (auto& pt : next) {
		circle(copy2, cv::Point2d(pt.x(), pt.y()), r, cv::Scalar(0, 255, 0), -1, 8, 0);
	}

	/// Show what you got
	cv::imshow("win", copy2);
	cv::waitKey(0);



}