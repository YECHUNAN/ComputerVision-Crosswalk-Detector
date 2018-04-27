/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		Stripe class implementation
*/
#include "Stripe.h"
#include <vector>
#include <iostream>

Stripe::Stripe(Segment& seg1, Segment& seg2) {
	this->up_seg = seg1;
	this->down_seg = seg2;
}

cv::Point2f Stripe::get_centroid() {
	float x = (this->up_seg.xmin + this->up_seg.xmax + this->down_seg.xmin + this->down_seg.xmax) / 4;
	float y = (this->up_seg.ymin + this->up_seg.ymax + this->down_seg.ymin + this->down_seg.ymax) / 4;
	return cv::Point2f(x, y);
}
float Stripe::get_vertical_width() {
	cv::Point2f c = get_centroid();
	return std::abs(c.x * cos(this->up_seg.theta) + c.y * sin(this->up_seg.theta) - up_seg.rho) +
		std::abs(c.x * cos(this->down_seg.theta) + c.y * sin(this->down_seg.theta) - down_seg.rho);
}
float Stripe::get_L() {
	return pow((this->up_seg.xmax - this->up_seg.xmin) / std::abs(sin(this->up_seg.theta)) +
		(this->down_seg.xmax - this->down_seg.xmin) / std::abs(sin(this->down_seg.theta)), 0.25);

}

bool check_color(const Segment& seg1, const Segment& seg2, const cv::Mat& img) {
	cv::Mat mask(cv::Size(N, M), CV_8UC1);
	for (int r = 0; r < M; ++r) {
		for (int c = 0; c < N; ++c) {
			mask.at<uchar>(r, c) = 0;
		}
	}

	std::vector<cv::Point> ROI_corner_points;
	ROI_corner_points.emplace_back(round(seg1.xmin), round(M - seg1.ymin));
	ROI_corner_points.emplace_back(round(seg1.xmax), round(M - seg1.ymax));
	ROI_corner_points.emplace_back(round(seg2.xmax), round(M - seg2.ymax));
	ROI_corner_points.emplace_back(round(seg2.xmin), round(M - seg2.ymin));
	
	std::vector<cv::Point> ROI_poly;
	cv::convexHull(ROI_corner_points, ROI_poly);
	cv::fillConvexPoly(mask, ROI_poly, cv::Scalar(255));

	int total_pix_num = 0;
	int white_pix_num = 0;
	for (int i = (int)round(std::min(seg1.xmin, seg2.xmin)); i < (int)round(std::max(seg1.xmax, seg2.xmax)); ++i) {
		for (int j = (int)round(std::min(seg2.ymin, seg2.ymax)); j < (int)round(std::max(seg1.ymin, seg1.ymax)); ++j) {
			int r = std::max(1, std::min(M - j + 1, M - 1));
			int c = std::max(1, std::min(i, N - 1));
			if (mask.at<uchar>(r, c) > 0) {
				++total_pix_num;
				if (img.at<float>(r, c) > white_threshold) {
					++white_pix_num;
				}
			}
		}
	}

	if ( ((float)white_pix_num / (float)total_pix_num) > 0.65) {
		return true;
	}
	return false;
}

float compute_connectivity(Stripe& stripe_up, Stripe& stripe_down) {
	float y_thresh = 5.0f;
	float d_thresh = 50.0f;
	float R_thresh = 0.15f;

	if (abs(stripe_up.get_centroid().y - stripe_down.get_centroid().y) <= y_thresh) {
		return 0.3*std::max(stripe_up.get_L(), stripe_down.get_L()) / std::min(stripe_up.get_L(), stripe_down.get_L());
	}
	if ((stripe_up.get_vertical_width() - stripe_down.get_vertical_width())
		* (stripe_up.get_centroid().y - stripe_down.get_centroid().y) > 0) {
		return 0.0;
	}
	float d = abs(stripe_up.get_centroid().y - stripe_down.get_centroid().y);
	if (d > d_thresh) {
		return 0.0;
	}
	float v1 = stripe_up.get_centroid().x;
	float A = (stripe_up.up_seg.rho - v1 * cos(stripe_up.up_seg.theta)) / sin(stripe_up.up_seg.theta);
	float B = (stripe_up.down_seg.rho - v1 * cos(stripe_up.down_seg.theta)) / sin(stripe_up.down_seg.theta);
	float C = (stripe_down.up_seg.rho - v1 * cos(stripe_down.up_seg.theta)) / sin(stripe_down.up_seg.theta);
	float D = (stripe_down.down_seg.rho - v1 * cos(stripe_down.down_seg.theta)) / sin(stripe_down.down_seg.theta);
	float r1 = abs((A - B)*(C - D) / ((C - B)*(A - D)));
	float v2 = stripe_down.get_centroid().x;
	A = (stripe_up.up_seg.rho - v1 * cos(stripe_up.up_seg.theta)) / sin(stripe_up.up_seg.theta);
	B = (stripe_up.down_seg.rho - v1 * cos(stripe_up.down_seg.theta)) / sin(stripe_up.down_seg.theta);
	C = (stripe_down.up_seg.rho - v1 * cos(stripe_down.up_seg.theta)) / sin(stripe_down.up_seg.theta);
	D = (stripe_down.down_seg.rho - v1 * cos(stripe_down.down_seg.theta)) / sin(stripe_down.down_seg.theta);
	float r2 = abs((A - B)*(C - D) / ((C - B)*(A - D)));
	float R = (abs(r1 - 0.25) + abs(r2 - 0.25) / 2 + 2 * abs(r1 - r2));
	if (R >= R_thresh) {
		return 0.0;
	}
	return R;
}

void shade_area(Stripe stripe, cv::Mat img) {
	Segment seg1 = stripe.up_seg;
	Segment seg2 = stripe.down_seg;

	std::vector<cv::Point> ROI_corner_points;
	ROI_corner_points.emplace_back(round(seg1.xmin), round(M - seg1.ymin));
	ROI_corner_points.emplace_back(round(seg1.xmax), round(M - seg1.ymax));
	ROI_corner_points.emplace_back(round(seg2.xmax), round(M - seg2.ymax));
	ROI_corner_points.emplace_back(round(seg2.xmin), round(M - seg2.ymin));

	std::vector<cv::Point> ROI_poly;
	cv::convexHull(ROI_corner_points, ROI_poly);
	cv::fillConvexPoly(img, ROI_poly, CV_RGB(0, 0, 255));
	// imshow("temp", img);
	// cv::waitKey(0);
}