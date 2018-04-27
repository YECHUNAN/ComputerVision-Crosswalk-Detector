/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		class Rectangle member definition
*/
#include "Rectangle.h"
#include <cstdlib>
#include <ctime>
using namespace cv;

void Rectangle::rectangle_self(std::vector<std::vector<int>>& region, cv::Mat& G, cv::Mat& gy) {
	float cx = 0.0;
	float cy = 0.0;
	float norm_term = 0.0;
	float cum_sum = 0.0;
	srand(time(NULL));
	for (int i = 0; i < 5; ++i) {
		int id = (rand() % region[0].size());
		int r = region[0][id];
		int c = region[1][id];
		cum_sum += gy.at<float>(r, c);
	}
	if (cum_sum > 0) {
		this->polarity = UP;
	} else {
		this->polarity = DOWN;
	}
	for (int i = 0; i < region[0].size(); ++i) {
		int r = region[0][i];
		int c = region[1][i];
		int x = c;
		int y = M - r + 1;
		cx += G.at<float>(r, c)*x;
		cy += G.at<float>(r, c)*y;
		norm_term += G.at<float>(r, c);
	}
	cx = cx / norm_term;
	cy = cy / norm_term;
	this->center_x = cx;
	this->center_y = cy;
	float mxx = 0.0;
	float mxy = 0.0;
	float myy = 0.0;
	for (int i = 0; i < region[0].size(); ++i) {
		int r = region[0][i];
		int c = region[1][i];
		int x = c;
		int y = M - r + 1;
		mxx = mxx + G.at<float>(r, c) * ((float)x - cx)*((float)x - cx);
		mxy = mxy + G.at<float>(r, c) * ((float)x - cx)*((float)y - cy);
		myy = myy + G.at<float>(r, c) * ((float)y - cy)*((float)y - cy);
	}
	Mat Mxx_yy(Size(2, 2), CV_32F);
	Mxx_yy.at<float>(0, 0) = mxx;
	Mxx_yy.at<float>(0, 1) = mxy;
	Mxx_yy.at<float>(1, 0) = mxy;
	Mxx_yy.at<float>(1, 1) = myy;
	Mat e, u;
	eigen(Mxx_yy, e, u);
	float u_x = u.at<float>(1, 0);
	float u_y = u.at<float>(1, 1);
	this->angle = atan2(u_y, u_x);

	float n_x = cos(this->angle);
	float n_y = sin(this->angle);
	float n_perp_x = sin(this->angle);
	float n_perp_y = -cos(this->angle);
	float pos_W_max = 0, neg_W_max = 0;
	float pos_L_max = 0, neg_L_max = 0;
	for (int i = 0; i < region[0].size(); ++i) {
		int r = region[0][i];
		int c = region[1][i];
		int x = c;
		int y = M - r + 1;
		float dw = (x - this->center_x)*n_x + (y - this->center_y)*n_y;
		float dl = (x - this->center_x)*n_perp_x + (y - this->center_y)*n_perp_y;
		if (dw > pos_W_max) {
			pos_W_max = dw;
		}
		else if (dw < neg_W_max) {
			neg_W_max = dw;
		}
		if (dl > pos_L_max) {
			pos_L_max = dl;
		}
		else if (dl < neg_L_max) {
			neg_L_max = dl;
		}
	}
	this->W = 2 * max(pos_W_max, -neg_W_max);
	this->L = 2 * max(pos_L_max, -neg_L_max);
}