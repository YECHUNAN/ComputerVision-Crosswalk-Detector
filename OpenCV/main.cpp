/*
	Author:			Chunan Ye
	Created Date:	4/13/2018
	Description:	An implementation of the zebra-crosswalk segmentation algorithm
	This file:		the main program
*/
#include "opencv2/opencv.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>
#include <cassert>
#include "utils.h"
#include "Rectangle.h"
#include "Segment.h"
#include "Stripe.h"

using namespace std;
using namespace cv;

int main() {
	Mat orig_img = imread("zebra7.jpg");
	Mat grey_img;
	Mat img;
	cvtColor(orig_img, grey_img, CV_BGR2GRAY);
	// white pixel 255; black pixel 0

	// resize grey scale image to fixed size 300x400
	// ------400------
	// 300
	// |
	resize(grey_img, img, cvSize(N, M));

	// Step 1: Image blurring
	Mat dst;
	GaussianBlur(img, dst, Size(3, 3), E / S);
	addWeighted(dst, 1.5, img, -0.5, 0, img);
	img.convertTo(img, CV_32F);
	// debug: display blurred image
	// imshow("blurred", img);
	// waitKey(0);
	
	// Step 2: Gradient computation	+ edge points
	// input: img
	Mat Dx_mask(Size(2, 2), CV_32FC1);
	Dx_mask.at<float>(0, 0) = -1.0;
	Dx_mask.at<float>(1, 0) = -1.0;
	Dx_mask.at<float>(0, 1) = 1.0;
	Dx_mask.at<float>(1, 1) = 1.0;
	Mat Dy_mask(Size(2, 2), CV_32FC1);
	Dy_mask.at<float>(0, 0) = -1.0;
	Dy_mask.at<float>(1, 0) = 1.0;
	Dy_mask.at<float>(0, 1) = -1.0;
	Dy_mask.at<float>(1, 1) = 1.0;

	Mat gx;
	filter2D(img, gx, -1, Dx_mask, Point(-1, -1), 0, BORDER_DEFAULT);
	Mat gy;
	filter2D(img, gy, -1, Dy_mask, Point(-1, -1), 0, BORDER_DEFAULT);
	
	// debug: display image gradients
	Mat gx_abs;
	convertScaleAbs(gx, gx_abs, 1, 0);
	// imshow("gx", gx_abs);
	// waitKey(0);
	Mat gy_abs;
	convertScaleAbs(gy, gy_abs, 1, 0);
	cvtColor(gy_abs, gy_abs, CV_GRAY2BGR);
	// imshow("gy", gy_abs);
	// waitKey(0);

	// input: gx, gy
	Mat G(Size(N, M), CV_32F);
	Mat angle(Size(N, M), CV_32F);
	for (int r = 0; r < M; ++r) {
		for (int c = 0; c < N; ++c) {
			G.at<float>(r, c) = sqrtf(gx.at<float>(r, c)*gx.at<float>(r, c) 
				+ gy.at<float>(r, c)*gy.at<float>(r, c));
			angle.at<float>(r, c) = atan2(-gy.at<float>(r, c), gx.at<float>(r, c));
		}
	}

	// Step 3: Gradient pseudo ordering (and thresholding)	
	int num_bins = 1024;
	int max_num_points = 10240;
	double min_G, max_G;
	minMaxLoc(G, &min_G, &max_G);
	float max_gradient = (float)max_G;

	// coord x num_points x num_buckets
	vector<vector<vector<int>>> bins(2, vector<vector<int>>(max_num_points, vector<int>(num_bins,0)));
	vector<int> bin_ends(num_bins, 0);
	vector<float> bin_centers(num_bins, 0.0);
	bin_centers[0] = max_gradient;
	float step_size = max_gradient / num_bins;
	for (int i = 1; i < num_bins; ++i) {
		bin_centers[i] = bin_centers[i - 1] - step_size;
	}

	for (int r = 1; r < M - 1; ++r) {
		for (int c = 1; c < N - 1; ++c) {
			if (G.at<float>(r, c) > thresh) {
				int min_id = -1;
				float min_dist = -1;
				for (int i = 0; i < bin_centers.size(); ++i) {
					float dist = abs(bin_centers[i] - G.at<float>(r, c));
					if (min_id < 0 || dist < min_dist) {
						min_id = i;
						min_dist = dist;
					}
				}
				bins[0][bin_ends[min_id]][min_id] = r;
				bins[1][bin_ends[min_id]][min_id] = c;
				bin_ends[min_id] = bin_ends[min_id] + 1;
			}
		}
	}

	// Step 4: Region grow
	vector<vector<vector<int>>> output_regions;
	vector<Rectangle> output_rectangles;

	vector<vector<int>> status(M, vector<int>(N, USED));
	for (int r = 1; r<=M-2; ++r) {
		for (int c = 1; c <= N - 2; ++c) {
			if (G.at<float>(r, c) > thresh && abs(gy.at<float>(r, c)) > thresh / 2.0) {
				status[r][c] = NOT_USED;
			}
		}
	}
	
	for (int iter = 0; iter < num_bins; ++iter) {
		for (int i = 0; i < bin_ends[iter]; ++i) {
			int r = bins[0][i][iter];
			int c = bins[1][i][iter];
			if (status[r][c] == NOT_USED) {
				vector<vector<int>> region = region_grow(angle, r, c, status);
				if (region[0].size() < 15) {
					continue;
				}
				Rectangle rec;
				rec.rectangle_self(region, G, gy);
				if (abs(abs(rec.angle) - CV_PI / 2) > CV_PI / 8) {
					continue;
				}
				output_regions.push_back(region);
				output_rectangles.push_back(rec);
			}
		}
	}

	// Step 5: Grouping line segments
	vector<Segment> output_segments;
	for (int i = 0; i < output_rectangles.size(); ++i) {
		Segment s;
		s.segment_self(output_rectangles[i]);
		output_segments.push_back(s);

		// draw the line segments
		// green means up; red means bottom
		/*float y_left = (s.rho - s.xmin*cos(s.theta)) / sin(s.theta);
		float y_right = (s.rho - s.xmax*cos(s.theta)) / sin(s.theta);
		if (s.polarity == UP) {
			line(gy_abs, Point(s.xmin, M - y_left), Point(s.xmax, M - y_right), Scalar(0, 255, 0));
		} else {
			line(gy_abs, Point(s.xmin, M - y_left), Point(s.xmax, M - y_right), Scalar(0, 0, 255));
		}*/
	}
	// line(gy_abs, Point(50, 50), Point(200, 200), Scalar(0, 0, 255));
	//imshow("gy", gy_abs);
	//waitKey(0);

	vector<int> segment_usage(output_segments.size(), NOT_USED);
	vector<Segment> combined_segments;
	vector<float> xdiff(output_segments.size(), 0);
	for (int i = 0; i < output_segments.size(); ++i) {
		xdiff[i] = output_segments[i].xmax - output_segments[i].xmin;
	}
	vector<int> id_xdiff(xdiff.size(), 0);
	for (int i = 0; i < id_xdiff.size(); ++i) {
		id_xdiff[i] = i;
	}
	sort(id_xdiff.begin(), id_xdiff.end(), [&](int a, int b) {return xdiff[a] < xdiff[b]; });
	
	vector<vector<int>> combinations;

	for (int ii = 0; ii < output_segments.size(); ++ii) {
		int i = id_xdiff[ii];
		if (segment_usage[i] == USED) {
			continue;
		}
		vector<pair<Segment, int>> wait_list(1, pair<Segment,int>(output_segments[i],i));
		segment_usage[i] = USED;
		for (int jj = ii + 1; jj < output_segments.size(); ++jj) {
			int j = id_xdiff[jj];
			if (segment_usage[j] == USED) {
continue;
			}
			if (is_same_segment(output_segments[i], output_segments[j]) == true) {
				wait_list.push_back(pair<Segment, int>(output_segments[j], j));
				segment_usage[j] = USED;
			}
		}

		sort(wait_list.begin(), wait_list.end(), [&](pair<Segment, int>& a, pair<Segment, int>& b) { return a.first.xmin < b.first.xmin; });
		vector<int> cur_comb;

		float cur_xmax = 0.0;
		float tolerance = 14.0;
		cur_comb.push_back(wait_list[0].second);
		cur_xmax = wait_list[0].first.xmax;
		for (int j = 1; j < wait_list.size(); ++j) {
			if (cur_xmax + tolerance > wait_list[j].first.xmin) {
				cur_comb.push_back(wait_list[j].second);
				cur_xmax = wait_list[j].first.xmax;
			}
			else {
				combinations.push_back(cur_comb);
				cur_comb.clear();
				cur_comb.push_back(wait_list[j].second);
				cur_xmax = wait_list[j].first.xmax;
			}
		}
		combinations.push_back(cur_comb);
	}

	// Combine the segments to aggregate segments
	vector<Segment> up_segments;
	vector<Segment> down_segments;

	for (int i = 0; i < combinations.size(); ++i) {
		int polar = output_segments[combinations[i][0]].polarity;
		float xmin = output_segments[combinations[i][0]].xmin;
		float xmax = output_segments[combinations[i][0]].xmax;
		float rho = output_segments[combinations[i][0]].rho, theta = output_segments[combinations[i][0]].theta;
		float x_beg = xmin;
		float y_beg = (rho - x_beg*cos(theta)) / sin(theta);
		float x_end = x_beg, y_end = y_beg;
		for (int j = 1; j < combinations[i].size(); ++j) {
			int idx = combinations[i][j];
			float new_x_min = output_segments[idx].xmin;
			float new_x_max = output_segments[idx].xmax;
			if (new_x_min < xmin) {
				xmin = new_x_min;
			}
			if (new_x_max > xmax) {
				xmax = new_x_max;
			}
			rho = output_segments[idx].rho;
			theta = output_segments[idx].theta;
			float x_temp = new_x_max;
			if (new_x_max > x_end) {
				y_end = (rho - x_temp* cos(theta)) / sin(theta);
				x_end = new_x_max;
			}

			// use simple averaging method: due to unresolvable opencv crash issue of Houghlines(...)
		}
		if (xmax - xmin < 20) {
			continue;
		}
		theta = atan((x_beg - x_end) / (y_end - y_beg));
		rho = x_beg * cos(theta) + y_beg * sin(theta);
		Segment s;
		s.segment_self(rho, theta, xmin, xmax, polar);
		if (polar == UP) {
			up_segments.push_back(s);
		}
		else {
			down_segments.push_back(s);
		}
		// draw the merged line segments
		// green means up; red means bottom
		/*float y_left = (s.rho - s.xmin*cos(s.theta)) / sin(s.theta);
		float y_right = (s.rho - s.xmax*cos(s.theta)) / sin(s.theta);
		if (s.polarity == UP) {
			line(dst, Point(s.xmin, M - y_left), Point(s.xmax, M - y_right), Scalar(0, 255, 0));
		} else {
			line(dst, Point(s.xmin, M - y_left), Point(s.xmax, M - y_right), Scalar(0, 0, 255));
		}*/
	}

	// cvtColor(dst, dst, CV_GRAY2BGR);
	// imshow("edge", gy_abs);
	// imshow("img", dst);
	// waitKey(0);

	// Step 6: Match line segments
	vector<vector<float>> score(up_segments.size(), vector<float>(down_segments.size(), 0.0));
	// up -> down

	for (int i = 0; i < up_segments.size(); ++i) {
		for (int j = 0; j < down_segments.size(); ++j) {
			score[i][j] = proximity(up_segments[i], down_segments[j]);
		}
	}

	// Step 7: Construct stripelets using matched line segments
	Mat img_output;
	img.convertTo(img_output, CV_8U);
	cvtColor(img_output, img_output, CV_GRAY2BGR);
	vector<Stripe> stripes;
	int count = 0;
	for (int i = 0; i < up_segments.size(); ++i) {
		for (int j = 0; j < down_segments.size(); ++j) {
			if (score[i][j] > 0) {
				if (check_color(up_segments[i], down_segments[j], img) ) {
					stripes.emplace_back(up_segments[i], down_segments[j]);
					// debug
					// shade_area(stripes.back(), img_output);
				}
			}
		}
	}
	// debug
	// imshow("all stripes", img_output);
	// waitKey(0);

	// Step 8: Compute connectivity of the stripelets
	vector<vector<float>> connectivity_matrix(stripes.size(), vector<float>(stripes.size(), 0.0));
	for (int i = 0; i < stripes.size(); ++i) {
		for (int j = i + 1; j < stripes.size(); ++j) {
			Point2f c_1 = stripes[i].get_centroid();
			Point2f c_2 = stripes[i].get_centroid();
			if (c_1.y < c_2.y) {
				connectivity_matrix[i][j] = compute_connectivity(stripes[j], stripes[i]);
				connectivity_matrix[j][i] = connectivity_matrix[i][j];
			}
			else {
				connectivity_matrix[i][j] = compute_connectivity(stripes[i], stripes[j]);
				connectivity_matrix[j][i] = connectivity_matrix[i][j];
			}
		}
	}

	// Step 9: Compute envelope

	vector<pair<int, pair<float,float>>> stripe_points;
	for (int i = 0; i < stripes.size(); ++i) {
		float y = stripes[i].get_centroid().y;
		float x = stripes[i].get_centroid().x;
		stripe_points.push_back(pair<int, pair<float, float>>(i, pair<float, float>(y, sqrt(y*y + x*x))));
	}
	// id, y, w
	sort(stripe_points.begin(), stripe_points.end(), 
		[&](pair<int, pair<float, float>> p1, pair<int, pair<float, float>> p2) { return p1.second.first < p2.second.first; });
	float y0 = stripes[stripe_points[0].first].get_centroid().y;
	float w0 = stripes[stripe_points[0].first].get_vertical_width();
	sort(stripe_points.begin(), stripe_points.end(),
		[&](pair<int, pair<float, float>> p1, pair<int, pair<float, float>> p2) { return p1.second.second < p2.second.second; });

	int num = 7;
	float k = 0.0;
	float denom = 0.0;
	for (int i = 0; i < stripe_points.size(); ++i) {
		float yi = stripes[stripe_points[i].first].get_centroid().y;
		float wi = stripes[stripe_points[i].first].get_vertical_width();
		if (wi > w0 || y0 == yi) {
			continue;
		}
		denom = denom + (y0 - yi)*(y0 - yi);
		k = k + (wi - w0)*(yi - y0);
		num = num - 1;
		if (num == 0) {
			break;
		}
	}
	k = k / denom;
	float b = w0 - k*y0;
	// the envelope line is characterized by w = k*y + b

	// Step 10: Compute unary potential and binary potential
	float binary_boost = 20.0f;;
	float unary_boost = 30.0f;
	vector<vector<float>> binary_potential = connectivity_matrix;
	for (int i = 0; i < binary_potential.size(); ++i) {
		for (int j = 0; j < binary_potential[0].size(); ++j) {
			if (connectivity_matrix[i][j] > 0) {
				float temp = binary_potential[i][j];
				binary_potential[i][j] = 10.0 / 3 * exp(-10.0*temp) * binary_boost;
			}
		}
	}

	vector<float> unary_potential(stripes.size(), 0);
	for (int i = 0; i < unary_potential.size(); ++i) {
		float E = abs(stripes[i].get_centroid().y * k - stripes[i].get_vertical_width() + b) / (sqrt(k*k + 1));
		float w_hat = k*stripes[i].get_centroid().y + b;
		if (w_hat > 0) {
			float temp = max(0.0f, stripes[i].get_L()*(1 - E / w_hat));
			unary_potential[i] = 0.1 * min(1.0f, temp) * unary_boost;
		}
		//cv::putText(img_output,
		//	to_string(stripes[i].get_vertical_width()),
		//	Point2f(stripes[i].get_centroid().x, M - stripes[i].get_centroid().y), // Coordinates
		//	cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
		//	0.5, // Scale. 2.0 = 2x bigger
		//	cv::Scalar(0, 255, 255), // Color
		//	0.5 // Thickness
		//);
		cv::putText(img_output,
			to_string(int(unary_potential[i])),
			Point2f(stripes[i].get_centroid().x, M - stripes[i].get_centroid().y), // Coordinates
			cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
			0.5, // Scale. 2.0 = 2x bigger
			cv::Scalar(0, 255, 255), // Color
			0.5 // Thickness
			);
	}
	imshow("stripe centers", img_output);
	waitKey(0);
	// Step 11: Belief propagation: message update
	int num_points = stripes.size();
	vector<vector<float>> old_message_0(num_points, vector<float>(num_points, 1.0f / num_points));
	vector<vector<float>> old_message_1(num_points, vector<float>(num_points, 1.0f / num_points));
	vector<vector<float>> new_message_0(num_points, vector<float>(num_points, 0.0f));
	vector<vector<float>> new_message_1(num_points, vector<float>(num_points, 0.0f));

	for (int iter = 0; iter < 20; ++iter) {
		for (int i = 0; i < num_points; ++i) {
			for (int j = 0; j < num_points; ++j) {
				if (i == j || connectivity_matrix[i][j] == 0) {
					continue;
				}
				// update for xj = 1 , xj = 0
				float update_0 = 1.0;
				float update_1 = 1.0;
				for (int k = 0; k < num_points; ++k) {
					if (k == j || k == i) {
						continue;
					}
					if (connectivity_matrix[k][i] > 0 && old_message_0[k][i] > 0) {
						update_0 = update_0 * old_message_0[k][i];
					}
					if (connectivity_matrix[k][i] > 0 && old_message_1[k][i] > 0) {
						update_1 = update_1 * old_message_1[k][i];
					}
				}
				update_0 = update_0 * 1.0 * 1.0;
				float update_11 = update_1 * binary_potential[i][j] * unary_potential[i];
				new_message_1[i][j] = update_0 + update_11;
				float update_01 = update_1 * unary_potential[i];
				new_message_0[i][j] = update_0 * update_01;
			}
		}
		old_message_0 = new_message_0;
		old_message_1 = new_message_1;
		for (int i = 0; i < num_points; ++i) {
			for (int j = 0; j < num_points; ++j) {
				float ssum = old_message_0[i][j] + old_message_1[i][j];
				if (ssum > 0) {
					old_message_0[i][j] = old_message_0[i][j] / ssum;
					old_message_1[i][j] = old_message_1[i][j] / ssum;
				}
			}
		}
	}

	vector<float> belief_0(num_points, 1.0f);
	vector<float> belief_1(num_points, 0.0f);
	vector<bool> belief(num_points, false);
	for (int i = 0; i < num_points; ++i) {
		for (int k = 0; k < num_points; ++k) {
			if (k == i || connectivity_matrix[k][i] == 0 || old_message_0[k][i] == 0) {
				continue;
			}
			belief_0[i] = belief_0[i] * old_message_0[k][i];
		}
		belief_1[i] = unary_potential[i];
		for (int k = 0; k < num_points; ++k) {
			if (k == i || connectivity_matrix[k][i] == 0 || old_message_1[k][i] == 0) {
				continue;
			}
			belief_1[i] = belief_1[i] * old_message_1[k][i];
		}
		if (belief_1[i] >= belief_0[i]) {
			belief[i] = true;
		}
	}

	// Visualize the output
	for (int i = 0; i < num_points; ++i) {
		if (belief[i] == true) {
			shade_area(stripes[stripe_points[i].first], img_output);
		}
	}
	imshow("final output", img_output);
	waitKey(0);
	return 0;
}