/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		Stripe class definition
*/
#include <opencv2/opencv.hpp>
#include "Segment.h"
#include "globals.h"

#ifndef STRIPE_H
#define STRIPE_H


class Stripe {
public:
	Segment up_seg;
	Segment down_seg;

	cv::Point2f get_centroid();
	float get_vertical_width();
	float get_L();
	Stripe(Segment& seg1, Segment& seg2);
};

bool check_color(const Segment& seg1, const Segment& seg2, const cv::Mat& img);

float compute_connectivity(Stripe& stripe_up, Stripe& stripe_down);

void shade_area(Stripe stripe, cv::Mat img);
#endif