/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		Segment class definition
*/
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Rectangle.h"
#include "utils.h"

#ifndef SEGMENT_H
#define SEGMENT_H

class Segment {
public:
	float rho;
	float theta;
	float xmin;
	float ymin;
	float xmax;
	float ymax;
	int polarity;

	Segment() : rho(0.0), theta(0.0), xmin(0.0), ymin(0.0), xmax(0.0), ymax(0.0) {}

	void segment_self(Rectangle& rec);
	void segment_self(float r, float t, float xmin, float xmax, int polarity);
};

bool is_same_segment(const Segment& seg1, const Segment& seg2);

float proximity(const Segment& seg1, const Segment& seg2);
#endif