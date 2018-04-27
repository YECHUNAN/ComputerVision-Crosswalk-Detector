/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		class Rectangle declaration
*/
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include "globals.h"

#ifndef RECTANGLE_H
#define RECTANGLE_H


class Rectangle {
public:
	float center_x;
	float center_y;
	float angle;
	float W;
	float L;
	int polarity;

	Rectangle(): center_x(0.0), center_y(0.0), angle(0.0), W(0.0), L(0.0), polarity(0) {}
	void rectangle_self(std::vector<std::vector<int>>& region, cv::Mat& G, cv::Mat& gy);
};

#endif