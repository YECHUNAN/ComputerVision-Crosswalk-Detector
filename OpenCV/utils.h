/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		utility function declarations
*/
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "globals.h"


#ifndef UTILS_H
#define UTILS_H


std::vector<std::vector<int>> region_grow(cv::Mat angle, int r, int c, std::vector<std::vector<int>>& status);

bool is_valid_pixel(int r, int c);

float angle_diff(float ang1, float ang2);


#endif