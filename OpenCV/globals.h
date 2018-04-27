/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		all global parameters
*/
#ifndef GLOBALS_H
#define GLOBALS_H

#define USED 1
#define NOT_USED 0
#define M 300
#define N 400
#define UP 1
#define DOWN 0

const float S = 0.8f;
const float E = 0.6f;
const float q = 2.0f;
const float tau = (float)CV_PI / 8.0f;
const float eps_theta = (float)CV_PI / 12.0f;
const float eps_rho_base = 5.0f;
const float eps_dist = 5.0f;
const float thresh = q / sin(tau);
const float G_thresh = thresh * 3.0f;
const float white_threshold = 190.0f;
#endif // !1

