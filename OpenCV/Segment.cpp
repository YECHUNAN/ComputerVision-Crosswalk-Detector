/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		Segment class implementation
*/
#include "Segment.h"

void Segment::segment_self(Rectangle& rec) {
	float x = rec.center_x;
	float y = rec.center_y;
	float n_angle = rec.angle;
	float n_x = cos(n_angle);
	float n_y = sin(n_angle);
	float L = rec.L;
	this->rho = (x - 1)*n_x + (y - 1)*n_y;
	if (this->rho > 0) {
		this->theta = n_angle;
	} else {
		this->theta = n_angle + (float)CV_PI;
		this->rho = -this->rho;
	}
	this->xmin = x - abs(L / 2 * sin(this->theta));
	this->xmax = x + abs(L / 2 * sin(this->theta));
	this->polarity = rec.polarity;
	this->ymin = (this->rho - cos(this->theta) * this->xmin) / sin(this->theta);
	this->ymax = (this->rho - cos(this->theta) * this->xmax) / sin(this->theta);
}

bool is_same_segment(const Segment& seg1, const Segment& seg2) {
	if (seg1.polarity != seg2.polarity) {
		return false;
	}
	float d_angle = angle_diff(seg1.theta, seg2.theta);
	if (d_angle > eps_theta) {
		return false;
	}
	float r1 = seg1.rho;
	float t1 = seg1.theta;
	float r2 = seg2.rho;
	float t2 = seg2.theta;

	float x = seg2.xmin;
	float y = seg2.ymin;
	float dist = abs(x*cos(t1) + y*sin(t1) - r1);
	x = seg2.xmax;
	y = seg2.ymax;
	dist = (abs(x*cos(t1) + y*sin(t1) - r1) + dist)/2;

	if (dist > eps_dist) {
		return false;
	}
	return true;
}

void Segment::segment_self(float r, float t, float xmin, float xmax, int polarity) {
	this->rho = r;
	this->theta = t;
	this->xmin = xmin;
	this->xmax = xmax;
	this->polarity = polarity;
	this->ymin = (this->rho - cos(this->theta) * this->xmin) / sin(this->theta);
	this->ymax = (this->rho - cos(this->theta) * this->xmax) / sin(this->theta);
}

float proximity(const Segment& seg1, const Segment& seg2) {
	float x1 = (seg1.xmin + seg1.xmax) / 2;
	float y1 = (seg1.rho - x1 * cos(seg1.theta)) / sin(seg1.theta);
	float x2 = (seg2.xmin + seg2.xmax) / 2;
	float y2 = (seg2.rho - x2 * cos(seg2.theta)) / sin(seg2.theta);
	if (y1 < y2) {
		return 0.0;
	}
	float v_width = abs((x1 - x2)*cos(seg1.theta) + (y1 - y2) * sin(seg1.theta));
	if (v_width < 5 || v_width > 40) {
		return 0.0;
	}

	float c = 2 * CV_PI;
	float dover = std::max(0.0f, float(std::min(seg1.xmax, seg2.xmax) - std::max(seg1.xmin, seg2.xmin)));
	float dnorm = std::min(seg1.xmax - seg1.xmin, seg2.xmax - seg2.xmin);
	float s = dover / dnorm;
	s = s * exp(-abs(seg1.theta - seg2.theta) / c);
	return s;
}