/*
Author:			Chunan Ye
Created Date:	4/13/2018
Description:	An implementation of the zebra-crosswalk segmentation algorithm
This file:		utility function definitions
*/

/*
REGION_GROW
  input: angle - line field angle : MxN ----- angles at each pixel
         seed pixel P(r,c) ----- pixel to grow from in rc
         space
         angle tolerance tau : R ----- scalar hyper-parameter
         status : MxN ----- binary mask
  output : region : 2xVar ----- coordinate of pixels in this region rc space
*/


#include "utils.h"

/*
offset view
1  2  3
4     5
6  7  8
*/
static int offseted_r(int r, int off) {
	if (off == 1 || off == 2 || off == 3) return (r - 1);
	if (off == 6 || off == 7 || off == 8) return (r + 1);
	return r;
}

static int offseted_c(int c, int off) {
	if (off == 1 || off == 8 || off == 7) return (c - 1);
	if (off == 3 || off == 4 || off == 5) return (c + 1);
	return c;
}

std::vector<std::vector<int>> region_grow(cv::Mat angle, int r, int c, std::vector<std::vector<int>>& status) {
	using namespace std;
	vector<vector<int>> region(2, vector<int>());
	vector<vector<int>> added(2, vector<int>());
	added[0].push_back(r);
	added[1].push_back(c);
	status[r][c] = USED;
	float theta_region = angle.at<float>(r, c);
	float Sx = cos(theta_region);
	float Sy = sin(theta_region);

	// BFS for region grow
	bool has_new_pixel = true;
	while (has_new_pixel) {
		vector<vector<int>> new_added(2, vector<int>());
		for (int i = 0; i < added[0].size(); ++i) {
			has_new_pixel = false;
			int p_r = added[0][i];
			int p_c = added[1][i];
			for (int off = 1; off <= 8; ++off) {
				int neighbor_r = offseted_r(p_r, off);
				int neighbor_c = offseted_c(p_c, off);
				if (is_valid_pixel(neighbor_r, neighbor_c) && status[neighbor_r][neighbor_c] == NOT_USED) {
					if (angle_diff(theta_region, angle.at<float>(neighbor_r, neighbor_c)) < tau) {
						region[0].push_back(neighbor_r);
						region[1].push_back(neighbor_c);
						status[neighbor_r][neighbor_c] = USED;
						Sx += cos(angle.at<float>(neighbor_r, neighbor_c));
						Sy += sin(angle.at<float>(neighbor_r,neighbor_c));
						theta_region = atan2(Sy, Sx);

						new_added[0].push_back(neighbor_r);
						new_added[1].push_back(neighbor_c);
						has_new_pixel = true;
					}
				}
			}
		}
		added = new_added;
	}
	// cout << region[0].size() << endl;
	return region;
}

bool is_valid_pixel(int r, int c) {
	if (r < 0 || r >= M || c < 0 || c >= N) return false;
	return true;
}

float angle_diff(float ang1, float ang2) {
	float diff = abs(ang1 - ang2);
	if (diff > CV_PI) {
		diff = 2 * float(CV_PI) - diff;
	}
	return diff;
}

