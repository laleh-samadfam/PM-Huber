//============================================================================
// Name        : PatchMatchStereo.cpp
// Author      : Laleh
// Version     :
// Copyright   : Your copyright notice
// Description : PatchMach stereo vision
//============================================================================

#include <sys/types.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <unistd.h>
#include <math.h>
#include "cost.h"
#include "utils.h"
#include "patchmatch.h"

//initial random planes
#define MAX_N 1
#define MIN_N 0
#define MAX_Z 1//take a look at these two
#define MIN_Z 0
//#define INFINITY std::numeric_limits<double>::infinity()


using namespace std;
using namespace cv;



/*/
 * ‌mark invalid the disparity for pixels which their disparity has a long distance
 * with their correspondece.
 */
void right_left_check(Mat image, plane ** planes[], double ** dm[]){
	for(int v = 0; v < 2; v ++){
		for(int i = 0; i < image.rows; i++){
			for(int j = 0; j < image.cols; j++){
				int cor = get_corresponding(j, double(dm[v][i][j]), v, image.cols); //second guess
				if(fabs(double(dm[v][i][j]) - double(dm[(v+1)%2][i][cor])) > 1){
					dm[v][i][j] = INFINITY; // invalid
				}
			}
		}
	}
}

int find_valid_right(int row, int location, double ** dm, int width){
	int i = 0;
	while((dm[row][location + i] < INFINITY) || location + i < width) i ++;
	return location + i;
}

int find_valid_left(int row, int location, double ** dm, int width){
	int i = 0;
	while((dm[row][location + i] < INFINITY) || location + i < width) i --;
	return location + i;
}
void fill_invalid_disp(Mat image, plane ** planes[] ,double ** dm[], int height){
	for (int v = 0; v < 2; v ++) {
		for (int i = 0; i < image.rows; i++) {
			for (int j = 0; j < image.cols; j++) {
				if(dm[v][i][j] == INFINITY){
					int l_valid = find_valid_left(i, j, dm[v], height);
					int r_valid = find_valid_right(i, j, dm[v], height);
					double l_cost = matching_cost(image, v, Point2d(l_valid, i), planes [v][i][l_valid]);
					double r_cost = matching_cost(image, v, Point2d(r_valid, i), planes[v][i][r_valid]);
					if(l_cost > r_cost){
						plane p = planes[v][i][r_valid];
						planes[v][i][j].update_plane(p.n, p.z);
						dm[v][i][j] = disparity(Point2d(j, i), p);
					}else{
						plane p = planes[v][i][l_valid];
						planes[v][i][j].update_plane(p.n, p.z);
						dm[v][i][j] = disparity(Point2d(j, i), p);
					}
				}
			}
		}
	}

}

/*
 * post processing step. first do the right/left check to mark invalid the disparities
 * for the pixels witch have a long distance with their correspondence, then fill the
 * invalid disparities.
 */
void patchmatch_pp(Mat image, plane ** planes[], double ** dm[], int height){
	right_left_check(image, planes, dm);
	fill_invalid_disp(image, planes, dm, height);
}

void extract_disparity_map(Mat image, plane ** planes[], double ** dm[]){
	for(int v = 0; v < 2; v ++){
		for(int i = 0; i < image.cols; i++ ){
			for(int j = 0; j < image.rows; j++){
				dm[v][i][j] = disparity(Point2d(j, i), planes[v][i][j]);
			}
		}
	}
}



void update_cost(int i, int j, double ** costm, double new_cost){
	costm[i][j] = new_cost;
}

/*
 * see if neighbor pixels have a better plane. check the upper, left pixels for even
 * iterations and lower, right pixels in odd iterations.
 */
void spatial_propagate(Mat image, double ** cost, plane ** pm[], int view, int iter){
	plane ** planes = pm[view];
	if(iter == 0){ //even iterations
		//starting second row and column
		for(int i = 1; i < image.cols; i++){
			for(int j = 1; j < image.rows; j++){
				double lcost = matching_cost(image,view, Point2d(j, i), planes[i - 1][j]);
				double ucost = matching_cost(image, view, Point2d(j, i), planes[i][j - 1]);
				if(lcost < ucost){
					if(lcost < cost[i][j]){
						//propagate left plane
						planes[i][j].update_plane(planes[i - 1][j].n, planes[i - 1][j].z);
						update_cost(i, j, cost, lcost);
					}
				}else if(ucost < cost[i][j]){
					//propagate upper plane
					planes[i][j].update_plane(planes[i][j - 1].n, planes[i][j-1].z);
					update_cost(i, j, cost, ucost);
				}
			}
		}
		//first row
		for(int i = 1; i < image.cols; i++){
			double lcost = matching_cost(image, view, Point2d(0, i), planes[i - 1][0]);
			if(lcost < cost[i][0]){
				planes[i][0].update_plane(planes[i - 1][0].n, planes[i - 1][0].z);
				update_cost(i, 0, cost, lcost);
			}

		}
		//first col
				for(int j = 1; j < image.rows; j++){
					double ucost = matching_cost(image, view, Point2d(j,0), planes[0][j - 1]);
					if(ucost < cost[0][j]){
						planes[0][j].update_plane(planes[0][j - 1].n, planes[0][j - 1].z);
						update_cost(0, j, cost, ucost);
					}
				}
	}
	else if(iter == 1){ //odd iterations
			//starting second row and column from the end
			for(int i = image.cols - 2; i < -1; i--){
				for(int j = image.rows -2; j < -1; j--){
					double rcost = matching_cost(image, view, Point2d(j, i), planes[i + 1][j]);
					double lcost = matching_cost(image, view, Point2d(j, i), planes[i][j + 1]);
					if(rcost < lcost){
						if(rcost < cost[i][j]){
							//propagate right plane
							planes[i][j].update_plane(planes[i + 1][j].n, planes[i + 1][j].z);
							update_cost(i, j, cost, rcost);

						}
					}else if(lcost < cost[i][j]){
						//propagate lower plane
						planes[i][j].update_plane(planes[i][j + 1].n, planes[i][j + 1].z);
						update_cost(i, j, cost, lcost);
					}
				}
			}
			//last row
			for(int i = image.cols - 1; i < -1; i--){
				double rcost = matching_cost(image, view, Point2d(0, i), planes[i + 1][0]);
				if(rcost < cost[i][0])
					planes[i][0].update_plane(planes[i + 1][0].n, planes[i + 1][0].z);
					update_cost(i, 0, cost, rcost);
			}
			//last col
					for(int j = image.rows; j < -1; j--){
						double lcost = matching_cost(image, view, Point2d(j, 0), planes[0][j + 1]);
						if(lcost < cost[0][j])
							planes[0][j].update_plane(planes[0][j + 1].n, planes[0][j + 1].z);
							update_cost(0, j, cost, lcost);
					}
		}

}

/*
 * see if the corresponding pixel in the other view has a better plane.
 * iterate over the other view to cover every corresponding pixel.
 */
void view_propagate(Mat image, double ** cost, plane ** pm[], int view, int iter){
	if(iter == 0){ //even iterations
		//iterate over the other view
		for(int i = 0; i < image.cols; i++){
			for(int j = 0; j < image.rows; j++){
				plane other_view = pm[(view + 1)%2][i][j]; //plane map of the other view
				double disp = disparity(Point2d(j, i), other_view);
				int cor = get_corresponding(j, disp, (view +1) % 2, image.cols);//get corresponding pixel in the reference view
				double other_cost = matching_cost(image, view, Point2d(cor, i), pm[view][cor][j]);
				if(other_cost < cost[i][cor]){
					pm[view][i][j].update_plane(other_view.n, other_view.z);
					update_cost(i, j, cost, other_cost);
				}
			}
		}
	}
	else{ //odd iterations
		//iterate over the other view
		for(int i = image.cols; i < -1; i--){
			for(int j = image.rows; j < -1; j--){
				plane other_view = pm[(view + 1)%2][i][j]; //plane map of the other view
				double disp = disparity(Point2d(j, i), other_view);
				int cor = get_corresponding(j, disp, (view +1) % 2, image.cols);//get corresponding pixel in the reference view
				//double ref_cost = matching_cost(Point2d(cor,i), other_view);
				double other_cost = matching_cost(image,view, Point2d(cor, i), pm[view][cor][j]);
				if(other_cost < cost[i][cor]){
					pm[view][i][j].update_plane(other_view.n, other_view.z);
					update_cost(i, j, cost, other_cost);
				}
			}
		}
	}

}


/*
 * refining plane parameters after view and spatial propagations using random values in
 * a certain range.
 */
void refine_plane(Mat image, double ** cost, plane ** pm[], int view){
	plane ** planes = pm[view];
	double z_max = MAX_DISP/2;
	double n_max = MAX_N;
	double z_rand;
	Vec3d n_rand;
	for(int i = 0; i < image.cols; i++){
		for(int j = 0; j < image.rows; j++){
			while(z_max > 0.1){
				z_rand = planes[i][j].z + random_generator(-1 * z_max, z_max);
				n_rand[0] = planes[i][j].n[0] +random_generator(-1 * n_max, n_max);
				n_rand[1] = planes[i][j].n[1] +random_generator(-1 * n_max, n_max);
				n_rand[2] = planes[i][j].n[2] +random_generator(-1 * n_max, n_max);
				plane rand_plane = plane(z_rand, n_rand);
				double rand_cost = matching_cost(image,view, Point2d(j, i), rand_plane);
				if (cost[i][j] > rand_cost){
					planes[i][j].n = n_rand;
					planes[i][j].z = z_rand;
				}
				z_max = z_max/2;
				n_max = n_max/2;
			}
		}
	}
}

/*
 * each iteration of patchmatch algorithm
 */
void patchmatch_iter(Mat img, double ** cost, plane ** pm[], int view, int iter){
	spatial_propagate(img, cost, pm , view, iter);
	view_propagate(img, cost, pm, view, iter);
	refine_plane(img, cost, pm, view);
}

/*
 * randomly initial planes for each pixel in both views.
 */
void initial_plane(Mat image, plane ** pm[], int view, double ** cost){

	for(int i = 0; i < image.rows ; i++){
		for(int j = 0; j < image.cols; j ++){
			Vec3d n;
			double z;
			n[0] = random_generator(MIN_N, MAX_N);
			n[1] = random_generator(MIN_N, MAX_N);
			n[2] = random_generator(MIN_N, MAX_N);
			z = random_generator(MIN_Z, MAX_Z);
			pm[view][i][j] = plane(z, n);
			cost[i][j] = matching_cost(image, view, Point2d(j, i), pm[view][i][j]);
		}
	}
}

void patchmatch(Mat r_img, Mat l_img, costmap cm, plane ** planes[], double ** dm[], int iter, int height){

	initial_plane(l_img, planes, 0, cm.l_cost);
	initial_plane(r_img, planes, 1, cm.r_cost);

	for(int i = 0; i < iter; i++){
		patchmatch_iter(l_img, cm.l_cost, planes, 0, iter % 2);
		patchmatch_iter(r_img, cm.r_cost, planes, 1, iter % 2);
	}
	extract_disparity_map(r_img, planes, dm);
	patchmatch_pp(r_img, planes, dm, height);
}

int main (int argc, char *argv[]){

	Mat r_img, l_img;
	int iter = 3;
	int width;

	if (argc != 3){
		cout << "You should provide two images as input";
	//	return -1;
	}
	cout << "here";
	r_img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	l_img = imread(argv[2], CV_LOAD_IMAGE_COLOR);

	width = r_img.rows;
	//width = r_img.cols;

	costmap cm;
	//cm.l_cost = Mat::zeros(height, width, CV_8UC1);
	//cm.l_cost = Mat::zeros(height, width, CV_8UC1);

	plane ** pm[2]; //plane maps, pm[0] is for left view and pm[1] is for left view
	double ** dm[2]; //disparity map (output)
	//initial_dm(); TODO
	//planemap pm;
	patchmatch(r_img, l_img, cm, pm, dm, iter, width);

}
