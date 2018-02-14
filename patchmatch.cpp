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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cost.h"
#include "utils.h"
#include "patchmatch.h"

#define IMAGE_ROW_SIZE 100
#define iMAGE_COLUMN_SIZE 100


using namespace std;
using namespace cv;


int get_corresponding(int x, double disp, int view, int width){
	int cor;
	if(view == 0){
		cor = x + ceil(disp);
	}else{
		cor = x - ceil(disp);
	}
	if(cor > width){
		cor = cor - width;
	}else if(cor < 0){
		cor = cor + width;
	}
	return cor;
}
void patchmatch_pp(){}

void spatial_propagate(Mat image, plane ** planes, int iter){
	if(iter == 0){ //even iterations
		//starting second row and column
		for(int i = 1; i < image.cols; i++){
			for(int j = 1; j < image.rows; j++){
				Point2d p;
				p.x = i; p.y = j;
				double lcost = matching_cost(p, planes[i - 1][j]);
				double ucost = matching_cost(p, planes[i][j - 1]);
				double cost = matching_cost(p, planes[i][j]);
				if(lcost < ucost){
					if(lcost < cost){
						//propagate left plane
						planes[i][j].updateplane(planes[i - 1][j].n, planes[i - 1][j].z);
					}
				}else if(ucost < cost){
					//propagate upper plane
					planes[i][j].updateplane(planes[i][j - 1].n, planes[i][j-1].z);
				}
			}
		}
		//first row
		for(int i = 1; i < image.cols; i++){
			Point2d p;
			p.x = i; p.y = 0;
			double lcost = matching_cost(p, planes[i - 1][0]);
			double cost = matching_cost(p, planes[i][0]);
			if(lcost < cost)
				planes[i][0].updateplane(planes[i - 1][0].n, planes[i - 1][0].z);
		}
		//first col
				for(int j = 1; j < image.rows; j++){
					Point2d p;
					p.x = 0; p.y = j;
					double ucost = matching_cost(p, planes[0][j - 1]);
					double cost = matching_cost(p, planes[0][j]);
					if(ucost < cost)
						planes[0][j].updateplane(planes[0][j - 1].n, planes[0][j - 1].z);
				}
	}
	else if(iter == 1){ //odd iterations
			//starting second row and column from the end
			for(int i = image.cols - 2; i < -1; i--){
				for(int j = image.rows -2; j < -1; j--){
					Point2d p;
					p.x = i; p.y = j;
					double rcost = matching_cost(p, planes[i + 1][j]);
					double lcost = matching_cost(p, planes[i][j + 1]);
					double cost = matching_cost(p, planes[i][j]);
					if(rcost < lcost){
						if(rcost < cost){
							//propagate right plane
							planes[i][j].updateplane(planes[i + 1][j].n, planes[i + 1][j].z);
						}
					}else if(lcost < cost){
						//propagate lower plane
						planes[i][j].updateplane(planes[i][j + 1].n, planes[i][j + 1].z);
					}
				}
			}
			//last row
			for(int i = image.cols - 1; i < -1; i--){
				Point2d p;
				p.x = i; p.y = 0;
				double rcost = matching_cost(p, planes[i + 1][0]);
				double cost = matching_cost(p, planes[i][0]);
				if(rcost < cost)
					planes[i][0].updateplane(planes[i + 1][0].n, planes[i + 1][0].z);
			}
			//last col
					for(int j = image.rows; j < -1; j--){
						Point2d p;
						p.x = 0; p.y = j;
						double lcost = matching_cost(p, planes[0][j + 1]);
						double cost = matching_cost(p, planes[0][j]);
						if(lcost < cost)
							planes[0][j].updateplane(planes[0][j + 1].n, planes[0][j + 1].z);
					}
		}

}

void view_propagate(Mat image, plane ** pm[], int view, int iter){
	if(iter == 0){ //even iterations
		//iterate over the other view
		for(int i = 0; i < image.cols; i++){
			for(int j = 0; j < image.rows; j++){
				plane other_view = pm[(view + 1)%2][i][j]; //plane map of the other view
				double disp = disparity(Point2d(j, i), other_view);
				int cor = get_corresponding(j, disp, (view +1) % 2, image.cols);//get corresponding pixel in the reference view
				double ref_cost = matching_cost(Point2d(cor,i), other_view);
				double other_cost = matching_cost(Point2d(cor, i), pm[view][cor][j]);
				if(other_cost < ref_cost){
					pm[view][i][j].updateplane(other_view.n, other_view.z);
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
				double ref_cost = matching_cost(Point2d(cor,i), other_view);
				double other_cost = matching_cost(Point2d(cor, i), pm[view][cor][j]);
				if(other_cost < ref_cost){
					pm[view][i][j].updateplane(other_view.n, other_view.z);
				}
			}
		}
	}

}

void refine_plane(int iter){

}

void patchmatch_iter(Mat img, Mat disp, plane ** pm[], int view, int iter){
	plane ** planes = pm[view];
	spatial_propagate(img, planes, iter);
	view_propagate(img, pm, view, iter);
	refine_plane(iter);
}

void initial_plane(Mat image, plane ** planes ){

	for(int i = 0; i < image.rows ; i++){
		for(int j = 0; j < image.cols; j ++){
			Vec3d n;
			double z;
			n[0] = random_generator(MIN_N, MAX_N);
			n[1] = random_generator(MIN_N, MAX_N);
			n[2] = random_generator(MIN_N, MAX_N);
			z = random_generator(MIN_Z, MAX_Z);
			planes[i][j] = plane(z, n);
		}
	}
}

void patchmatch(Mat r_img, Mat l_img, dispmap dm, plane ** planes[], int iter){

	initial_plane(l_img, planes[0]);
	initial_plane(r_img, planes[1]);

	for(int i = 0; i < iter; i++){
		patchmatch_iter(l_img, dm.l_disp, planes, 0, iter % 2);
		patchmatch_iter(r_img, dm.r_disp, planes, 1, iter % 2);
	}

	patchmatch_pp();
}

int main (int argc, char *argv[]){

	Mat r_img, l_img;
	int iter = 3;
	int width, height;

	if (argv != 3){
		cout << "You should provide two images as input";
		return -1;
	}

	r_img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	l_img = imread(argv[2], CV_LOAD_IMAGE_COLOR);

	height = r_img.rows;
	width = r_img.cols;

	dispmap dm;
	dm.l_disp = Mat::zeros(height, width, CV_8UC1);
	dm.r_disp = Mat::zeros(height, width, CV_8UC1);

	plane ** pm[2]; //plane maps, pm[0] is for left view and pm[1] is for left view
	//planemap pm;
	patchmatch(r_img, l_img, dm, pm, iter);

}
