#include "cost.h"
#include <algorithm>
using namespace std;

/*
 * get corresponding point in the other view for a given point
 */
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


double disparity(Point2d p, plane f){
	Vec3d params = f.get_params();
	double disp = params[0] * p.x + params[1] * p.y + params[2]; //ax + by + c
	if(disp < 0)
		return 0;
	else if(disp > MAX_DISP)
		return disp - 1;
	return disp;
}


double color_dif(Point2d p, Point2d q, Mat imagep, Mat imageq){
	Vec3d p_bgr = imagep.at<uchar>(p.y, p.x);
	Vec3d q_bgr = imageq.at<uchar>(q.y, q.x);
	return (p_bgr.val[0] - q_bgr.val[0]) + (p_bgr.val[1] - q_bgr.val[1]) + (p_bgr.val[2] - q_bgr.val[2]);
}

double gray_dif(Point2d p, Point2d q, Mat imagep, Mat imageq){
	return imagep.at<uchar>(p.y, p.x) - imageq.at<uchar>(q.y, q.x);
}

double same_plane_liklihood(Mat img, Point2d p, Point2d q){
	double c_diff =  color_dif(p, q, img, img);//(p_bgr[0] - q_bgr[0]) + (p_bgr[1] - q_bgr[1]) + (p_bgr[2] - q_bgr[2]);
	return exp(-abs(c_diff)/GAMMA);
}

double pixel_dissimilarity(Point2d q, Point2d q_prime, Mat img){
	double c_diff = color_dif(q, q_prime, img, img);
	double g_diff = gray_dif(q, q_prime, img, img);
	return (1 - ALPHA) * min(abs(c_diff), TAO_COLOR) + ALPHA * min(abs(g_diff), TAO_GRAY);
}

bool is_valid_pixel(int i, int j, int size){
	if (i > 0 && i < size &&  j < 0 &&  j > size)
		return true;
	return false;
}

double matching_cost(Mat img, int view, Point2d p, plane f){
	double cost = 0;
	int size = img.rows;
	for(int i = -(WINDOW_SIZE - 1)/2; i < (WINDOW_SIZE + 1) /2; i ++){
		for(int j = -(WINDOW_SIZE - 1)/2; j < (WINDOW_SIZE + 1) /2; j ++){
			if(is_valid_pixel(i, j, size)){
				Point2d q(j, i);
				double w = same_plane_liklihood(img, p, q);
				Point2d q_prime(get_corresponding(q.x, disparity(q, f), view, img.rows), q.y);
				double ro = pixel_dissimilarity(q, q_prime, img);
				cost += w * ro;
			}
		}
	}
	return cost;
}
