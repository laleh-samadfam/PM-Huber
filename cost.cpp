#include "cost.h"
#include "utils.h"
using namespace std;


double same_plane_liklihood(point p, Plane f){
}


double disparity(Point2d p, plane f){
	Vec3d params = f.getparams();
	double disp = params[0] * p.x + params[1] * p.y + params[2]; //ax + by + c
	if(disp < 0)
		return 0;
	else if(disp > MAX_DISP)
		return disp - 1;
	return disp;
}

double pixel_dissimilarity(point q, point q_prime){

	c_diff = color_dif(q, q_prime);
	g_diff = gray_dif(q, q_prime);
	return (1 - ALPHA) * min(c_diff, TAO_COLOR) + ALPHA * min(g_diff, TAO_GRAY)
}

double matching_cost(Point2d p, plane f){

	for(int i = 0; i <= WINDOW_SIZE; i++){
		q = p.getwindow().getpoint(i);
		double w = same_plane_liklihood();
		point q_prime = correspondence(q, f);
		double ro = pixel_dissimilarity(q, q_prime);

	}
	return //TODO
}
