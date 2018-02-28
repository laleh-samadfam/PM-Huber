#include "utils.h"
#include <stdlib.h>


//plane class methods
void plane :: update_plane(Vec3d pn, double pz){
	n = pn;
	z = pz;
	plane::update_pramas();
}
void plane :: update_pramas(){
	params[0] = - (n[0]/n[2]); //a
	params[1] = - (n[1]/n[2]); //b
	params[2] = (n[0] * p.x + n[1] * p.y + n[2] * z) / z;
}

Vec3d plane :: get_params(){
	return params;
}


double random_generator(double min_range, double max_range){
		double f = (double)rand() / RAND_MAX;
	    return min_range + f * (max_range - min_range);
}
