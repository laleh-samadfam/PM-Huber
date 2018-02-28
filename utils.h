#pragma  once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


class plane{

public:

	plane(double pz, Vec3d pn): z(pz), n(pn){}
	void update_plane(Vec3d n, double z);
	void update_pramas();
	Vec3d get_params();


	Point2d p;
	double z;
	Vec3d n;
	Vec3d params; //a, b, c

};

struct planemap{
	plane ** l_planes;
	plane ** r_planes;
};

struct costmap{
	double ** l_cost;
	double ** r_cost;
};

double random_generator(double min_range, double max_range);
