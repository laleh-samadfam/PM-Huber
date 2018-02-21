using namespace std;
using namespace cv;

#define WINDOW_SIZE 9



class plane {
	public:
		plane(double pz, Vec3d pn): z(pz), n(pn){}
		void updateplane(Vec3d n, double z);
		void updatepramas(Vec3d params);
		Vec3d getparams();

	private:
			double z;
			Vec3d n;
			Vec3d params;

};

struct planemap{
	plane ** l_planes;
	plane ** r_planes;
};

struct costmap{
	Mat l_cost;
	Mat r_cost;
};


class Window{
	Point3d * w;

	public:
		Point2d get_point(int i);
};


double color_dif(Point2d p, Point2d q);

double gray_dif(Point2d p, Point2d q);

double random_generator(double min_range, double max_range);
