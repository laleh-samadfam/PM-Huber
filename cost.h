#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utils.h"

#define TAO_COLOR 1.0
#define TAO_GRAY 1.0
#define ALPHA 0.5
#define MAX_DISP 1
#define WINDOW_SIZE 35
#define GAMMA 10


using namespace std;
using namespace cv;


int get_corresponding(int x, double disp, int view, int width);

double disparity(Point2d p, plane f);

double matching_cost(Mat img, int view, Point2d p, plane f);

//double dissimilarity(Point3d q, plane f);
