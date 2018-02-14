#include "utils.h"

#define TAO_COLOR 1
#define TAO_GRAY 1
#define ALPHA 0.5

using namespace std;


double disparity(Point2d p, plane f);

double matching_cost(Point3d p, plane f);

double dissimilarity(Point3d q, plane f);
