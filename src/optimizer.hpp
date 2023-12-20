#include <opencv2/core.hpp>
#include <cmath>
using namespace std;
using namespace cv;

//double f1, theta11, theta12, theta13, f2, theta21, theta22, theta23;
namespace bottom_up{
Mat getIntrinsicMatrix(double focal_length);
Mat getRotationMatrix(double theta1, double theta2, double theta3);
}
