#include <opencv2/core.hpp>
#include <cmath>
using namespace std;
using namespace cv;

double f1, theta11, theta12, theta13, f2, theta21, theta22, theta23;
namespace bottom_up{

Mat getIntrinsicMatrix(double focal_length){
    return Mat_<double>(3,3) << focal_length,0,0, 0,focal_length,0, 0,0,1;
}
Mat getRotationMatrix(double theta1, double theta2, double theta3){
    // angle axis to euler angle 
    double angle = sqrt(theta1 * theta1 + theta2 * theta2 + theta3 * theta3);
    double rotation_x = theta1/angle;
    double rotation_y = theta2/angle;
    double rotation_z = theta3/angle;

    return Mat_<double>(3,3) << \
        (1-cos(angle))*rotation_x*rotation_x + cos(angle), (1-cos(angle))*rotation_x*rotation_y - sin(angle)*rotation_z, (1-cos(angle))*rotation_x*rotation_y + sin(angle)*rotation_y,\
        (1-cos(angle))*rotation_x*rotation_y + sin(angle)*rotation_z, (1-cos(angle))*rotation_y*rotation_y + cos(angle), (1-cos(angle))*rotation_y*rotation_z - sin(angle)*rotation_x,\
        (1-cos(angle))*rotation_x*rotation_z - sin(angle)*rotation_y, (1-cos(angle))*rotation_y*rotation_z + sin(angle)*rotation_x, (1-cos(angle))*rotation_z*rotation_z + cos(angle);

}

}
