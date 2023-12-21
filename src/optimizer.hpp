#include <opencv2/core.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include "matcher.hpp"

using namespace std;
using namespace cv;

//double f1, theta11, theta12, theta13, f2, theta21, theta22, theta23;
namespace bottom_up{
Mat getIntrinsicMatrix(double focal_length);
Mat getRotationMatrix(double theta1, double theta2, double theta3);
cv::Mat computeHomographyDLT(const std::vector<cv::KeyPoint>& source, 
                             const std::vector<cv::KeyPoint>& destination,
                             const vector<bottom_up::FeatureMapping>& matches);
}
