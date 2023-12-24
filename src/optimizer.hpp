#ifndef ___optimizer___
#define ___optimizer___
#include <opencv2/core.hpp>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include "matcher.hpp"
#include "stitcher.hpp" // to use get transformed point

using namespace std;
using namespace cv;
using namespace Eigen;

//double f1, theta11, theta12, theta13, f2, theta21, theta22, theta23;
namespace bottom_up{

Mat getIntrinsicMatrix(double focal_length);
Mat getRotationMatrix(double theta1, double theta2, double theta3);
cv::Mat computeHomographyDLT(const std::vector<cv::KeyPoint>& source, 
                             const std::vector<cv::KeyPoint>& destination,
                             const vector<bottom_up::FeatureMapping>& matches);
cv::Mat computeHomographyGaussNewton(const std::vector<cv::KeyPoint>& source, 
                                     const std::vector<cv::KeyPoint>& destination,
                                     const vector<bottom_up::FeatureMapping>& matches);
//int countInlier(vector<KeyPoint> queries, vector<KeyPoint> ground_truthes, Mat homography, double threshold);
int countInlier(vector<Point2d> queries, vector<Point2d> ground_truthes, Mat homography, double threshold);

Eigen::MatrixXd Mat2Eigen(const cv::Mat &cvMat);
Point2d applyHomography(const Eigen::MatrixXd &H, const Point2d &pt);
vector<bottom_up::FeatureMapping> sampleArbitaryMatches(const vector<bottom_up::FeatureMapping>& matches, int sample_size);
Eigen::MatrixXd computeJacobian(const Point2d &srcPt, const Eigen::MatrixXd &H);
Eigen::MatrixXd refineHomographyGaussNewton(const vector<Point2d> &srcPoints, const vector<Point2d> &dstPoints, const Eigen::MatrixXd &initialHomography, int maxIterations);
Mat ransacHomographyGN(const vector<KeyPoint> &src, const vector<KeyPoint> dst, const vector<bottom_up::FeatureMapping> matches, const Mat& initial_homogarphy, int num_iter);

cv::Mat getInitialHomographyRANSAC(const vector<cv::KeyPoint>& source, 
                                   const vector<cv::KeyPoint>& destination,
                                   const vector<bottom_up::FeatureMapping>& matches,
                                   const int iteration);


cv::Mat getHomographyMat(const vector<Point2d> &src, const vector<Point2d> &dst);
Mat RANSAC(vector<Point2d> src, vector<Point2d> dst, int iteration, double euclidian_threshold);
vector<int> sampleMatch(int sample_size,int max_size);



}

#endif
