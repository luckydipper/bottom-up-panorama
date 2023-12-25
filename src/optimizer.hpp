#ifndef ___optimizer___
#define ___optimizer___
#include <opencv2/core.hpp>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include "matcher.hpp"
#include "stitcher.hpp" // to use getTransformedPoint function

using namespace std;
using namespace cv;
using namespace Eigen;

namespace bottom_up{

cv::Mat computeHomographyDLT(const std::vector<cv::KeyPoint>& source, 
                             const std::vector<cv::KeyPoint>& destination,
                             const vector<bottom_up::FeatureMapping>& matches);

int countInlier(vector<Point2d> queries, vector<Point2d> ground_truthes, Mat homography, double threshold);

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
Mat getDLTHomographyRANSAC(vector<Point2d> src, vector<Point2d> dst, int iteration, double euclidian_threshold);
vector<int> sampleIndexes(int sample_size,int max_size);


//*********************************************************************************************************************//
// Below headers are redundant. :(
// TODO : find homography by decomposition those models  
Mat getIntrinsicMatrix(double focal_length);
Mat getRotationMatrix(double theta1, double theta2, double theta3);
Eigen::MatrixXd Mat2Eigen(const cv::Mat &cvMat);

//TODO : optimize each inlier 
cv::Mat computeHomographyGaussNewton(const std::vector<cv::KeyPoint>& source, 
                                     const std::vector<cv::KeyPoint>& destination,
                                     const vector<bottom_up::FeatureMapping>& matches);

}

#endif
