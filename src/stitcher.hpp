#ifndef ___stitcher___
#define ___stitcher___
#include <opencv2/core.hpp>
#include <algorithm>
#include <queue>
#include "util.hpp"
using namespace cv;
using namespace std;

namespace bottom_up{
    enum Interpolation{
        linear,
        nearest,
    };

    // This function only suport double 3d points
    // TODO : generic programming 
    // This function apply perspective transform to each points
    // input points and out put points are 2d col vector (x,y)^T format. 
    vector<Point2d> getTransformedPoints(const Mat& perspect_transform, const vector<Point2d>& original_points);
    Point2d getTransformedPoints(const Mat& perspect_transform, const Point2d& original_point);


    // This function return Region Of Interest after applying perspective transform 
    // By transform the corner of imgs, get ROI.
    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img);

    // !!CAUTION!! This function is realy realy heavy. to execute this function you should have large RAM space. (16G was enough, in ubuntu22.04)
    // !!CAUTION!! To execute this program you need to chat "ulimit -s unlimited" command in your shell. That command will prevent stack overflow.
    // using DFS back flood filling the transformed imgs.
    // TBD : linear interpolation 
    Mat backWarpImgFloodFill(const Mat& img, const Mat& perspective_transform, Interpolation interpol = Interpolation::nearest);
    void implFlooding(const vector<Point2d>& square_points, const Mat& inverse_perspective, const int starting_y, const int starting_x, const Mat&origin_img,  Mat& target,Interpolation interpol = Interpolation::nearest);
    
    // Check channel are visted 
    bool isChannelOccupied(const Mat& img, int y, int x);

    // Using outer produt determin suspecting point are counter clockwise to the img corner.
    bool isInSquare(const vector<Point2d>& square_points, const Point2d& query_point);
    bool isCounterClock(const Point2d& origin, const Point2d& img_corner, const Point2d& suspect);


    enum Blending{
        average,
        first_come,
    };
    // This has O(img.cols * img.rows * num_channel) complexity. 
    // We can do this more effectively 
    // 1. By memset and controlling the sequence pasted. 
    void fillUnoccupiedImage(Mat& sparse_img, const Mat &filler, pair<int,int> origin, Blending blend = Blending::first_come);
}
#endif