#ifndef ___stitcher___
#define ___stitcher___
#include <opencv2/core.hpp>
#include <algorithm>
#include <queue>
#include "util.hpp"
using namespace cv;
using namespace std;

namespace bottom_up{
    //void transposeToReference(int reference_index, const Mat** const Homograpys, Mat* transpose_to_reference,int num_imgs);
    // Size getTranslatedSize(const Mat& perspective_transform, const Mat& img);
    // Point2d getTranslatedOrigin(const Mat& perspective_transform, const Mat& img);
    enum Interpolation{
        linear,
        polar,
    };
    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img);
    Mat backWarpImgFloodFill(const Mat& img, const Mat& perspective_transform);
    vector<Point2d> getTransformedPoints(const Mat& perspect_transform, const vector<Point2d>& original_points);
    Point2d getTransformedPoints(const Mat& perspect_transform, const Point2d& original_point);
    bool isChannelOccupied(const Mat& img, int y, int x);

    
    void implFlooding(const vector<Point2d>& square_points, const Mat& inverse_perspective, const int starting_y, const int starting_x, const Mat&origin_img,  Mat& target);
    bool isInSquare(const vector<Point2d>& square_points, const Point2d& query_point);
    bool isCounterClock(const Point2d& origin, const Point2d& img_corner, const Point2d& suspect);

}
#endif