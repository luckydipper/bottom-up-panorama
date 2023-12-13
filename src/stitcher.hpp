#include <opencv2/core.hpp>
#include <algorithm>
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
    Mat getHomographyImg(const Mat& img, const Mat& perspective_tranform, const Interpolation inter=Interpolation::linear);

    bool isInSquare(const Point2d square_points[4], const Point2d& query_point);
    bool isCounterClock(const Point2d& origin, const Point2d& img_corner, const Point2d& suspect);

}
