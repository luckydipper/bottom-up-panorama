#include <opencv2/core.hpp>
#include <algorithm>
using namespace cv;
using namespace std;

namespace bottom_up{
    //void transposeToReference(int reference_index, const Mat** const Homograpys, Mat* transpose_to_reference,int num_imgs);
    // Size getTranslatedSize(const Mat& perspective_transform, const Mat& img);
    // Point2d getTranslatedOrigin(const Mat& perspective_transform, const Mat& img);
    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img);
}
