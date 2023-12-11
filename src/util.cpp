#include "util.hpp"

namespace bottom_up{
void showKeypoints(const Mat &img, const vector<KeyPoint> &keypoints ,const double size_ratio){
    Mat result;
    drawKeypoints(img, keypoints, result, Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    resize(result, result, Size(result.cols * size_ratio, result.rows* size_ratio));
    imshow("result", result);
    waitKey();
    destroyAllWindows();
}
} 