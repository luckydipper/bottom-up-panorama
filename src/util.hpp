// This module is made in order to testing other modules
#ifndef gui_test_bottom_up_panorama
    #include <iostream>
    #include <vector>
    #include <opencv2/core.hpp>
    #include <opencv2/features2d.hpp>
    #include <opencv2/imgproc/imgproc.hpp> 
    #include <opencv2/highgui.hpp>
    
#endif
using namespace cv;
using namespace std;

namespace bottom_up{ 
void showKeypoints(const Mat &img, const vector<KeyPoint> &keypoints, const double size_ratio);
void showResizedImg(const Mat& img, const double size_ratio);

// This has O(img.cols * img.rows * num_channel) complexity. 
// We can do this more effectively 
// 1. By memset and controlling the sequence pasted. 
// 2. By image blending.
}