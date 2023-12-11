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
}