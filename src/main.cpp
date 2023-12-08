#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <cassert>
#include <opencv2/features2d.hpp> // FeatureDetector, DescriptorExtractor
#include <opencv2/core.hpp> // inside type,  Keypoints
#include <eigen3/Eigen/Core>

using namespace cv;
using namespace std;
// Homography to reference image
// detector
// descriptor 
// macher 



vector<DMatch> featureMatcher(const Mat &img1, const vector<KeyPoint> &keypoints_1, const Mat &img2, const vector<KeyPoint> &keypoints_2 ){
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    detector->detect(img1)
    descriptor->compute()
}


int main(){
    // Load ten images 
    Mat imgs[10];
    for(int i=0; i < 10; i++){
        string index = to_string(i);
        string extension(".jpg");
        string img_name = extension + index;
        imgs[i] = imread(img_name, IMREAD_COLOR);
        assert(!imgs[i].data); //image load error
        imshow(img_name, imgs[i]);   

    }
    
    waitKey();
}