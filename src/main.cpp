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


namespace bottom_up{
    int hammingDistance(const uchar code1, const uchar code2){
        int result = 0;
        int XOR = (code1 ^ code2);
        while(XOR >0){
            if(XOR%2 > 0)
                result++;
            XOR >>= 1;
        }
        return result;
    }

    

}



int main(){
    // Load ten images 
    Mat imgs[11];
    vector<KeyPoint> keypoints[11];
    Mat descriptors[11];
    int end_img_index = 3;

    for(int i=1; i < end_img_index; i++){
        // Load imgs
        string src("../imgs/");
        string index = to_string(i);
        string extension(".jpg");
        string img_path = src + index + extension;
        cout << img_path; 
        imgs[i] = imread(img_path, IMREAD_COLOR);
        assert(imgs[i].data); 
        
        // detect and descript the imgs
        Ptr<FeatureDetector> detector = ORB::create(); // by default make 500 features
        Ptr<DescriptorExtractor> descriptor = ORB::create(); // by default 32 bits per one keypoint. 
        detector->detect(imgs[i],keypoints[i]);
        descriptor->compute(imgs[i],keypoints[i],descriptors[i]); // 256bit,  256 pair of points 256X2 points 32=> 8bit*32 bitmap
        cout << descriptors[i].size() << "\n\n";   
    }

    //BF Matcher implementation 




    // test code
    // cout << bottom_up::hammingDistance(128,5);
    //imshow(img_path, imgs[i]); 
    //waitKey();
}