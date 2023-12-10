#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <map>
#include <string>
#include <cassert>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d.hpp> // FindHomography
#include <opencv2/calib3d/calib3d_c.h> // CV_RANSAC
#include <opencv2/imgproc/imgproc.hpp>
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

    //FLANN matcher


    
    //Brute Force matcher 
    multimap<int, pair<int,int> > bruteForceMatcher(const Mat & descriptor1, const Mat &descriptor2){
        assert(descriptor1.rows == descriptor2.rows && descriptor1.cols == descriptor2.cols);
        multimap<int, pair<int,int>> result_match;
        int num_features = descriptor1.rows;
        for(int i = 0; i < num_features; i++){
            for(int j = 0; j < num_features; j++){
                int distance=0;
                for(int bit_seq = 0; bit_seq < descriptor1.cols; bit_seq++){
                    uchar batch1 = descriptor1.at<int>(i,bit_seq);
                    uchar batch2 = descriptor2.at<int>(j,bit_seq);
                    distance += hammingDistance(batch1, batch2);
                }
                result_match.insert({distance, {i,j}});
            }
        }
        return result_match;
    }

    //TODO : should make match class multimap<int, pair<int,int> > 
    // struct matche{
    //     matche(const Mat &descriptor1, const Mat &descriptor2){
             
    //     }
        
    // }


    //find homography


    // TODO : add interpolation method
    enum Interpolate{
        linear,
    };


    // Mat perspectiveTransform(const Mat& origin, const Mat& homography, int rows, int cols, bottom_up::Interpolate interpolate_method = bottom_up::Interpolate::linear){
    //     assert(homography.cols == 3 && homography.rows == 3);
    //     assert(origin.rows <= rows && origin.cols <= cols);
        
    // }

}



int main(){
    // Load ten images 
    Mat imgs[11];
    vector<KeyPoint> keypoints[11];
    Mat descriptors[11];
    int end_img_index = 3;

    for(int i=3; i < 5; i++){
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
    }
    //BF Matcher implementation 
    multimap<int, pair<int,int> > matchs = bottom_up::bruteForceMatcher(descriptors[3],descriptors[4]);




    // vector<DMatch> matchs_;
    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    // matcher->matchs_
    
    // for(auto m : matchs){
    //     cout <<" m : " << m.first << "\n" << m.second.first << ", "<<m.second.second <<"\n\n";
    //     Mat result_img;

    //     matchs_.push_back(DMatch(m.second.first, m.second.second, m.first) );
        //drawMatches(imgs[3],keypoints[3],imgs[4],keypoints[4],matchs_ ,result_img, Scalar::all(-1),Scalar::all(-1),vector<char>(),DrawMatchesFlags::DEFAULT);
        // cv::resize(result_img,result_img, Size(result_img.cols/4,result_img.rows/4));
        // imshow("a", result_img); 
        
        // waitKey();
    //}
    // namedWindow("a",CV_WINDOW_AUTOSIZE);

    // test code
    // cout << bottom_up::hammingDistance(128,5);
    //
    //


    vector<Point2f> domain_features, co_domain_features;
    
    Mat homography = findHomography(imgs[3],imgs[4],CV_RANSAC);
    cout << homography;
    Mat projective_img;
    warpPerspective(imgs[3],projective_img, homography,Size(imgs[4].cols*2, imgs[4].rows*1.2), INTER_CUBIC);
    resize(projective_img,projective_img, Size(projective_img.cols/4,projective_img.rows/4));
    imshow("a", projective_img);
    waitKey();
}