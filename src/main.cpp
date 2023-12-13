#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <cassert>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d.hpp> // FindHomography
#include <opencv2/calib3d/calib3d_c.h> // CV_RANSAC
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp> // FeatureDetector, DescriptorExtractor
#include <opencv2/core.hpp> // inside type,  Keypoints
#include <eigen3/Eigen/Core>
#include "matcher.hpp"
#include "util.hpp"
using namespace cv;
using namespace std;


int main(){
    // Load ten images 
    Mat imgs[11];
    vector<KeyPoint> keypoints[11];
    Mat descriptors[11];
    
    for(int i=1; i <= 10; i++){
        // Load imgs
        string src("../imgs/");
        string index = to_string(i);
        string extension(".jpg");
        string img_path = src + index + extension;
        cout << "loading " <<img_path << "\n"; 
        imgs[i] = imread(img_path, IMREAD_COLOR);
        assert(imgs[i].data && "image path is not valid."); 
        
        // detect and descript the imgs
        Ptr<FeatureDetector> detector = ORB::create(); // By default make 500 features // 1000,1.2,8,31,0,2,ORB::HARRIS_SCORE,31,20
        Ptr<DescriptorExtractor> descriptor = ORB::create(); // By default 32 byte per one keypoint. 
        detector->detect(imgs[i],keypoints[i]);
        descriptor->compute(imgs[i],keypoints[i],descriptors[i]); // 256bit, 256 pair of points 256X2 points 32=> 8bit*32 bitmap
    }

    // upper triangle matrix. ex) [3][3],[3][4]-> ok, [4][3] -> error 
    vector<DMatch> matches[11][11]; 
    Mat homographys[11][11];
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming"); //, norm_hamming, DescriptorMatcher::create("BruteForce-Hamming")
    for(int i = 1; i <= 10; i++){
        for(int j = i; j <=10; j++){
            cout << "match between " << i << " and " << j <<" image.\n";
            matcher->match(descriptors[i],descriptors[j],matches[i][j]);
            vector<Point2i> pts1, pts2;
            for(int k = 0; k <matches[i][j].size(); k++){
                pts1.push_back(keypoints[i][ matches[i][j][k].queryIdx].pt );
                pts2.push_back(keypoints[j][ matches[i][j][k].trainIdx].pt );
            }
            //cout << pts1.size() << " " << pts2.size();
            homographys[i][j] = findHomography(pts1, pts2,CV_RANSAC,3.,noArray(),10000,0.995);
            pts1.clear();
            pts2.clear();
        }
    }

    // homographys
    

    cout << "complete matching! \n";

    int test_idx_l=9, test_idx_r=10;
    //matcher->match(descriptors[test_idx_l],descriptors[test_idx_r],matches[test_idx_l][test_idx_r]);

    // vector<Point2i> pts1, pts2;
    // for(int i = 0; i <matches[test_idx_l][test_idx_r].size(); i++){
    //         pts1.push_back(keypoints[test_idx_l][ matches[test_idx_l][test_idx_r][i].queryIdx].pt );
    //         pts2.push_back(keypoints[test_idx_r][ matches[test_idx_l][test_idx_r][i].trainIdx].pt );
    // }    
    // Mat homography = findHomography(pts1, pts2, CV_RANSAC,3.,noArray(),10000,0.995);
    // Mat inv_homography;
    // cout << homography << "\n" << homography.size;
    // invert(homography, inv_homography);
    Mat projective_img;
    cout << homographys[test_idx_l][test_idx_r];

    warpPerspective(imgs[test_idx_l],projective_img, homographys[test_idx_l][test_idx_r], Size(imgs[test_idx_r].rows*2, imgs[test_idx_r].cols*1.2), INTER_LINEAR);//INTER_CUBIC

    Mat panorama;
    panorama = projective_img.clone();
    Mat ROI(panorama, Rect(0,0,imgs[test_idx_l].cols, imgs[test_idx_l].rows));
    imgs[test_idx_r].copyTo(ROI);

    bottom_up::showResizedImg(panorama,0.25);
    
    bottom_up::showResizedImg(projective_img, 0.25);
    
    const int IMAGE_ROW_SIZE = imgs[1].rows, IMAGE_COL_SIZE = imgs[1].cols;
    const int ORIGIN_ROW = IMAGE_ROW_SIZE*2, ORIGIN_COL = IMAGE_COL_SIZE*2;

    cv::Mat stitched_img(Size(imgs[1].rows*5,imgs[1].cols*5),CV_8UC3);

    Mat tmp(stitched_img, Rect(ORIGIN_ROW, ORIGIN_COL, IMAGE_ROW_SIZE, IMAGE_COL_SIZE));
    projective_img.copyTo(stitched_img);

    //stitched_img.ptr<Vec3d>(ORIGIN_TRANSPOSE.first, ORIGIN_TRANSPOSE.second) = 1 ;
    bottom_up::showResizedImg(stitched_img,0.1);


}