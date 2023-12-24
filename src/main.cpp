#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <cassert>
#include <cmath>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d.hpp> // FindHomography
#include <opencv2/calib3d/calib3d_c.h> // CV_RANSAC
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp> // FeatureDetector, DescriptorExtractor
#include <opencv2/core.hpp> // inside type,  Keypoints
#include <eigen3/Eigen/Core>
#include "matcher.hpp"
#include "util.hpp"
#include "stitcher.hpp"
#include "optimizer.hpp"
using namespace cv;
using namespace std;


int main(){
    // !!CAUTION!! All object's indexing imgs start with index 1.    
    // !!CAUTION!! If segment fault error are occured, type "ulimit -s unlimited" in your shell. FloodFill algorithm with DFS has heavy overhead
    Mat imgs[11];
    vector<KeyPoint> keypoints[11];
    Mat descriptors[11];
    const int NUM_IMGS = 10;
    for(int i=1; i <= NUM_IMGS; i++){
        // Load imgs
        string src("../imgs/");
        string index = to_string(i);
        string extension(".jpg");
        string img_path = src + index + extension;
        cout << "loading " <<img_path << "\n"; 
        imgs[i] = imread(img_path, IMREAD_COLOR);
        assert(imgs[i].data && "image path is not valid."); 
    
        // detect and descript the imgs
        Ptr<FeatureDetector> detector = SIFT::create(); // By default make 500 features // 1000,1.2,8,31,0,2,ORB::HARRIS_SCORE,31,20
        Ptr<DescriptorExtractor> descriptor = SIFT::create(); // By default 32 byte per one keypoint. 
        detector->detect(imgs[i],keypoints[i]);
        descriptor->compute(imgs[i],keypoints[i],descriptors[i]); // 256bit, 256 pair of points 256X2 points 32=> 8bit*32 bitmap
    }
    // matches and homographys are upper triangle matrix. without diagonose elements ex) [3][4]-> ok, [3][3], [4][3] -> error 
    vector<DMatch> matches[11][11]; 
    Mat homographys[11][11];
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED); //, norm_hamming, DescriptorMatcher::create("BruteForce-Hamming")
    
    for(int i = 1; i <= 2; i++){
        cout << "match between " << i << " and " << 5 <<" image.\n";
        matcher->match(descriptors[i],descriptors[5],matches[i][5]);
        vector<Point2d> pts1, pts2;
        for(int k = 0; k <matches[i][5].size(); k++){
            pts1.push_back(keypoints[i][ matches[i][5][k].queryIdx].pt );
            pts2.push_back(keypoints[5][ matches[i][5][k].trainIdx].pt );
        }
        //homographys[i][5] = findHomography(pts1, pts2,CV_RANSAC,3.,noArray(),10000,0.995);
        homographys[i][5] = bottom_up::RANSAC(pts1, pts2, 500, 20.); //20
        cout << "homography : " << homographys[i][5] << "\n";
        pts1.clear();
        pts2.clear();

    }
    cout << "Complete matching and homography. \n";


    ///////////////////////////////////////////////////////////////////////////////////////
    const int IMAGE_HEIGHT = imgs[1].rows, IMAGE_WIDTH = imgs[1].cols;
    const int ORIGIN_ROW = IMAGE_HEIGHT*1, ORIGIN_COL = IMAGE_WIDTH*2;
    
    // This is Domain img.
    cv::Mat stitched_img(Size(IMAGE_WIDTH*5, IMAGE_HEIGHT*5), CV_8UC3);

    const int REFERENCE = 5;
    Mat reference_img = imgs[REFERENCE];
    bottom_up::fillUnoccupiedImage(stitched_img, reference_img, make_pair(ORIGIN_ROW , ORIGIN_COL));

    bottom_up::showResizedImg(stitched_img, 0.05);

    for(int i = 1; i <= NUM_IMGS; i++){
        Mat perspectiv_transform;
        perspectiv_transform = homographys[i][REFERENCE]; 
        if(i == REFERENCE)
            continue;
        else if( i > REFERENCE )
            invert(homographys[REFERENCE][i],perspectiv_transform);

        //perspectiv_transform = (Mat_<double>(3,3) << 1.11673615e+00, -7.17904939e-02, -1.03418243e+03, 9.13088818e-02,  1.02092184e+00, -1.48211942e+02, 6.31173358e-05,-3.50995542e-05, 1.00000000e+00);
        Point2d translated_origin = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).first;
        Size transform_size = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).second; 
        Mat translation_matrix = Mat::eye(3, 3, CV_64F);
        translation_matrix.at<double>(0,2) = -translated_origin.x;
        translation_matrix.at<double>(1,2) = -translated_origin.y;

        cout << i << " image Stitching...\n";
        Mat projective_img = bottom_up::getHomographyImg(imgs[i],translation_matrix*perspectiv_transform);
        ////TODO : If i use bottomup homography matrix, abort error happen.
        bottom_up::fillUnoccupiedImage(stitched_img, projective_img, make_pair(ORIGIN_ROW+translated_origin.y , ORIGIN_COL+translated_origin.x));
        //cout << stitched_img.size() << " :  stitched_img size. \n";
        cout << projective_img.size() << " :  projective size. \n";
        bottom_up::showResizedImg(projective_img,0.2);
        bottom_up::showResizedImg(stitched_img, 0.1);
    }



    cout << "Image saving..\n";
    imwrite("my_result.jpg", stitched_img);

}