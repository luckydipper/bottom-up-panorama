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
#include "stitcher.hpp"
using namespace cv;
using namespace std;


int main(){
    // !!CAUTION!! All object's indexing imgs start with index 1.    
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
        Ptr<FeatureDetector> detector = ORB::create(); // By default make 500 features // 1000,1.2,8,31,0,2,ORB::HARRIS_SCORE,31,20
        Ptr<DescriptorExtractor> descriptor = ORB::create(); // By default 32 byte per one keypoint. 
        detector->detect(imgs[i],keypoints[i]);
        descriptor->compute(imgs[i],keypoints[i],descriptors[i]); // 256bit, 256 pair of points 256X2 points 32=> 8bit*32 bitmap
    }

    // matches and homographys are upper triangle matrix. without diagonose elements ex) [3][4]-> ok, [3][3], [4][3] -> error 
    vector<DMatch> matches[11][11]; 
    Mat homographys[11][11];
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming"); //, norm_hamming, DescriptorMatcher::create("BruteForce-Hamming")
    for(int i = 1; i <= NUM_IMGS; i++){
        for(int j = i+1; j <=NUM_IMGS; j++){
            cout << "match between " << i << " and " << j <<" image.\n";
            matcher->match(descriptors[i],descriptors[j],matches[i][j]);
            vector<Point2i> pts1, pts2;
            for(int k = 0; k <matches[i][j].size(); k++){
                pts1.push_back(keypoints[i][ matches[i][j][k].queryIdx].pt );
                pts2.push_back(keypoints[j][ matches[i][j][k].trainIdx].pt );
            }
            homographys[i][j] = findHomography(pts1, pts2,CV_RANSAC,3.,noArray(),10000,0.995);
            pts1.clear();
            pts2.clear();
        }
    }
    cout << "Complete matching and homography. \n";

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

        Point2d translated_origin = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).first;
        Size transform_size = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).second; 
        
        Mat translation_matrix = Mat::eye(3, 3, CV_64F);
        translation_matrix.at<double>(0,2) = -translated_origin.x;
        translation_matrix.at<double>(1,2) = -translated_origin.y;

        Mat projective_img(transform_size, CV_8UC3);
        warpPerspective(imgs[i], projective_img, translation_matrix*perspectiv_transform, transform_size, INTER_LINEAR);

        bottom_up::fillUnoccupiedImage(stitched_img, projective_img, make_pair(ORIGIN_ROW+translated_origin.y , ORIGIN_COL+translated_origin.x));

        bottom_up::showResizedImg(stitched_img, 0.05);
    }
    imwrite("my_result.jpg", stitched_img);
}