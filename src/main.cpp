#include <iostream>
#include <string>
#include <cassert>
#include <fstream>
#include <cmath>
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

    // matches and homography's square brackets are img's index!:  
    // ex) matches[1][5] -> matching data img 1 to img 5
    // ex) homography[1][5] -> homography matrix img 1 to img 5
    vector<DMatch> matches[11][11]; 
    Mat homographys[11][11];
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    
    // Reference is mid index of the imgs
    const int REFERENCE = 5;


    // We need to compute step homography(perspective) matrix. H[i][i+1] 
    // Dynamic Programming caching the homography.
    // ex) H[1][5] = H[1][2] * H[2][3] * H[3][4] * H[4][5];
    for(int i = 1; i < REFERENCE; i++){
        cout << "match between " << i << " and " << i+1 <<" image.\n";
        matcher->match(descriptors[i],descriptors[i+1],matches[i][i+1]);
        vector<Point2d> pts1, pts2;
        for(int k = 0; k <matches[i][i+1].size(); k++){
            pts1.push_back(keypoints[i][ matches[i][i+1][k].queryIdx].pt );
            pts2.push_back(keypoints[i+1][ matches[i][i+1][k].trainIdx].pt );
        }
        homographys[i][i+1] = bottom_up::getDLTHomographyRANSAC(pts1, pts2, 1000, 100.); //20
        cout << "homography : " << homographys[i][i+1] << "\n";
        pts1.clear();
        pts2.clear();
    }
    for(int i = 10; i > REFERENCE ; i--){
        cout << "match between " << i << " and " << i-1 <<" image.\n";
        matcher->match(descriptors[i],descriptors[i-1],matches[i][i-1]);
        vector<Point2d> pts1, pts2;
        for(int k = 0; k <matches[i][i-1].size(); k++){
            pts1.push_back(keypoints[i][ matches[i][i-1][k].queryIdx].pt );
            pts2.push_back(keypoints[i-1][ matches[i][i-1][k].trainIdx].pt );
        }
        homographys[i][i-1] = bottom_up::getDLTHomographyRANSAC(pts1, pts2, 1000, 100.); //20
        cout << "homography : " << homographys[i][i-1] << "\n";
        pts1.clear();
        pts2.clear();
    }

    // Get homography(persepective) matrix to reference img.
    // Using DP, use stored homography.
    for(int i = 1; i < REFERENCE ; i++){
        int step = REFERENCE - i;
        homographys[i][REFERENCE] = homographys[i][i+1];
        for(int j = 0; j < step; j++)
            homographys[i][REFERENCE] = homographys[i][REFERENCE] * homographys[i+j][i+j+1];
        cout << "homography " << i << " -> " << REFERENCE << "\n";
        cout << homographys[i][REFERENCE] << "\n";   
    }
    for(int i = 10; i > REFERENCE ; i--){
        int step = i - REFERENCE;
        homographys[i][REFERENCE] = homographys[i][i-1];
        for(int j = 0; j < step; j++)
            homographys[i][REFERENCE] = homographys[i][REFERENCE] * homographys[i-j][i-j-1];
        cout << "homography " << i << " -> " << REFERENCE << "\n";
        cout << homographys[i][REFERENCE] << "\n";   
    }

    cout << "Complete matching and homography. \n";

    const int IMAGE_HEIGHT = imgs[1].rows, IMAGE_WIDTH = imgs[1].cols;
    const int ORIGIN_ROW = IMAGE_HEIGHT*1, ORIGIN_COL = IMAGE_WIDTH*2;
    
    // This is Domain img.
    cv::Mat stitched_img(Size(IMAGE_WIDTH*5, IMAGE_HEIGHT*5), CV_8UC3);

    Mat reference_img = imgs[REFERENCE];
    bottom_up::fillUnoccupiedImage(stitched_img, reference_img, make_pair(ORIGIN_ROW , ORIGIN_COL));
    bottom_up::showResizedImg(stitched_img, 0.05);

    for(int i = NUM_IMGS; i >= 1; i--){
        Mat perspectiv_transform;
        perspectiv_transform = homographys[i][REFERENCE]; 
        if(i == REFERENCE)
            continue;
        
        // Get region of interest size and location. 
        Point2d translated_origin = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).first;
        Size transform_size = bottom_up::getTranslatedBox(perspectiv_transform, imgs[i]).second; 

        Mat translation_matrix = Mat::eye(3, 3, CV_64F);
        translation_matrix.at<double>(0,2) = -translated_origin.x;
        translation_matrix.at<double>(1,2) = -translated_origin.y;

        cout << i << " image Stitching...\n";
        // Get transformed images. 
        // The reason multiplying the translation_matrix is to make full image, without skip
        Mat projective_img = bottom_up::backWarpImgFloodFill(imgs[i],translation_matrix*perspectiv_transform); 
        bottom_up::fillUnoccupiedImage(stitched_img, projective_img, make_pair(ORIGIN_ROW+translated_origin.y , ORIGIN_COL+translated_origin.x));
        cout << projective_img.size() << " :  projective size. \n";
        bottom_up::showResizedImg(projective_img,0.2);
        bottom_up::showResizedImg(stitched_img, 0.1);
        
        // Save perspective transformed img
        cout << "projective Image  saving..\n";
        string file_name = "projective";
        string extention =".jpg";
        file_name = file_name + to_string(i) + extention;
        cv::imwrite(file_name, projective_img);

        // Save Homography matrix and it's location
        ofstream fout;
        file_name = to_string(i) + "_box_roi.txt";
        fout.open(file_name);
        fout << translated_origin.x << ", " << translated_origin.y << ", " << transform_size.width << ", " << transform_size.height << endl;
        fout << perspectiv_transform << endl;
        fout.close();
    }



    cout << "Image saving..\n";
    cv::imwrite("my_result.jpg", stitched_img);

}