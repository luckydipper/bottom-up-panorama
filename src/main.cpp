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
    
    int end_img_index = 3;

    for(int i=1; i < 10; i++){
        // Load imgs
        string src("../imgs/");
        string index = to_string(i);
        string extension(".jpg");
        string img_path = src + index + extension;
        cout << "loading " <<img_path << "\n"; 
        imgs[i] = imread(img_path, IMREAD_COLOR);
        assert(imgs[i].data && "image path is not valid."); 
        
        // detect and descript the imgs
        Ptr<FeatureDetector> detector = ORB::create(); // By default make 500 features
        Ptr<DescriptorExtractor> descriptor = ORB::create(); // By default 32 byte per one keypoint. 
        detector->detect(imgs[i],keypoints[i]);
        descriptor->compute(imgs[i],keypoints[i],descriptors[i]); // 256bit, 256 pair of points 256X2 points 32=> 8bit*32 bitmap
        bottom_up::showKeypoints(imgs[i],keypoints[i],0.25);
    }
    //BF Matcher implementation 
    //multimap<int, pair<int,int> > matchs = bottom_up::bruteForceMatcher(descriptors[3],descriptors[4]);

    vector<DMatch> matches[11][11];
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    int test_idx_l=4, test_idx_r=3;
    matcher->match(descriptors[test_idx_l],descriptors[test_idx_r],matches[test_idx_l][test_idx_r]);
    

	double dMaxDist = matches[test_idx_l][test_idx_r][0].distance;
	double dMinDist = matches[test_idx_l][test_idx_r][0].distance;
	double dDistance;

	// 두 개의 keypoint 사이에서 min-max를 계산한다 (min값만 사용)
	for (int i = 0; i < descriptors[test_idx_l].rows; i++) {
		dDistance = matches[test_idx_l][test_idx_r][i].distance;

		if (dDistance < dMinDist) dMinDist = dDistance;
		if (dDistance > dMaxDist) dMaxDist = dDistance;
	}
	printf("max_dist : %f \n", dMaxDist);
	printf("min_dist : %f \n", dMinDist);

	//match의 distance 값이 작을수록 matching이 잘 된 것
	//min의 값의 3배 또는 good_matches.size() > 60 까지만 goodmatch로 인정해준다.
	vector<DMatch>good_matches;
	int distance = 10;
	do {
		vector<DMatch>good_matches2;
		for (int i = 0; i < descriptors[test_idx_l].rows; i++) {
			if (matches[test_idx_l][test_idx_r][i].distance < distance * dMinDist)
				good_matches2.push_back(matches[test_idx_l][test_idx_r][i]);
		}
		good_matches = good_matches2;
		distance -= 1;
	} while (distance != 2 && good_matches.size() > 60);

    Mat matGoodMatches;
    drawMatches(imgs[test_idx_l], keypoints[test_idx_l], imgs[test_idx_r], keypoints[test_idx_r], good_matches, matGoodMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    resize(matGoodMatches,matGoodMatches, Size(matGoodMatches.cols/4,matGoodMatches.rows/4));

    imshow("-1", matGoodMatches );
    waitKey();

    // cout << matches[test_idx_l][test_idx_r].size();
    vector<Point2f> pts1, pts2;
    for(int i = 0; i <good_matches.size(); i++){
            pts1.push_back(keypoints[test_idx_l][ matches[test_idx_l][test_idx_r][i].queryIdx].pt );
            pts2.push_back(keypoints[test_idx_r][ matches[test_idx_l][test_idx_r][i].trainIdx].pt );
    }    
    Mat homography = findHomography(pts1, pts2, CV_RANSAC,3.,noArray(),10000,0.995);
    cout << homography << "\n" << homography.size;
    Mat projective_img;

    warpPerspective(imgs[test_idx_r],projective_img, homography, Size(imgs[test_idx_l].cols*2, imgs[test_idx_l].rows*1.2), INTER_LINEAR);//INTER_CUBIC


    Mat panorama;
    panorama = projective_img.clone();
    Mat ROI(panorama, Rect(0,0,imgs[test_idx_l].cols, imgs[test_idx_l].rows));
    imgs[test_idx_l].copyTo(ROI);

    resize(panorama,panorama,Size(panorama.cols/4,panorama.rows/4));
    imshow("d",panorama);
    resize(projective_img,projective_img, Size(projective_img.cols/4,projective_img.rows/4));
    imshow("a", projective_img);
    

    resize(imgs[test_idx_l],imgs[test_idx_l], Size(imgs[test_idx_l].cols/4,imgs[test_idx_l].rows/4));
    resize(imgs[test_idx_r],imgs[test_idx_r], Size(imgs[test_idx_r].cols/4,imgs[test_idx_r].rows/4));
    
    imshow("b", imgs[test_idx_l]);
    imshow("c", imgs[test_idx_r]);
    waitKey();
}