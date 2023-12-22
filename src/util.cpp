#include "util.hpp"

namespace bottom_up{
void showKeypoints(const Mat &img, const vector<KeyPoint> &keypoints ,const double size_ratio){
    Mat result;
    drawKeypoints(img, keypoints, result, Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    resize(result, result, Size(result.cols * size_ratio, result.rows* size_ratio));
    imshow("result", result);
    waitKey();
    destroyAllWindows();
}
void showResizedImg(const Mat& img, const double size_ratio){
    Mat result = img;
    resize(result, result, Size(result.cols * size_ratio, result.rows* size_ratio));
    namedWindow("result", WINDOW_NORMAL);
    imshow("result", result);
    waitKey();
    destroyAllWindows();
}

void fillUnoccupiedImage(Mat& sparse_img, const Mat &filler, const pair<int,int> origin){
    for(int i = 0; i < filler.rows; i++){
        for(int j =0; j < filler.cols; j++){
            int target_y = origin.first + i;
            int target_x = origin.second + j;
            
            for(int channel = 0; channel < 3; channel++){
                if(sparse_img.at<Vec3b>(target_y, target_x)[channel] != 0)
                    continue;
                sparse_img.at<Vec3b>(target_y, target_x)[channel] = filler.at<Vec3b>(i, j)[channel];
            }
            
        }
    }
}
} 