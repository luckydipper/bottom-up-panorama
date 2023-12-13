#include "stitcher.hpp"
#include <iostream>

namespace bottom_up{
    // void transposeToReference(int reference_index, const Mat** const Homograpys, Mat* transpose_to_reference, int num_imgs){
    //     for(int i = 1; i <= num_imgs; i++){
    //         transpose_to_reference[i] = Mat::eye(3,3,CV_16U);
    //         if( i <= reference_index){
    //             for(int i = 1; i<= reference_index; i++)
    //                 transpose_to_reference[i] *= Homograpys[i][]; 
    //         }
            
    //     }
    // }


    // !!Caution!! Corner points sequance should be 0,0 -> 0,1 -> 1,0 -> 1,1 ex) like   
    // Size getTranslatedSize(const Mat& perspective_transform, const Mat& img){
    //     vector<Vec3d> corners = { Vec3d(0, 0, 1),        Vec3d(img.cols, 0, 1),
    //                               Vec3d(0, img.rows, 1), Vec3d(img.cols, img.rows, 1)};

    //     Point2d transformed_coners[4];
    //     for(int i = 0; i < 4; i++){
    //         Mat homgeneous_temp = perspective_transform * Mat(corners[i]);
    //         transformed_coners[i] = {homgeneous_temp.at<double>(0)/homgeneous_temp.at<double>(2), homgeneous_temp.at<double>(1)/homgeneous_temp.at<double>(2) } ;
    //     }

    //     Point2d minPt = transformed_coners[0];
    //     Point2d maxPt = transformed_coners[0];
    //     for (int i = 1; i < 4; ++i) {
    //         minPt.x = min(minPt.x, transformed_coners[i].x);
    //         minPt.y = min(minPt.y, transformed_coners[i].y);
    //         maxPt.x = max(maxPt.x, transformed_coners[i].x);
    //         maxPt.y = max(maxPt.y, transformed_coners[i].y);
    //     }

    //     return Size(maxPt.x - minPt.x, maxPt.y - minPt.y);
    // }

    // Point2d getTranslatedOrigin(const Mat& perspective_transform, const Mat& img){
    //     vector<Vec3d> corners = { Vec3d(0, 0, 1),        Vec3d(img.cols, 0, 1),
    //                               Vec3d(0, img.rows, 1), Vec3d(img.cols, img.rows, 1)};

    //     Point2d transformed_coners[4];
    //     for(int i = 0; i < 4; i++){
    //         Mat homgeneous_temp = perspective_transform * Mat(corners[i]);
    //         transformed_coners[i] = {homgeneous_temp.at<double>(0)/homgeneous_temp.at<double>(2), homgeneous_temp.at<double>(1)/homgeneous_temp.at<double>(2) } ;
    //     }

    //     Point2d minPt = transformed_coners[0];
    //     Point2d maxPt = transformed_coners[0];
    //     for (int i = 1; i < 4; ++i) {
    //         minPt.x = min(minPt.x, transformed_coners[i].x);
    //         minPt.y = min(minPt.y, transformed_coners[i].y);
    //         maxPt.x = max(maxPt.x, transformed_coners[i].x);
    //         maxPt.y = max(maxPt.y, transformed_coners[i].y);
    //     }
    //     return minPt;
    // }



    // !!Caution!! Corner points sequance should be 0,0 -> 0,1 -> 1,0 -> 1,1 ex) like drawing the latter "Z"
    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img){
        vector<Vec3d> corners = { Vec3d(0, 0, 1),        Vec3d(img.cols, 0, 1),
                                  Vec3d(0, img.rows, 1), Vec3d(img.cols, img.rows, 1)};

        Point2d transformed_coners[4];
        for(int i = 0; i < 4; i++){
            Mat homgeneous_temp = perspective_transform * Mat(corners[i]);
            transformed_coners[i] = {homgeneous_temp.at<double>(0)/homgeneous_temp.at<double>(2), homgeneous_temp.at<double>(1)/homgeneous_temp.at<double>(2) } ;
        }
        
        Point2d minPt = transformed_coners[0];
        Point2d maxPt = transformed_coners[0];
        for (int i = 1; i < 4; ++i) {
            minPt.x = min(minPt.x, transformed_coners[i].x);
            minPt.y = min(minPt.y, transformed_coners[i].y);
            maxPt.x = max(maxPt.x, transformed_coners[i].x);
            maxPt.y = max(maxPt.y, transformed_coners[i].y);
        }

        return {minPt, Size(maxPt.x - minPt.x, maxPt.y - minPt.y)};
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
