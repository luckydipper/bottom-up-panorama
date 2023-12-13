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


    // This function only suport double 3d points
    // 
    // vector<Point2d> getTransformedPoints(const Mat& perspect_transform, vector<Vec3d> original_points){
    //     vector<Point2d> transformedPoints;
    //     for(int i = 0; i < original_points.size(); i++){
    //         Mat homgeneous_temp = perspect_transform * Mat(original_points[i]);
    //         transformedPoints.push_back({homgeneous_temp.at<double>(0)/homgeneous_temp.at<double>(2), homgeneous_temp.at<double>(1)/homgeneous_temp.at<double>(2) } );
    //     }
    //     return transformedPoints;
    // }


    //
    // TODO : generic programming 
    vector<Point2d> getTransformedPoints(const Mat& perspect_transform, const vector<Point2d>& original_points){
        vector<Vec3d> original_homogeneous;
        for(const auto& o_point : original_points){
            original_homogeneous.push_back({o_point.y, o_point.x, 1});
        }
        vector<Point2d> transformedPoints;
        for(int i = 0; i < original_points.size(); i++){
            Mat homgeneous_temp = perspect_transform * Mat(original_homogeneous[i]);
            transformedPoints.push_back({homgeneous_temp.at<double>(0)/homgeneous_temp.at<double>(2), homgeneous_temp.at<double>(1)/homgeneous_temp.at<double>(2) } );
        }
        return transformedPoints;
    }


    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img){
        // !!Caution!! Corner points sequance should be 0,0 -> 0,1 -> 1,0 -> 1,1 ex) like drawing the latter "Z"
        vector<Point2d> orignal_points = {Point2d(0,0),         Point2d(img.cols, 0),
                                              Point2d(0, img.rows), Point2d(img.rows, img.cols)};

        vector<Point2d> transformed_points = getTransformedPoints(perspective_transform, orignal_points);
        
        Point2d minPt = transformed_points[0];
        Point2d maxPt = transformed_points[0];
        for (int i = 1; i < 4; ++i) {
            minPt.x = min(minPt.x, transformed_points[i].x);
            minPt.y = min(minPt.y, transformed_points[i].y);
            maxPt.x = max(maxPt.x, transformed_points[i].x);
            maxPt.y = max(maxPt.y, transformed_points[i].y);
        }
        return {minPt, Size(maxPt.x - minPt.x, maxPt.y - minPt.y)};
    }

    // Mat getHomographyImg(const Mat& img, const Mat& perspective_transform, const Size &size, const Interpolation& inter=Interpolation::linear){
    //     Mat inv_perspect;
    //     Mat result(size, CV_8UC3);
    //     invert(perspective_transform,inv_perspect);
    //     Point2d transformed_points[4];
    //  transformed_points   TransformedPoints(perspective_transform, img, transformed_points);
        
        
    //     return ;
    // }

    // void implFlooding(const Point2d square_points[4], const Mat& inverse_perspective, const int starting_x, const int starting_y, Mat& target, const Interpolation& inter=Interpolation::linear){
    //     if(starting_x < 0 && starting_y < 0 && starting_x > target.cols && starting_y > target.rows)
    //         return;
        
    //     if( !isInSquare( square_points, Point2d(starting_y, starting_x) ) )
    //         return;
        
    //     for(int channel = 0; channel < 3; channel++)
    //         if(!target.at<Vec3b>(starting_y,starting_x)[channel]) // channel is not empty
    //             return;
        


    //     int dy[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    //     int dx[8] = {-1, -1, -1,  0, 0,  1, 1, 1};

    // }

    bool isInSquare(const Point2d square_points[4], const Point2d& query_point){
        for(int i = 0; i < 4; i++)
            if(!isCounterClock(square_points[i], square_points[(i+1)%4], query_point))
                return false;
        return true;
    }

    bool isCounterClock(const Point2d& origin, const Point2d& img_corner, const Point2d& suspect){
        // OC
        double vector_corner_x = img_corner.x - origin.x;
        double vector_corner_y = img_corner.y - origin.y;

        // OS
        double vector_suspect_x = suspect.x - origin.x;
        double vector_suspect_y = suspect.y - origin.y;

        // crossproduct = OS X OC
        double cross_product = vector_suspect_y * vector_corner_x - vector_suspect_x * vector_corner_y;
        if(cross_product > 0)
            return true;
        else 
            return false;
    }


    // TODO : should make match class multimap<int, pair<int,int> > 
    // struct matche{
    //     matche(const Mat &descriptor1, const Mat &descriptor2){
             
    //     }
        
    // }

    //find homography

    // TODO : add interpolation method


    // Mat perspectiveTransform(const Mat& origin, const Mat& homography, int rows, int cols, bottom_up::Interpolate interpolate_method = bottom_up::Interpolate::linear){
    //     assert(homography.cols == 3 && homography.rows == 3);
    //     assert(origin.rows <= rows && origin.cols <= cols);
        
    // }

}
