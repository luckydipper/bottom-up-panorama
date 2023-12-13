#include "stitcher.hpp"
#include <iostream>

namespace bottom_up{

    // TODO : get homography By multiplying the adjacent Homography. 

    // This function only suport double 3d points
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

    Mat getHomographyImg(const Mat& img, const Mat& perspective_transform, const Size &size){

        Mat result(size, CV_8UC3);
        Mat inv_perspect;

        invert(perspective_transform,inv_perspect);

        vector<Point2d> orignal_points = {Point2d(0,0),         Point2d(img.cols, 0),
                                          Point2d(0, img.rows), Point2d(img.rows, img.cols)}; // TODO : vector is collum. so we do like this. 

        vector<Point2d> transformed_points = getTransformedPoints(perspective_transform, orignal_points);
        
        cout << transformed_points << "\n";
        Point2d* square_points = &transformed_points[0];
        implFlooding(square_points, inv_perspect, size.width/2, size.height/2, img, result);
        bottom_up::showResizedImg(result, 0.05);

        return result;
    }

    void implFlooding(const Point2d square_points[4], const Mat& inverse_perspective, const int starting_x, const int starting_y, const Mat&origin_img,  Mat& target){
        if( !isInSquare( square_points, Point2d(starting_y, starting_x) ) )
            return;

        for(int channel = 0; channel < 3; channel++){
            if(target.at<Vec3b>(starting_y, starting_x)[channel]) // channel is not empty
                return;
        }
        
        vector<Point2d> inv_coordinate = getTransformedPoints(inverse_perspective, vector<Point2d>  {{starting_x,starting_y}} );
        //cout << inv_coordinate << " \n\n";
        double inv_x = inv_coordinate[0].x, inv_y = inv_coordinate[0].y;
        if(inv_x < 0 || inv_y < 0 || inv_x >  target.cols || inv_y > target.rows)
            return;

        //TODO : Linear interpolation
        //int floor_x = (int)inv_x, floor_y = (int)inv_y;
        //int ceiling_x = floor_x+1, ceiling_x = floor_y+1;
        // if(inter == Interpolation::linear){
        //     for(int channel = 0; channel < 3; channel++)
        //         target.at<Vec3b>(starting_y,starting_x)[channel] 
        // }
        // else
        //     assert(false && "TBD");
        
        for(int channel = 0; channel < 3; channel++)
            target.at<Vec3b>(starting_y, starting_x)[channel] = origin_img.at<Vec3b> ((int)inv_y , (int)inv_x)[channel];
        
        int dy[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
        int dx[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
        
        for(int direction = 0; direction < 8; direction++)
            implFlooding(square_points, inverse_perspective, starting_x+dx[direction],starting_y+dy[direction],origin_img, target);
    }

    bool isInSquare(const Point2d square_points[4], const Point2d& query_point){

        if(!isCounterClock(square_points[2], square_points[3], query_point))
                return false;
        if(!isCounterClock(square_points[3], square_points[1], query_point))
                return false;
        if(!isCounterClock(square_points[1], square_points[0], query_point))
                return false;
        if(!isCounterClock(square_points[0], square_points[2], query_point))
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
