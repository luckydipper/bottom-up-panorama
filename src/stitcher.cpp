#include "stitcher.hpp"
#include <iostream>

namespace bottom_up{

    vector<Point2d> getTransformedPoints(const Mat& perspect_transform, const vector<Point2d>& original_points){
        vector<Vec3d> original_homogeneous;
        // make homogeneous coordinate
        for(const auto& o_point : original_points)
            original_homogeneous.push_back({o_point.x, o_point.y, 1.});
        
        vector<Point2d> transformedPoints;
        for(int i = 0; i < original_points.size(); i++){
            Mat homgeneous_tmp = perspect_transform * Mat(original_homogeneous[i]);
            // normalize the homogeneous coordinate
            transformedPoints.push_back({homgeneous_tmp.at<double>(0)/homgeneous_tmp.at<double>(2), homgeneous_tmp.at<double>(1)/homgeneous_tmp.at<double>(2) } );
        }
        return transformedPoints;
    }
    Point2d getTransformedPoints(const Mat& perspect_transform, const Point2d& original_point){
        Mat homgeneous_tmp = perspect_transform * Mat( Vec3d(original_point.x, original_point.y, 1) );
        return {homgeneous_tmp.at<double>(0)/homgeneous_tmp.at<double>(2), homgeneous_tmp.at<double>(1)/homgeneous_tmp.at<double>(2)} ;
    }
    pair<Point2d,Size> getTranslatedBox(const Mat& perspective_transform, const Mat& img){
        // !!Caution!! Corner points sequance should be 0,0 -> 0,1 -> 1,0 -> 1,1 ex) like drawing the latter "Z"
        vector<Point2d> orignal_points = {Point2d(0, 0),        Point2d(img.cols, 0),
                                          Point2d(0, img.rows), Point2d(img.cols, img.rows)};

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


    // !!CAUTION!! This function is realy realy heavy. to execute this function you should have large RAM space. (16G was enough, in ubuntu22.04)
    // !!CAUTION!! To execute this program you need to chat "ulimit -s unlimited" command in your shell. That command will prevent stack overflow.
    Mat backWarpImgFloodFill(const Mat& origin_img, const Mat& perspective_transform, Interpolation interpol){

        Mat inv_perspect;
        invert(perspective_transform, inv_perspect);

        Size perspective_size = getTranslatedBox(perspective_transform, origin_img).second;
        Mat result(perspective_size, CV_8UC3);

        vector<Point2d> counter_clock_origin_corners = {Point2d(0, 0), Point2d(0, origin_img.rows), Point2d(origin_img.cols, origin_img.rows), Point2d(origin_img.cols, 0)};
        vector<Point2d> transformed_points = getTransformedPoints(perspective_transform, counter_clock_origin_corners);

        Point2d perspective_mid = getTransformedPoints(perspective_transform, Point2d(origin_img.cols/2,origin_img.rows/2) );

        implFlooding(transformed_points, inv_perspect, (int)perspective_mid.y, (int)perspective_mid.x, origin_img, result, interpol);
        return result;
    }
    // !!CAUTION!! This function is realy realy heavy. to execute this function you should have large RAM space. (16G was enough, in ubuntu22.04)
    // !!CAUTION!! To execute this program you need to chat "ulimit -s unlimited" command in your shell. That command will prevent stack overflow.
    void implFlooding(const vector<Point2d>& square_points, const Mat& inverse_perspective, const int starting_y, const int starting_x, const Mat&origin_img,  Mat& target, Interpolation interpol){
        Point2d inv_coordinate = getTransformedPoints(inverse_perspective, Point2d(starting_x, starting_y));
        double inv_y = inv_coordinate.y, inv_x = inv_coordinate.x;

        int floor_x = (int)inv_x, floor_y = (int)inv_y;
        int ceiling_x = floor_x+1, ceiling_y = floor_y+1;
        if(floor_y <= 0 || floor_x <= 0 || ceiling_x >=  origin_img.cols || ceiling_y >= origin_img.rows)
            return;
        if(starting_x <=0 || starting_y <= 0)
            return;
        if(interpol == Interpolation::nearest)
            target.at<Vec3b>(starting_y,starting_x) = origin_img.at<Vec3b>(floor_y, floor_x);
        else if(interpol == Interpolation::linear)
            target.at<Vec3b>(starting_y,starting_x) = origin_img.at<Vec3b>(floor_y, floor_x);
        else
            assert(false);
        
        if(origin_img.at<Vec3b> (floor_y,floor_x)[0] == 0)
            target.at<Vec3b>(starting_y, starting_x)[0]++; 
        int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
        int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1, };

        for(int direction = 0; direction < 8; direction++){

            if(isChannelOccupied(target, starting_y+dy[direction], starting_x+dx[direction]))
                continue;
            if( !isInSquare( square_points, Point2d(starting_x+dx[direction],starting_y+dy[direction]) ) ) // HERE
                continue;
            implFlooding(square_points, inverse_perspective, starting_y+dy[direction], starting_x+dx[direction], origin_img, target, interpol);
        }
    }

    bool isChannelOccupied(const Mat& img, int y, int x){
        for(int channel = 0; channel < 3; channel++)
            if(img.at<Vec3b>(y,x)[channel] != 0)
                return true;
        return false;
    }

    bool isInSquare(const vector<Point2d>& counter_clockwise_corner, const Point2d& query_point){

        for(int here = 0; here < counter_clockwise_corner.size(); here++){
            int there = (here + 1)%counter_clockwise_corner.size();
            if(!isCounterClock(counter_clockwise_corner[here], counter_clockwise_corner[there], query_point))
                return false;
        }
        return true;
    }

    bool isCounterClock(const Point2d& origin, const Point2d& img_corner, const Point2d& suspect){
        // OC
        double vector_corner_y = img_corner.y - origin.y;
        double vector_corner_x = img_corner.x - origin.x;

        // OS
        double vector_suspect_y = suspect.y - origin.y;
        double vector_suspect_x = suspect.x - origin.x;

        // crossproduct = OS X OC
        double cross_product = vector_suspect_y * vector_corner_x - vector_suspect_x * vector_corner_y;
        if(cross_product < 0)
            return true;
        else 
            return false;
    }
    
    void fillUnoccupiedImage(Mat& sparse_img, const Mat &filler, const pair<int,int> origin, Blending blend){
    for(int i = 0; i < filler.rows; i++){
        for(int j =0; j < filler.cols; j++){
            int target_y = origin.first + i;
            int target_x = origin.second + j;
            
            for(int channel = 0; channel < 3; channel++){
                //out of range
                if(target_x <= 0 || target_y <= 0 || target_x >= sparse_img.cols || target_y >= sparse_img.rows)
                    continue;
                //is visited
                if(sparse_img.at<Vec3b>(target_y, target_x)[channel] != 0){
                    if(blend == Blending::first_come)
                        continue;
                    else if(blend == Blending::average){
                        if( filler.at<Vec3b>(i, j)[channel] != 0)
                            sparse_img.at<Vec3b>(target_y, target_x)[channel] = (filler.at<Vec3b>(i, j)[channel] + sparse_img.at<Vec3b>(target_y, target_x)[channel])/2;
                        continue;
                    }
                    else{
                        cout << blend <<"\n";
                        cout << Blending::average << "\n";
                        cout << Blending::first_come << "\n";
                        continue;
                    }
                }
                    
                sparse_img.at<Vec3b>(target_y, target_x)[channel] = filler.at<Vec3b>(i, j)[channel];
            }
            
        }
    }
}

}
