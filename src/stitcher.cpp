#include "stitcher.hpp"
#include <iostream>

namespace bottom_up{

    // TODO : get homography By multiplying the adjacent Homography. 

    // This function only suport double 3d points
    // TODO : generic programming 
    vector<Point2d> getTransformedPoints(const Mat& perspect_transform, const vector<Point2d>& original_points){
        vector<Vec3d> original_homogeneous;
        cout << "perspective _transform are \n" << perspect_transform <<"\n\n";

        for(const auto& o_point : original_points){
            original_homogeneous.push_back({o_point.x, o_point.y, 1.});
            cout << o_point <<" ";
        }
        cout << "\n\n";
        vector<Point2d> transformedPoints;
        for(int i = 0; i < original_points.size(); i++){
            Mat homgeneous_tmp = perspect_transform * Mat(original_homogeneous[i]);
            transformedPoints.push_back({homgeneous_tmp.at<double>(0)/homgeneous_tmp.at<double>(2), homgeneous_tmp.at<double>(1)/homgeneous_tmp.at<double>(2) } );
            cout << Point2d{homgeneous_tmp.at<double>(0)/homgeneous_tmp.at<double>(2), homgeneous_tmp.at<double>(1)/homgeneous_tmp.at<double>(2) } << " ";
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

    Mat getHomographyImg(const Mat& origin_img, const Mat& perspective_transform){

        Mat inv_perspect;
        invert(perspective_transform, inv_perspect);

        Size perspective_size = getTranslatedBox(perspective_transform, origin_img).second;
        Mat result(perspective_size, CV_8UC3);

        vector<Point2d> counter_clock_origin_corners = {Point2d(0, 0), Point2d(0, origin_img.rows), Point2d(origin_img.cols, origin_img.rows), Point2d(origin_img.cols, 0)};
        vector<Point2d> transformed_points = getTransformedPoints(perspective_transform, counter_clock_origin_corners);
        // for(auto a : counter_clock_origin_corners)
        //     cout << a << "\n";
        // cout <<" - \n";
        // for(auto a : transformed_points)
        //     cout << a << "\n";
        Point2d perspective_mid = getTransformedPoints(perspective_transform, Point2d(origin_img.cols/2,origin_img.rows/2) );

        // cout << perspective_mid << "\n";        
        implFlooding(transformed_points, inv_perspect, (int)perspective_mid.y, (int)perspective_mid.x, origin_img, result);
        return result;
    }
    // TODO : interpolation 
    // !!CAUTION!! This function is realy realy heavy. to execute this function you should have lage RAM space. (16G was enough, in ubuntu22.04)
    // !!CAUTION!! To execute this program you need to chat "ulimit -s unlimited" command in your shell. That command will prevent stack overflow.
    void implFlooding(const vector<Point2d>& square_points, const Mat& inverse_perspective, const int starting_y, const int starting_x, const Mat&origin_img,  Mat& target){
        Point2d inv_coordinate = getTransformedPoints(inverse_perspective, Point2d(starting_x, starting_y));
        double inv_y = inv_coordinate.y, inv_x = inv_coordinate.x;

        //TODO : Linear interpolation
        int floor_x = (int)inv_x, floor_y = (int)inv_y;
        int ceiling_x = floor_x+1, ceiling_y = floor_y+1;
        if(floor_y <= 0 || floor_x <= 0 || ceiling_x >=  origin_img.cols || ceiling_y >= origin_img.rows)
            return;

        target.at<Vec3b>(starting_y,starting_x) = origin_img.at<Vec3b>(floor_y, floor_x);
        if(origin_img.at<Vec3b> (floor_y,floor_x)[0] == 0)
            target.at<Vec3b>(starting_y, starting_x)[0]++; 
        //int i;
        //i++;
        //cout << i <<"--"<< starting_y<<"---" << starting_y << "-----\n";
        //cout << i <<"\n";
        // cout << i <<"--"<< starting_y<<"---" << starting_x << "-----\n";
        // cout << target.at<Vec3b>(starting_y, starting_x) <<"\n"; 
        // cout << origin_img.at<Vec3b> (floor_y,floor_x) << "\n";
        // if(i == 5000000||i == 10000000|| i == 20000000 || i == 30000000)
        //     bottom_up::showResizedImg(target, 0.25);
            //cout << target.at<Vec3d> (3000, 2000) << "\n";
        
        //cout << getTransformedPoints(inverse_perspective, Point2d(1449, 1449)) << "\n";
        //cout << origin_img.at<Vec3d> (getTransformedPoints(inverse_perspective, Point2d(1449, 1449)).x, getTransformedPoints(inverse_perspective, Point2d(1449, 1449)).y) << "---------\n";

        int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
        int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1, };

        for(int direction = 0; direction < 8; direction++){
            // if(starting_y+dy[direction] == 2000 && starting_x+dx[direction] ==3024){
            //     if(isChannelOccupied(target, starting_y+dy[direction], starting_x+dx[direction]))
            //         cout << 111111111111;        
            //     for(auto a : square_points)
            //         cout << a << " ";
            //     if( !isInSquare( square_points, Point2d(starting_y+dy[direction], starting_x+dx[direction]) ) )
            //         cout << 22222222222222;
            //     cout <<"33333333333";
            //     bottom_up::showResizedImg(target,0.5);
            // }
            if(isChannelOccupied(target, starting_y+dy[direction], starting_x+dx[direction]))
                continue;
            if( !isInSquare( square_points, Point2d(starting_x+dx[direction],starting_y+dy[direction]) ) ) // HERE
                continue;
            implFlooding(square_points, inverse_perspective, starting_y+dy[direction], starting_x+dx[direction], origin_img, target);
        }
    }
    

    // TODO : Linear interpolation
    // void implFlooding(const vector<Point2d>& square_points, const Mat& inverse_perspective, const int starting_y, const int starting_x, const Mat&origin_img,  Mat& target){
    //     queue<Point2d> q;
    //     q.push(Point2d(starting_x, starting_y));
    //     int i = 0;
    //     while(!q.empty()){

    //         Point2d point_interest = q.front();
    //         q.pop();

    //         Point2d source_coordinate = getTransformedPoints(inverse_perspective, point_interest);
            
    //         double inv_y = source_coordinate.y, inv_x = source_coordinate.x;
    //         int floor_source_x = (int)inv_x, floor_source_y = (int)inv_y;
    //         int ceiling_x = floor_source_x+1, ceiling_y = floor_source_y+1;
    //         //cout << "ceil x " << ceiling_y << "origin col : " << origin_img.rows << "\n"; 
    //         if(floor_source_y <= 0 || floor_source_x <= 0 || ceiling_x >=  origin_img.cols || ceiling_y >= origin_img.rows)
    //             continue;
    //         // Is visited?
    //         if( !isInSquare(square_points, Point2d(point_interest.x, point_interest.y)) )
    //             continue;
    //         if(isChannelOccupied(target, point_interest.y, point_interest.x))
    //             continue;


    //         i++;
    //         //cout << "target size : " << target.size() << "\n";
    //         //cout << "original size : " << origin_img.size() << "\n";
    //         cout <<"i : " << i << "\n";
    //         cout <<"y :   " << floor_source_y << " x:   " << floor_source_x << "\n";
    //         //cout <<"t_y : " << point_interest.y <<" t x : " << point_interest.x << "\n";
    //         cout << target.at<Vec3b>( point_interest.y, point_interest.x) << " \n";
    //         if(i == 1000000 || i == 2000000 || i == 3000000)
    //             bottom_up::showResizedImg(target, 0.25);
            
    //         // coloring
    //         for(int channel = 0; channel < 3; channel++){
    //             if(origin_img.at<Vec3b> (floor_source_y, floor_source_x)[channel] == 0){
    //                 target.at<Vec3b>(point_interest.y, point_interest.x)[channel]++; 
    //                 break;   
    //             }
    //             //cout <<origin_img.at<Vec3b>(floor_source_y, floor_source_x)[channel]<< " ------ ";
    //             target.at<Vec3b>(point_interest.y, point_interest.x)[channel]= origin_img.at<Vec3b>(floor_source_y,floor_source_x)[channel];
    //         }
    //         cout<<"\n" << target.at<Vec3b>(point_interest.y, point_interest.x ) << "\n";
            
    //         int dx[4] = {-1, 0, 0, 1};
    //         int dy[4] = { 0, -1, 1, 0};
    //         for(int direction = 0; direction < 4; direction++){
    //             if(isChannelOccupied(target, point_interest.y+dy[direction], point_interest.x+dx[direction]))
    //                 continue;
                
    //             if( !isInSquare(square_points, Point2d(point_interest.x+dx[direction],point_interest.y+dy[direction]) ))
    //                 continue;
                
    //             q.push(  Point2d(point_interest.x+dx[direction], point_interest.y+dy[direction])  );
    //         }
    //         cout << "size : " << q.size() << "\n";
            
    //     }
    // }
    
    bool isChannelOccupied(const Mat& img, int y, int x){
        for(int channel = 0; channel < 3; channel++)
            if(img.at<Vec3b>(y,x)[channel] != 0)
                return true;
        return false;
    }

    bool isInSquare(const vector<Point2d>& counter_clockwise_corner, const Point2d& query_point){

        for(int here = 0; here < counter_clockwise_corner.size(); here++){
            int there = (here + 1)%counter_clockwise_corner.size();
            //cout << counter_clockwise_corner[here] << " " <<  counter_clockwise_corner[there] << " " << query_point << "\n";
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
