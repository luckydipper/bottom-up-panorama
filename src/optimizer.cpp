#include "optimizer.hpp"
#define _XOPEN_SOURCE 700 // to do this.

using namespace std;
using namespace cv;

double f1, theta11, theta12, theta13, f2, theta21, theta22, theta23;
namespace bottom_up{

Mat getIntrinsicMatrix(double focal_length){
    return Mat_<double>(3,3) << focal_length,0,0, 0,focal_length,0, 0,0,1;
}

//TODO : Division by zero
// Mat getRotationMatrix(double theta1, double theta2, double theta3){
//     // angle axis to euler angle 
//     double angle = sqrt(theta1 * theta1 + theta2 * theta2 + theta3 * theta3);
//     double rotation_x = theta1/angle;
//     double rotation_y = theta2/angle;
//     double rotation_z = theta3/angle;

//     return Mat_<double>(3,3) << \
//         (1-cos(angle))*rotation_x*rotation_x + cos(angle), (1-cos(angle))*rotation_x*rotation_y - sin(angle)*rotation_z, (1-cos(angle))*rotation_x*rotation_y + sin(angle)*rotation_y,\
//         (1-cos(angle))*rotation_x*rotation_y + sin(angle)*rotation_z, (1-cos(angle))*rotation_y*rotation_y + cos(angle), (1-cos(angle))*rotation_y*rotation_z - sin(angle)*rotation_x,\
//         (1-cos(angle))*rotation_x*rotation_z - sin(angle)*rotation_y, (1-cos(angle))*rotation_y*rotation_z + sin(angle)*rotation_x, (1-cos(angle))*rotation_z*rotation_z + cos(angle);

// }

Mat getRotationMatrix(double theta1, double theta2, double theta3){
    double angle = sqrt(theta1 * theta1 + theta2 * theta2 + theta3 * theta3);

    // Handle the case when the angle is zero to avoid division by zero
    if (angle == 0) {
        return Mat::eye(3, 3, CV_64F);
    }

    double rotation_x = theta1 / angle;
    double rotation_y = theta2 / angle;
    double rotation_z = theta3 / angle;

    // Constructing the skew-symmetric matrix from the axis
    Mat K = (Mat_<double>(3,3) << 
                0, -rotation_z, rotation_y, 
                rotation_z, 0, -rotation_x, 
                -rotation_y, rotation_x, 0);

    // Applying Rodrigues' formula
    Mat rotationMatrix = Mat::eye(3, 3, CV_64F) + sin(angle) * K + (1 - cos(angle)) * K * K;

    return rotationMatrix;
}



// cv::Mat computeHomographyDLT(const std::vector<cv::KeyPoint>& source, 
//                                      const std::vector<cv::KeyPoint>& destination,
//                                      const vector<bottom_up::FeatureMapping>& matches ) {
    

//     int SAMPLE_SIZE = 6;
//     double EUCLIDIAN_THRESHOLD = 2.;
 
//     Mat result_homography;
    
//     int max_inlier =-999999999;// -INF
//     while(max_inlier < 6){
//         //RANSAC 
//         vector<bottom_up::FeatureMapping> interesting_match = sampleArbitaryMatches(matches, SAMPLE_SIZE);

//         // Jacobian 
//         Eigen::MatrixXd A(2 * interesting_match.size(), 9);
//         for(int i = 0; i < interesting_match.size(); ++i){
//             Point2d src_point = source[interesting_match[i].here].pt, dst_point = destination[interesting_match[i].there].pt;
//             double x = src_point.x, y = src_point.y;
//             double u = dst_point.x, v = dst_point.y;
    
//             A(2*i, 0) = x;
//             A(2*i, 1) = y;
//             A(2*i, 2) = 1;
//             A(2*i, 3) = 0;
//             A(2*i, 4) = 0;
//             A(2*i, 5) = 0;
//             A(2*i, 6) = -1*x * u;
//             A(2*i, 7) = -1*y * u;
//             A(2*i, 8) = -1*u;
    
//             A(2*i+1, 0) = 0;
//             A(2*i+1, 1) = 0;
//             A(2*i+1, 2) = 0;
//             A(2*i+1, 3) = x;
//             A(2*i+1, 4) = y;
//             A(2*i+1, 5) = 1;
//             A(2*i+1, 6) = -1*x * v;
//             A(2*i+1, 7) = -1*y * v;
//             A(2*i+1, 8) = -1*v;
//         }
//         // Compute SVD of A
//         Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
//         Eigen::VectorXd h = svd.matrixV().col(8);

//         // Convert the Eigen matrix to cv::Mat
//         Mat H = Mat::zeros(3, 3, CV_64F);
//         for (int i = 0; i < 3; ++i) {
//             for (int j = 0; j < 3; ++j) {
//                 H.at<double>(i, j) = h(i * 3 + j) / h(8);
//             }
//         }
//         vector<KeyPoint> query_points, gt_points;
//         for (const auto& m : interesting_match){
//             query_points.push_back(source[m.here]);
//             gt_points.push_back(destination[m.there]);
//         }
//         int inlier = countInlier(query_points, gt_points, H, EUCLIDIAN_THRESHOLD);
//         if(inlier > max_inlier){
//             max_inlier = inlier;
//             result_homography = H;
//         }
//         interesting_match.clear(); 
//     }
//     return result_homography;
// }


cv::Mat getHomographyMat(const vector<Point2d> &src, const vector<Point2d> &dst){
    Eigen::MatrixXd A(2 * src.size(), 9);
    for(int i = 0; i < src.size(); ++i){
        double x = src[i].x, y = src[i].y;
        double u = dst[i].x, v = dst[i].y;

        A(2*i, 0) = -x;
        A(2*i, 1) = -y;
        A(2*i, 2) = -1;
        A(2*i, 3) = 0;
        A(2*i, 4) = 0;
        A(2*i, 5) = 0;
        A(2*i, 6) = x * u;
        A(2*i, 7) = y * u;
        A(2*i, 8) = u;

        A(2*i+1, 0) = 0;
        A(2*i+1, 1) = 0;
        A(2*i+1, 2) = 0;
        A(2*i+1, 3) = -x;
        A(2*i+1, 4) = -y;
        A(2*i+1, 5) = -1;
        A(2*i+1, 6) = x * v;
        A(2*i+1, 7) = y * v;
        A(2*i+1, 8) = v;
    }
    // Compute SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd h = svd.matrixV().col(8);
    // Convert the Eigen matrix to cv::Mat
    Mat H = Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H.at<double>(i, j) = h(i * 3 + j) / h(8);
        }
    }
    return H;
}

////////
// cv::Mat getInitialHomographyRANSAC(const vector<cv::KeyPoint>& source, 
//                                    const vector<cv::KeyPoint>& destination,
//                                    const vector<bottom_up::FeatureMapping>& matches,
//                                    const int iteration) {
//     extern Mat imgs[11];
//     extern vector<KeyPoint> keypoints[11];
//     extern Mat descriptors[11];
        
//     cout << matches.size( ) << " " << source.size() << " "<< destination.size();
//     int MODEL_SAMPLE_SIZE = 4;
//     double EUCLIDIAN_THRESHOLD = 20.;
//     Mat best_homography = Mat::eye(3,3,CV_64FC1); 
//     int max_inlier =-999999999;// -INF
//     for(int i = 0; i < iteration; i++){
//         vector<bottom_up::FeatureMapping> interesting_match = sampleArbitaryMatches(matches, MODEL_SAMPLE_SIZE);
//         Mat result_img;
//         vector<DMatch> matchs_;

//         vector<Point2d> model_src_points, model_dst_points;
//         for(const auto &m : interesting_match){
//             model_src_points.push_back(source[m.here].pt);
//             model_dst_points.push_back(destination[m.there].pt);
//         }

//         Mat current_model = getHomographyMat(model_src_points, model_dst_points); 

//         int inlier = countInlier(source, destination, current_model, EUCLIDIAN_THRESHOLD);
//         if(inlier > max_inlier){
//             max_inlier = inlier;
//             best_homography = current_model;
//         }
//         interesting_match.clear(); 
//     }
//     cout << "max inlier" << max_inlier << "\n";
//     return best_homography;
// }


// int countInlier(vector<KeyPoint> queries, vector<KeyPoint> ground_truthes, Mat homography, double threshold){
//     int num_inlier = 0;
//     vector<Point2d> queries_;
//     for(const KeyPoint &kp : queries)
//         queries_.push_back(Point2d(kp.pt));
//     vector<Point2d> predicts = bottom_up::getTransformedPoints(homography, queries_);//std::vector<cv::Point2d> &original_points)

//     for(int i = 0; i < predicts.size(); i++){
//         double dx = predicts[i].x - ground_truthes[i].pt.x;
//         double dy = predicts[i].y - ground_truthes[i].pt.y;
//         double distance = sqrt(dx*dx + dy*dy);

//         if(distance < threshold)
//             num_inlier++;
//     }
//     return num_inlier;
// }


int countInlier(vector<Point2d> queries, vector<Point2d> ground_truthes, Mat homography, double threshold){
    vector<Point2d> predicts = bottom_up::getTransformedPoints(homography, queries);//std::vector<cv::Point2d> &original_points)
    int num_inlier = 0;
    for(int i = 0; i < predicts.size(); i++){
        double dx = predicts[i].x - ground_truthes[i].x;
        double dy = predicts[i].y - ground_truthes[i].y;
        double distance = sqrt(dx*dx + dy*dy);

        if(distance < threshold)
            num_inlier++;
    }
    return num_inlier;
}



Mat RANSAC(vector<Point2d> src, vector<Point2d> dst, int iteration, double euclidian_threshold){
    
    const int MODEL_SAMPLE_SIZE = 4; 
    Mat best_homography = Mat::eye(3,3,CV_64FC1); 
    int max_inlier =-999999999;// -INF

    
    while(iteration--){
        vector<int> interesting_index =  sampleMatch(MODEL_SAMPLE_SIZE, src.size());
        vector<Point2d> sampled_src, sampled_dst;
        for(int index : interesting_index){
            sampled_src.push_back(src[index]);
            sampled_dst.push_back(dst[index]);
        }
        Mat current_model = getHomographyMat(sampled_src, sampled_dst);
        int inlier = countInlier(src, dst, current_model, euclidian_threshold);
        if(inlier > max_inlier){
            max_inlier = inlier;
            best_homography = current_model;
            cout <<"max inlier : " << max_inlier << "\n";
        }
    }
    return best_homography;
}


vector<bottom_up::FeatureMapping> sampleArbitaryMatches(const vector<bottom_up::FeatureMapping>& matches, int sample_size){
    random_device seed;
    mt19937 gen(seed());
    uniform_int_distribution<> dist(0, matches.size());
    vector<bottom_up::FeatureMapping> interesting_match;
    for(int i = 0; i < sample_size; i++){
        int index = dist(gen);
        interesting_match.push_back(matches[index]);
    }
    return interesting_match;
} 


vector<int> sampleMatch(int sample_size,int max_size){
    random_device seed;
    mt19937 gen(seed());
    vector<int> interesting_match;
    uniform_int_distribution<> dist(0, max_size-1);
    for(int i = 0; i < sample_size; i++)
        interesting_match.push_back(dist(gen));
    
    return interesting_match;
} 
///////////////////////////////////////////////////////////////////



// Function to apply homography to a point
Point2d applyHomography(const Eigen::MatrixXd &H, const Point2d &pt) {
    Eigen::Vector3d pt_homogeneous(pt.x, pt.y, 1.0);
    Eigen::Vector3d transformed_pt = H * pt_homogeneous;
    return Point2d(transformed_pt(0) / transformed_pt(2), transformed_pt(1) / transformed_pt(2));
}

// Function to compute the Jacobian matrix of the homography at a given point
Eigen::MatrixXd computeJacobian(const Point2d &srcPt, const Eigen::MatrixXd &H) {
    double x = srcPt.x;
    double y = srcPt.y;
    Eigen::Vector3d pt_homogeneous(x, y, 2.0);
    Eigen::Vector3d transformed_pt = H * pt_homogeneous;

    double u = transformed_pt(0);
    double v = transformed_pt(1);
    double w = transformed_pt(2);

    Eigen::MatrixXd J(2, 8);
    J << x/w, y/w, 1.0/w, 0.0, 0.0, 0.0, -x*u/(w*w), -y*u/(w*w),
         0.0, 0.0, 0.0, x/w, y/w, 1.0/w, -x*v/(w*w), -y*v/(w*w);
    return J;
}

vector<Point2d> vecKeyPoint2vecPoint2d(const vector<KeyPoint> kp){
    vector<Point2d> result;
    for(const auto &p : kp)
        result.push_back(p.pt);
    return result;
}



Eigen::MatrixXd Mat2Eigen(const Mat &m) {
    Eigen::MatrixXd eigen_mat(m.rows, m.cols);
    for (int i = 0; i < m.rows; ++i) {
        for (int j = 0; j < m.cols; ++j) {
            eigen_mat(i, j) = m.at<double>(i, j);
        }
    }
    return eigen_mat;
}


// Gauss-Newton optimization to refine the homography matrix
Eigen::MatrixXd refineHomographyGaussNewton(const vector<Point2d> &srcPoints, const vector<Point2d> &dstPoints, const Eigen::MatrixXd &initialHomography, int maxIterations) {
    Eigen::MatrixXd H = initialHomography;
    for (int iter = 0; iter < maxIterations; ++iter) {
        Eigen::MatrixXd J; // Overall Jacobian matrix
        Eigen::VectorXd r; // Overall residual vector

        for (size_t i = 0; i < srcPoints.size(); ++i) {
            Point2d transformed_pt = applyHomography(H, srcPoints[i]);
            Eigen::MatrixXd J_i = computeJacobian(srcPoints[i], H); // Jacobian for this point

            // Residual for this point
            Eigen::VectorXd r_i(2);
            r_i << transformed_pt.x - dstPoints[i].x, transformed_pt.y - dstPoints[i].y;

            // Append to overall Jacobian and residual
            J.conservativeResize(J.rows() + J_i.rows(), Eigen::NoChange);
            J.bottomRows(J_i.rows()) = J_i;
            r.conservativeResize(r.size() + r_i.size());
            r.tail(r_i.size()) = r_i;
        }

        // Compute the update
        Eigen::VectorXd delta = (J.transpose() * J).ldlt().solve(-J.transpose() * r);

        // Update the homography matrix
        Eigen::MatrixXd deltaMat = Eigen::MatrixXd::Identity(3, 3);
        deltaMat(0, 0) += delta(0);
        deltaMat(0, 1) += delta(1);
        deltaMat(0, 2) += delta(2);
        deltaMat(1, 0) += delta(3);
        deltaMat(1, 1) += delta(4);
        deltaMat(1, 2) += delta(5);
        deltaMat(2, 0) += delta(6);
        deltaMat(2, 1) += delta(7);

        H = H * deltaMat;
    }
    return H;
}



}
