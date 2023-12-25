#include "optimizer.hpp"
#define _XOPEN_SOURCE 700 // to do this.

using namespace std;
using namespace cv;

namespace bottom_up{

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

Mat getDLTHomographyRANSAC(vector<Point2d> src, vector<Point2d> dst, int iteration, double euclidian_threshold, int model_sample_size){
    
    Mat best_homography = Mat::eye(3,3,CV_64FC1); 
    int max_inlier =-999999999;// -INF

    while(iteration--){
        vector<int> interesting_index =  sampleIndexes(model_sample_size, src.size());
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

vector<int> sampleIndexes(int sample_size,int max_size){
    random_device seed;
    mt19937 gen(seed());
    vector<int> interesting_match;
    uniform_int_distribution<> dist(0, max_size-1);
    for(int i = 0; i < sample_size; i++)
        interesting_match.push_back(dist(gen));
    
    return interesting_match;
} 

//*********************************************************************************************************************//
// Below source codes are redundant. :(
// I tried but I couldn't used it.

Mat getIntrinsicMatrix(double focal_length){
    return Mat_<double>(3,3) << focal_length,0,0, 0,focal_length,0, 0,0,1;
}

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


// Function to compute the Jacobian matrix of the homography at a given point
Eigen::MatrixXd computeJacobian(const Point2d &src, const Eigen::MatrixXd &H) {
    double x = src.x;
    double y = src.y;
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
// Eigen::MatrixXd refineHomographyGaussNewton(const vector<Point2d> &src, const vector<Point2d> &dst, const Eigen::MatrixXd &initial_homography, int max_iterations) {
//     Eigen::MatrixXd H = initial_homography;
//     for (int iter = 0; iter < maxIterations; ++iter) {
//         Eigen::MatrixXd J; // Overall Jacobian matrix
//         Eigen::VectorXd r; // Overall residual vector

//         for (size_t i = 0; i < src.size(); ++i) {
//             Point2d transformed_pt = applyHomography(H, src[i]);
//             Eigen::MatrixXd J_i = computeJacobian(src[i], H); // Jacobian for this point

//             // Residual for this point
//             Eigen::VectorXd r_i(2);
//             r_i << transformed_pt.x - dstPoints[i].x, transformed_pt.y - dstPoints[i].y;

//             // Append to overall Jacobian and residual
//             J.conservativeResize(J.rows() + J_i.rows(), Eigen::NoChange);
//             J.bottomRows(J_i.rows()) = J_i;
//             r.conservativeResize(r.size() + r_i.size());
//             r.tail(r_i.size()) = r_i;
//         }

//         // Compute the update
//         Eigen::VectorXd delta = (J.transpose() * J).ldlt().solve(-J.transpose() * r);

//         // Update the homography matrix
//         Eigen::MatrixXd deltaMat = Eigen::MatrixXd::Identity(3, 3);
//         deltaMat(0, 0) += delta(0);
//         deltaMat(0, 1) += delta(1);
//         deltaMat(0, 2) += delta(2);
//         deltaMat(1, 0) += delta(3);
//         deltaMat(1, 1) += delta(4);
//         deltaMat(1, 2) += delta(5);
//         deltaMat(2, 0) += delta(6);
//         deltaMat(2, 1) += delta(7);

//         H = H * deltaMat;
//     }
//     return H;
// }
}
