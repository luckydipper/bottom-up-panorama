#include "optimizer.hpp"
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

cv::Mat computeHomographyDLT(const std::vector<cv::Point2f>& srcPoints, 
                             const std::vector<cv::Point2f>& dstPoints) {
    assert(srcPoints.size() == dstPoints.size() && srcPoints.size() >= 4);

    // Construct matrix A
    Eigen::MatrixXd A(2 * srcPoints.size(), 9);
    for (size_t i = 0; i < srcPoints.size(); ++i) {
        double x = srcPoints[i].x, y = srcPoints[i].y;
        double u = dstPoints[i].x, v = dstPoints[i].y;

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
    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H.at<double>(i, j) = h(i * 3 + j);
        }
    }

    return H;
}



    //Mat rotation_matrix = bottom_up::getRotationMatrix(0,0,M_PI/2);
    //Mat result = rotation_matrix* (Mat_<double>(3,1) << 1.,1.,1.);
    //cout << rotation_matrix << "\n" << result << "\n\n";

    // Mat tranpose_rot_mat, invert_intrinsic;
    // transpose(bottom_up::getRotationMatrix(0, 0, 1), tranpose_rot_mat);
    // invert(bottom_up::getIntrinsicMatrix(12), invert_intrinsic);
    // Mat perspective_transform = bottom_up::getIntrinsicMatrix(20) * bottom_up::getRotationMatrix(0, 0, M_PI/20) * tranpose_rot_mat * invert_intrinsic;
    
    // cout << bottom_up::getRotationMatrix(0, 0, 1) * tranpose_rot_mat <<"\n";
    // Point2d translated_origin = bottom_up::getTranslatedBox(perspective_transform, imgs[1]).first;
    // Size transform_size = bottom_up::getTranslatedBox(perspective_transform, imgs[1]).second; 
        
    // Mat translation_matrix = Mat::eye(3, 3, CV_64F);
    // translation_matrix.at<double>(0,2) = -translated_origin.x;
    // translation_matrix.at<double>(1,2) = -translated_origin.y;
    // cout << translation_matrix*perspective_transform << "\n";
    // cout << transform_size << "\n";
    // bottom_up::showResizedImg(imgs[1], 0.1);
    // Mat projective_img = bottom_up::getHomographyImg(imgs[1], translation_matrix*perspective_transform);
    // bottom_up::showResizedImg(projective_img,0.1);
    // return 1;
}
