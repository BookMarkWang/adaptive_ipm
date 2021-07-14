#include "camera_parameters.h"

CameraParameters::CameraParameters(const cv::Matx33d& camera_matrix, const cv::Vec4d& distortion_coeffs)
{
    SetParameters(camera_matrix, distortion_coeffs);

    roll_angle_ = 0.0;
    yaw_angle_ = 0.0;
    pitch_angle_ = 0.0;
    height_ = 0.0;
}

void CameraParameters::SetParameters(const cv::Matx33d& camera_matrix, const cv::Vec4d& distortion_coeffs) 
{
    cx_ = camera_matrix(0, 2);
    cy_ = camera_matrix(1, 2);
    fx_ = camera_matrix(0, 0);
    fy_ = camera_matrix(1, 1);
    k1_ = distortion_coeffs(0);
    k2_ = distortion_coeffs(1);
    k3_ = distortion_coeffs(2);
    k4_ = distortion_coeffs(3);
    p1_ = 0.0;
    p2_ = 0.0;
}

cv::Matx33d CameraParameters::GetCameraMatrix()
{
    return cv::Matx33d(
        fx_, 0., cx_,
        0., fy_, cy_,
        0., 0., 1.);
}

cv::Vec4d CameraParameters::GetDistortionCoeffs()
{
    return cv::Vec4d(k1_, k2_, k3_, k4_);
}
