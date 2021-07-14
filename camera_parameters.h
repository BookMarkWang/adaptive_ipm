#ifndef __CAMERA_PARAMETERS_H__
#define __CAMERA_PARAMETERS_H__

#include <opencv2/opencv.hpp>

const double DEG2RAD = 0.01745329252;  //radian = degree * PI / 180

class CameraParameters
{
public:
	CameraParameters() = default;
	CameraParameters(const cv::Matx33d& camera_matrix, const cv::Vec4d& distortion_coeffs);
	~CameraParameters() = default;

	void SetParameters(const cv::Matx33d& camera_matrix, const cv::Vec4d& distortion_coeffs);
	inline void SetK1(const double k1) { k1_ = k1; }
	inline void SetK2(const double k2) { k2_ = k2; }
	inline void SetK3(const double k3) { k3_ = k3; }
	inline void SetK4(const double k4) { k4_ = k4; }
	inline void SetP1(const double p1) { p1_ = p1; }
	inline void SetP2(const double p2) { p2_ = p2; }
	inline void SetFx(const double fx) { fx_ = fx; }
	inline void SetFy(const double fy) { fy_ = fy; }
	inline void SetCx(const double cx) { cx_ = cx; }
	inline void SetCy(const double cy) { cy_ = cy; }

	inline void SetRoll(const double roll) { roll_angle_ = roll * DEG2RAD; }
	inline void SetYaw(const double yaw) { yaw_angle_ = yaw * DEG2RAD; }
	inline void SetPitch(const double pitch) { pitch_angle_ = pitch * DEG2RAD; }
	inline void SetHeight(const double height) { height_ = height; }

	inline double GetK1() const{ return k1_; }
	inline double GetK2() const{ return k2_; }
	inline double GetK3() const{ return k3_; }
	inline double GetK4() const{ return k4_; }
	inline double GetP1() const { return p1_; }
	inline double GetP2() const { return p2_; }
	inline double GetFx() const { return fx_; }
	inline double GetFy() const { return fy_; }
	inline double GetCx() const { return cx_; }
	inline double GetCy() const { return cy_; }



	inline double GetRoll() const { return roll_angle_; }
	inline double GetYaw() const { return yaw_angle_; }
	inline double GetPitch() const { return pitch_angle_; }
	inline double GetHeight() const { return height_; }

	cv::Matx33d GetCameraMatrix();
	cv::Vec4d GetDistortionCoeffs();

private:
	//内参
	double k1_ = 0.0;
	double k2_ = 0.0;
	double k3_ = 0.0;
	double k4_ = 0.0;
	double p1_ = 0.0;
	double p2_ = 0.0;

	double fx_ = 0.0;
	double fy_ = 0.0;

	double cx_ = 0.0;
	double cy_ = 0.0;

	//外参
	double roll_angle_ = 0.0;
	double yaw_angle_ = 0.0;
	double pitch_angle_ = 0.0;
	double height_ = 0.0;  //mm
};

#endif //__CAMERA_PARAMETERS_H__

