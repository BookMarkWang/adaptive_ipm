#ifndef __ADAPTIVE_IPM_H__
#define __ADAPTIVE_IPM_H__

#include <string>
#include <vector>
#include <map>
#include "camera_parameters.h"


class IpmInfo
{
public:
	IpmInfo();
	~IpmInfo() = default;
	///min and max x-value on ground in world coordinates
	float xLimits[2];
	///min and max y-value on ground in world coordinates
	float yLimits[2];

	///conversion between mm in world coordinate on the ground in x-direction and pixel in image
	float xScale;
	///conversion between mm in world coordinate on the ground in y-direction and pixel in image
	float yScale;

	//set the IPM image width
	int width;
	//set the IPM image height
	int height;

	//portion of image height to add to y-coordinate of vanishing point
	float vpPortion;

	//initial vpPortion
	float vpInitPortion;

	//Left point in original image of region to make IPM for //ROI?
	float ipmLeft;
	///Right point in original image of region to make IPM for
	float ipmRight;
	///Top point in original image of region to make IPM for
	float ipmTop;
	///Bottom point in original image of region to make IPM for
	float ipmBottom;

	///interpolation to use for IPM (0: bilinear, 1:nearest neighbor) //双线性 0 和最近邻插值 1
	int ipmInterpolation;
};


class AdaptiveIPM
{
public:
	AdaptiveIPM() = delete;
	AdaptiveIPM(CameraParameters cam, IpmInfo info);
	~AdaptiveIPM() = default;

public:

	void SetVp(cv::Point pt);
	void SetCamera(CameraParameters cam);
	void SetIPMInfo(IpmInfo info);
	void GetVertices(std::vector<cv::Point2f>& vertices);
	void Transfer2IPM(cv::Mat &src, cv::Mat& dst);

	cv::Point2f GetVanishPoint(CameraParameters& cam);

	IpmInfo GetIIPMInfo() const;

	cv::Mat GetMaps() const 
	{
		return maps_.clone();
	}

private:
	void CalcIPMInfo();

	void TransformImage2Ground(const cv::Mat inPoints, cv::Mat outPoints, CameraParameters cameraInfo);
	void TransformGround2Image(const cv::Mat inPoints, cv::Mat outPoints, CameraParameters cameraInfo);

private:

	cv::Point2f vanish_point_;
	IpmInfo ipm_info_;
	CameraParameters camera_;
	cv::Mat maps_;
};

#endif //__ADAPTIVE_IPM_H__
