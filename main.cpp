#include "adaptive_ipm.h"

int main(int argc, char** argv) 
{

	cv::Matx33d camera_matrix = cv::Matx33d(
		1431.002979924706600, 0., 1218.826159997833700,
		0., 1428.665155204724400, 1026.414146259616900,
		0., 0., 1.);
	cv::Vec4d distortion_coeffs = cv::Vec4d(-0.051456953321389, 0.071852289379636, 0.001291245731554, 0.000738842703160);
	CameraParameters camera(camera_matrix, distortion_coeffs);
	camera.SetP1(0.001186043659633);
	camera.SetHeight(3000);
	camera.SetPitch(16.9);
	camera.SetRoll(0);
	camera.SetYaw(0);


	//set the ipm info
	IpmInfo info;
	info.width = 1280;
	info.height = 1280;
	info.ipmBottom = 2047;
	info.ipmRight = 2447;
	info.ipmTop = 1220;
	info.ipmLeft = 0;
	AdaptiveIPM ipm(camera, info);

	//set vanishpoint
	cv::Point vp = ipm.GetVanishPoint(camera);
	std::cout << "vp = " << vp << std::endl;
	ipm.SetVp(vp);

	cv::Mat dst,test;
	test = cv::imread("ipm_para_source.jpg");
	dst = cv::Mat::zeros(cv::Size(info.width, info.height), CV_8UC3);
	ipm.Transfer2IPM(test, dst);

	cv::imshow("result", dst);
	cv::waitKey(0);

	return 0;
}
