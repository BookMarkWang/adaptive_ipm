#include "adaptive_ipm.h"
#include <fstream>

IpmInfo::IpmInfo() 
{
	ipmLeft = 0;
	ipmTop = 0;

	vpInitPortion = static_cast<float>(0.2);//0.02;  //初始化
	vpPortion = 0;// 0.13;		//随后更新的vpPortion
	ipmInterpolation = 0;  //双线性插值方法
}

AdaptiveIPM::AdaptiveIPM(CameraParameters cam, IpmInfo info) 
{
	camera_ = std::move(cam);
	ipm_info_ = std::move(info);
}

void AdaptiveIPM::SetVp(cv::Point pt)
{
	vanish_point_ = std::move(pt);
	CalcIPMInfo();
}

void AdaptiveIPM::SetCamera(CameraParameters cam)
{
	camera_ = std::move(cam);
}

void AdaptiveIPM::SetIPMInfo(IpmInfo info)
{ 
	ipm_info_ = std::move(info);
}

void AdaptiveIPM::CalcIPMInfo()
{
	//根据灭点来进行反投影变换
	cv::Point2f ptVP = vanish_point_;


	ptVP.y = MAX(0, ptVP.y);

	ipm_info_.vpPortion = ipm_info_.vpInitPortion;


	//float eps = ipm_info_.vpPortion * v;
	//int v = ipm_info_.height;
	//int u = ipm_info_.width;
	//std::cout << "eps" << eps << std::endl;
	//std::cout << "ptVP.y" << ptVP.y << std::endl;
	//std::cout<<eps<<std::endl;
	//ipm_info_.ipmLeft = MAX(0, ipm_info_.ipmLeft)+150;
	//ipm_info_.ipmRight = MIN(u - 1, ipm_info_.ipmRight)-150;
	//ipm_info_.ipmTop = ptVP.y  + eps - 150;// MAX(ptVanish.y+eps, ipm_info_.ipmTop);//动态转化大小
	//ipm_info_.ipmBottom = MIN(v - 1, ipm_info_.ipmBottom);

	//ipm_info_.ipmLeft = 0;
	//ipm_info_.ipmRight = ipm_info_.ipmRight;
	//ipm_info_.ipmTop = 1290;
	//ipm_info_.ipmBottom = ipm_info_.ipmBottom;
#ifdef DEBUG
	std::cout << "left:" << ipm_info_.ipmLeft << std::endl;
	std::cout << "right:" << ipm_info_.ipmRight << std::endl;
	std::cout << "top:" << ipm_info_.ipmTop << std::endl;
	std::cout << "bottom:" << ipm_info_.ipmBottom << std::endl;
#endif

	float uvLimitsp[] = { ptVP.x,				ipm_info_.ipmRight,		ipm_info_.ipmLeft,		ptVP.x,
		ipm_info_.ipmTop,	ipm_info_.ipmTop,		ipm_info_.ipmTop,		ipm_info_.ipmBottom };

	cv::Mat uvLimits(2, 4, CV_32FC1, uvLimitsp);
#ifdef DEBUG
	std::cout << "uvLimits:" << std::endl;
	std::cout << uvLimits << std::endl;
#endif

	//get these points on the ground plane
	cv::Mat xyLimits(2, 4, CV_32FC1);

	TransformImage2Ground(uvLimits, xyLimits, camera_);
#ifdef DEBUG
	std::cout << "xyLimits:" << std::endl;
	std::cout << xyLimits << std::endl;
#endif

	//get extent on the ground plane
	cv::Mat row1, row2;
	row1 = xyLimits.row(0);
	row2 = xyLimits.row(1);
	double xfMax, xfMin, yfMax, yfMin;
	minMaxLoc(row1, &xfMin, &xfMax, 0, 0);
	minMaxLoc(row2, &yfMin, &yfMax, 0, 0);

	int outRow =  ipm_info_.height; //设定512*512
	int outCol = ipm_info_.width;

	double stepRow = (yfMax - yfMin) / outRow;
	double stepCol = (xfMax - xfMin) / outCol;

	cv::Mat xyGrid(2, outRow * outCol, CV_32FC1);
	int i, j;
	double x, y;


	//fill it with x-y values on the ground plane in world frame
	for (i = 0, y = yfMax - .5 * stepRow; i < outRow; i++, y -= stepRow)
	{
		for (j = 0, x = xfMin + .5 * stepCol; j < outCol; j++, x += stepCol)
		{
			xyGrid.at<float>(0, i * outCol + j) = static_cast<float>(x);
			xyGrid.at<float>(1, i * outCol + j) = static_cast<float>(y);
		}
	}

	//get their pixel values in image frame //获取每个像素的真实像素值，创建输出2行，outRow*outCol大小的矩阵
	cv::Mat uvGrid(2, outRow * outCol, CV_32FC1);
	TransformGround2Image(xyGrid, uvGrid, camera_);

	maps_ = uvGrid.clone();

	ipm_info_.xLimits[0] = xyGrid.at<float>(0, 0);
	ipm_info_.xLimits[1] = xyGrid.at<float>(0, (outRow - 1) * outCol + outCol - 1);
	ipm_info_.yLimits[1] = xyGrid.at<float>(1, 0);
	ipm_info_.yLimits[0] = xyGrid.at<float>(1, (outRow - 1) * outCol + outCol - 1);
	ipm_info_.xScale = 1 / static_cast<float>(stepCol);
	ipm_info_.yScale = 1 / static_cast<float>(stepRow);
	ipm_info_.width = outCol;
	ipm_info_.height = outRow;
#ifdef DEBUG
	std::cout << "stepCol " << stepCol << std::endl;
	std::cout << "stepRow " << stepRow << std::endl;
#endif
}

void AdaptiveIPM::GetVertices(std::vector<cv::Point2f>& vertices)
{
	int outRow = ipm_info_.height;
	int outCol = ipm_info_.width;
	float ui, vi;

	int mid = ipm_info_.width / 2 - 1;
	int left_max = 0;
	int right_min = ipm_info_.width;

	for (int j = 0; j < outCol; j++)
	{
		/*get pixel coordiantes*/
		ui = maps_.at<float>(0, (outRow - 1) * outCol + j);
		vi = maps_.at<float>(1, (outRow - 1) * outCol + j);

		if (ui<ipm_info_.ipmLeft || ui>ipm_info_.ipmRight || vi<ipm_info_.ipmTop || vi>ipm_info_.ipmBottom)
		{
			if (j < mid && j > left_max) 
			{
				left_max = j;
			}
			else if (j > mid && j < right_min) 
			{
				right_min = j;
			}
		}
	}
	vertices.push_back(cv::Point2f(0, 0));
	vertices.push_back(cv::Point2f(static_cast<float>(ipm_info_.width), 0));
	vertices.push_back(cv::Point2f(static_cast<float>(right_min +1), static_cast<float>(ipm_info_.height)));
	vertices.push_back(cv::Point2f(static_cast<float>(left_max + 1), static_cast<float>(ipm_info_.height)));
}

void AdaptiveIPM::Transfer2IPM(cv::Mat& src, cv::Mat& dst) 
{
	int outRow = ipm_info_.height;
	int outCol = ipm_info_.width;
	float ui, vi;

	//get mean of the input image
	//cv::Scalar means = mean(src);

	float* ui_array = maps_.ptr<float>(0);
	float* vi_array = maps_.ptr<float>(1);

	for (int i = 0; i < outRow; i++)
	{
		for (int j = 0; j < outCol; j++)
		{
			/*get pixel coordiantes*/
			//ui = maps_.at<float>(0, i * outCol + j);
			//vi = maps_.at<float>(1, i * outCol + j);
			ui = ui_array[i * outCol + j];
			vi = vi_array[i * outCol + j];

			uchar* p = dst.ptr<uchar>(i);
			if (ui<ipm_info_.ipmLeft || ui>ipm_info_.ipmRight || vi<ipm_info_.ipmTop || vi>ipm_info_.ipmBottom)
			{
				//dst.at<cv::Vec3b>(i, j) = 0;// cv::Vec3b(means.val[0], means.val[1], means.val[2]);
				p[j * 3 + 0] = 0;
				p[j * 3 + 1] = 0;
				p[j * 3 + 2] = 0;
			}
			/*not out of bounds, then get nearest neighbor*/
			else
			{
				/*Bilinear interpolation 双线性插值*/
				if (ipm_info_.ipmInterpolation == 0)
				{
					int x1 = int(ui);
					int x2 = int(ui + 0.5);
					int y1 = int(vi);
					int y2 = int(vi + 0.5);
					float x = ui - x1;
					float y = vi - y1;
					uchar* y1_array = src.ptr<uchar>(y1);
					p[j * 3 + 0] += static_cast<uchar>(y1_array[x1 * 3 + 0] * (1 - x) * (1 - y));
					p[j * 3 + 1] += static_cast<uchar>(y1_array[x1 * 3 + 1] * (1 - x) * (1 - y));
					p[j * 3 + 2] += static_cast<uchar>(y1_array[x1 * 3 + 2] * (1 - x) * (1 - y));
					p[j * 3 + 0] += static_cast<uchar>(y1_array[x2 * 3 + 0] * x * (1 - y));
					p[j * 3 + 1] += static_cast<uchar>(y1_array[x2 * 3 + 1] * x * (1 - y));
					p[j * 3 + 2] += static_cast<uchar>(y1_array[x2 * 3 + 2] * x * (1 - y));
					uchar* y2_array = src.ptr<uchar>(y2);
					p[j * 3 + 0] += static_cast<uchar>(y2_array[x1 * 3 + 0] * (1 - x) * y);
					p[j * 3 + 1] += static_cast<uchar>(y2_array[x1 * 3 + 1] * (1 - x) * y);
					p[j * 3 + 2] += static_cast<uchar>(y2_array[x1 * 3 + 2] * (1 - x) * y);
					p[j * 3 + 0] += static_cast<uchar>(y2_array[x2 * 3 + 0] * x * y);
					p[j * 3 + 1] += static_cast<uchar>(y2_array[x2 * 3 + 1] * x * y);
					p[j * 3 + 2] += static_cast<uchar>(y2_array[x2 * 3 + 2] * x * y);
					//cv::Vec3b val = src.at<cv::Vec3b>(y1, x1) * (1 - x) * (1 - y) + src.at<cv::Vec3b>(y1, x2) * x * (1 - y) + src.at<cv::Vec3b>(y2, x1) * (1 - x) * y + src.at<cv::Vec3b>(y2, x2) * x * y;
					//dst.at<cv::Vec3b>(i, j) = val;
				}
				/*nearest-neighbor interpolation最近邻插值*/
				else
				{
					int x2 = int(ui + 0.5);
					uchar* y1_array = src.ptr<uchar>(int(vi + .5));
					p[j * 3 + 0] = y1_array[x2 * 3 + 0];
					p[j * 3 + 1] = y1_array[x2 * 3 + 1];
					p[j * 3 + 2] = y1_array[x2 * 3 + 2];
					//dst.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(int(vi + .5), int(ui + .5));
				}
			}
		}
	}
}

cv::Point2f AdaptiveIPM::GetVanishPoint(CameraParameters& cam)
{

	//检查到 struct 的初始化
	float vpp[] = { static_cast<float>(std::sin( cam.GetYaw()) / std::cos(cam.GetPitch())),
		static_cast<float>(std::cos(cam.GetYaw()) / std::cos(cam.GetPitch())),
		0 };
	cv::Mat vp(3, 1, CV_32FC1, vpp);

	//rotation matrix for yaw
	float tyawp[] = { static_cast<float>(cos(cam.GetYaw())),static_cast<float>(-sin(cam.GetYaw())),0,
		static_cast<float>(sin(cam.GetYaw())),static_cast<float>(cos(cam.GetYaw())),0,
		0,0,1 };
	cv::Mat tyaw(3, 3, CV_32FC1, tyawp);

	//rotation matrix for pitch
	float tpitchp[] = { 1,0,0,
		0,static_cast<float>(-sin(cam.GetPitch())),static_cast<float>(-cos(cam.GetPitch())),
		0,static_cast<float>(cos(cam.GetPitch())),static_cast<float>(-sin(cam.GetPitch())) };
	cv::Mat transform(3, 3, CV_32FC1, tpitchp);

	transform = tyaw * transform;

	//matrix to shift optical center and focal length
	float tlp[] = { static_cast<float>(cam.GetFx()),       0,     static_cast<float>(cam.GetCx()),
		0,       static_cast<float>(cam.GetFy()),     static_cast<float>(cam.GetCy()),
		0,              0,            1 };


	cv::Mat tl(3, 3, CV_32FC1, tlp);
	//combine transform
	transform = tl * (transform);

	//transform
	vp = transform * (vp);

	return cv::Point2f(vp.at<float>(0, 0), vp.at<float>(1, 0));
}

IpmInfo AdaptiveIPM::GetIIPMInfo() const
{
	return ipm_info_;
}


void AdaptiveIPM::TransformImage2Ground(const cv::Mat inPoints, cv::Mat outPoints, CameraParameters cameraInfo)
{

	//add two rows to the input points
	cv::Mat inPoints4 = cv::Mat::zeros(inPoints.rows + 2, inPoints.cols, inPoints.type());
#ifdef DEBUG
	std::cout << "inPoints4:" << std::endl;
	std::cout << inPoints4 << std::endl;
#endif

	//copy inPoints to first two rows
	cv::Mat inPoints2, inPoints3, inPointsr4, inPointsr3;

	inPoints2 = inPoints4.rowRange(0, 2);
	inPoints3 = inPoints4.rowRange(0, 3);
	inPointsr3 = inPoints4.row(2);
	inPointsr4 = inPoints4.row(3);
#ifdef DEBUG
	std::cout << "inPoints3:" << std::endl;
	std::cout << inPoints3 << std::endl;
#endif

	inPointsr3.setTo(cv::Scalar(1));
	inPoints.copyTo(inPoints2);
#ifdef DEBUG
	std::cout << "inPoints3:" << std::endl;
	std::cout << inPoints3 << std::endl;
#endif

	//create the transformation matrix
	double c1 = cos(cameraInfo.GetPitch());
	double s1 = sin(cameraInfo.GetPitch());
	double c2 = cos(cameraInfo.GetYaw());
	double s2 = sin(cameraInfo.GetYaw());
#ifdef DEBUG
	std::cout << "pitch = " << cameraInfo.GetPitch() << std::endl;
	std::cout << "yaw = " << cameraInfo.GetYaw() << std::endl;
	std::cout << "c1 = " << c1 << std::endl;
	std::cout << "s1 = " << s1 << std::endl;
	std::cout << "c2 = " << c2 << std::endl;
	std::cout << "s2 = " << s2 << std::endl;
	std::cout << "Fx = " << cameraInfo.GetFx() << std::endl;
	std::cout << "Fy = " << cameraInfo.GetFy() << std::endl;
	std::cout << "Cx = " << cameraInfo.GetCx() << std::endl;
	std::cout << "Cy = " << cameraInfo.GetCy() << std::endl;
	std::cout << "Hi = " << cameraInfo.GetHeight() << std::endl;
#endif
	float matp[] = {
		static_cast<float>(-cameraInfo.GetHeight() * c2 / cameraInfo.GetFx()),
		static_cast<float>(cameraInfo.GetHeight() * s1 * s2 / cameraInfo.GetFy()),
		static_cast<float>((cameraInfo.GetHeight() * c2 * cameraInfo.GetCx() /
		cameraInfo.GetFx()) - (cameraInfo.GetHeight() * s1 * s2 * cameraInfo.GetCy() /
			cameraInfo.GetFy()) - cameraInfo.GetHeight() * c1 * s2),

		static_cast<float>(cameraInfo.GetHeight() * s2 / cameraInfo.GetFx()),
		static_cast<float>(cameraInfo.GetHeight() * s1 * c2 / cameraInfo.GetFy()),
		static_cast<float>((-cameraInfo.GetHeight() * s2 * cameraInfo.GetCx()
			/ cameraInfo.GetFx()) - (cameraInfo.GetHeight() * s1 * c2 *
				cameraInfo.GetCy() / cameraInfo.GetFy()) -
		cameraInfo.GetHeight() * c1 * c2),

		0,		static_cast<float>(cameraInfo.GetHeight() * c1 / cameraInfo.GetFy()),
		static_cast<float>((-cameraInfo.GetHeight() * c1 * cameraInfo.GetCy() /
			cameraInfo.GetFy()) + cameraInfo.GetHeight() * s1),

		0,		static_cast<float>(-c1 / cameraInfo.GetFy()),static_cast<float>((c1 * cameraInfo.GetCy() / cameraInfo.GetFy()) - s1),
	};

	cv::Mat mat(4, 3, CV_32FC1, matp);
#ifdef DEBUG
	std::cout << "mat:" << std::endl;
	std::cout << mat << std::endl;
	std::cout << "inPoints3:" << std::endl;
	std::cout << inPoints3 << std::endl;
#endif
	//multiply
	inPoints4 = mat * (inPoints3);
#ifdef DEBUG
	std::cout << "inPoints3:" << std::endl;
	std::cout << inPoints3 << std::endl;
#endif

	//divide by last row of inPoints4
	for (int i = 0; i < inPoints.cols; i++)
	{
		float div = inPointsr4.at<float>(0, i);
		inPoints4.at<float>(0, i) = inPoints4.at<float>(0, i) / div;
		inPoints4.at<float>(1, i) = inPoints4.at<float>(1, i) / div;
	}
	//put back the result into outPoints
	inPoints2.copyTo(outPoints);

}



void AdaptiveIPM::TransformGround2Image(const cv::Mat inPoints, cv::Mat outPoints, CameraParameters cameraInfo)
{
	//add two rows to the input points
	cv::Mat inPoints3(inPoints.rows + 1, inPoints.cols, inPoints.type());

	//copy inPoints to first two rows
	cv::Mat inPoints2, inPointsr3;
	inPoints2 = inPoints3.rowRange(0, 2);
	inPointsr3 = inPoints3.row(2);

	inPointsr3.setTo(cv::Scalar(-cameraInfo.GetHeight()));
	inPoints.copyTo(inPoints2);
	//create the transformation matrix
	double c1 = cos(cameraInfo.GetPitch());
	double s1 = sin(cameraInfo.GetPitch());
	double c2 = cos(cameraInfo.GetYaw());
	double s2 = sin(cameraInfo.GetYaw());
	float matp[] = {
		static_cast<float>(cameraInfo.GetFx() * c2 + c1 * s2 * cameraInfo.GetCx()),
		static_cast<float>(-cameraInfo.GetFx() * s2 + c1 * c2 * cameraInfo.GetCx()),
		static_cast<float>(-s1 * cameraInfo.GetCx()),

		static_cast<float>(s2 * (-cameraInfo.GetFy() * s1 + c1 * cameraInfo.GetCy())),
		static_cast<float>(c2 * (-cameraInfo.GetFy() * s1 + c1 * cameraInfo.GetCy())),
		static_cast<float>(-cameraInfo.GetFy() * c1 - s1 * cameraInfo.GetCy()),

		static_cast<float>(c1 * s2),
		static_cast<float>(c1 * c2),
		static_cast<float>(-s1)
	};
	cv::Mat mat(3, 3, CV_32FC1, matp);
	//multiply
	inPoints3 = mat * inPoints3;
	//divide by last row of inPoints4
	for (int i = 0; i < inPoints.cols; i++)
	{
		float div = inPointsr3.at<float>(0, i);
		inPoints3.at<float>(0, i) = inPoints3.at<float>(0, i) / div;
		inPoints3.at<float>(1, i) = inPoints3.at<float>(1, i) / div;
	}
	//put back the result into outPoints
	inPoints2.copyTo(outPoints);

}



