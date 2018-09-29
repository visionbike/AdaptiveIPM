#pragma once

#include "stdafx.h"

#include <iostream>
#include <string>
#include <vector>
#include "Camera.h"


using namespace std;
using namespace cv;


#define DEG2RAD 0.01745329252f
//constant definitions
#define FLOAT_MAT_TYPE CV_32FC1
#define FLOAT_MAT_ELEM_TYPE float

#define INT_MAT_TYPE CV_8UC1
#define INT_MAT_ELEM_TYPE unsigned char

#define FLOAT_IMAGE_TYPE IPL_DEPTH_32F
#define FLOAT_IMAGE_ELEM_TYPE float

#define INT_IMAGE_TYPE IPL_DEPTH_8U
#define INT_IMAGE_ELEM_TYPE unsigned char

#define FLOAT_POINT2D CvPoint2D32f
#define FLOAT_POINT2D_F cvPoint2D632f

#define FLOAT float
#define INT int
#define SHORT_INT unsigned char



typedef struct IPMInfo
{
	///min and max x-value on ground in world coordinates
	float xLimits[2];
	///min and max y-value on ground in world coordinates
	float yLimits[2];

	///conversion between mm in world coordinate on the ground in x-direction and pixel in image
	float xScale;
	///conversion between mm in world coordinate on the ground in y-direction and pixel in image
	float yScale;

	//set the IPM image width
	int _width;
	//set the IPM image height
	int _height;

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

	///interpolation to use for IPM (0: bilinear, 1:nearest neighbor) 双线性 0 和最近邻插值 1
	int ipmInterpolation;

};


class AdaptiveIPM
{

	typedef struct
	{
		string sLon;
		string sLat;
		string sAlt;
		string sPitch;
		string sRoll;
		string sYaw;
		string sOrien;

	} GPSandPose;

	typedef map<string, GPSandPose> GPSMap;

public:
	AdaptiveIPM();
	AdaptiveIPM(Camera & cam, string & strFile, string & strDir)
		:m_cam(cam), m_strGPSFile(strFile), m_strDir(strDir) {
		initIPMInfo();
	};
	~AdaptiveIPM();

public:

	void SetVp(Point & pt) { m_ptVP = pt; }
	void SetImage(Mat & img)
	{
		m_SrcImg = img;
		cvtColor(m_SrcImg, m_SrcImg, CV_BGR2GRAY);
		m_IpmImg.create(1024, 1024, CV_8U);//创建目标IPM图像大小为原图大小（可调整）
		m_ipmInfo.ipmBottom = m_SrcImg.rows;
		m_ipmInfo.ipmRight = m_SrcImg.cols;

	}
	void ipm_based_on_vp();

	void SetGPSandPoseFile(string &strFile) { m_strGPSFile = strFile; }
	void SetCamera(Camera & cam) { m_cam = cam; }
	void SetSrcImgDir(string & strDir)
	{
		m_strDir = strDir;
	}

	bool run();


	Mat GetIpm() { return m_IpmImg; }

protected:
	vector<string> split(const string &s, const string &seperator);
	void initIPMInfo();
	CvPoint2D32f getVanishPoint(Camera & cam);

	void transformImage2Ground(const CvMat *inPoints, CvMat *outPoints, Camera cameraInfo);
	void transformGround2Image(const CvMat *inPoints, CvMat *outPoints, Camera cameraInfo);

private:

	Point m_ptVP;
	Mat m_SrcImg;
	IPMInfo m_ipmInfo;
	Mat m_IpmImg;

	Camera m_cam;
	string m_strDir;

	vector <string> m_vecImgPaths;

	GPSMap m_MapGPSandPose;
	string m_strGPSFile;

};