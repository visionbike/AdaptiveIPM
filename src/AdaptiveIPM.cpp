
#include "stdafx.h"
#include "AdaptiveIPM.h"

AdaptiveIPM::AdaptiveIPM()
{
	initIPMInfo();
}

AdaptiveIPM::~AdaptiveIPM()
{

}

void AdaptiveIPM::initIPMInfo()
{
	m_ipmInfo.ipmLeft = 0;
	m_ipmInfo.ipmTop = 0;

	m_ipmInfo.vpInitPortion = 0.02;  //初始化
	m_ipmInfo.vpPortion = 0.13;		//随后更新的vpPortion
	m_ipmInfo.ipmInterpolation = 0;  //双线性插值方法

}

void AdaptiveIPM::ipm_based_on_vp()
{
	//根据灭点来进行反投影变换
	Point2d ptVP = m_ptVP;
	Mat imgSrc = m_SrcImg;

	int v = imgSrc.rows;
	int u = imgSrc.cols;

	ptVP.y = MAX(0, ptVP.y);

	m_ipmInfo.vpPortion = m_ipmInfo.vpInitPortion;


	FLOAT_MAT_ELEM_TYPE eps = m_ipmInfo.vpPortion * v;
	//cout<<eps<<endl;
	m_ipmInfo.ipmLeft = MAX(0, m_ipmInfo.ipmLeft);
	m_ipmInfo.ipmRight = MIN(u - 1, m_ipmInfo.ipmRight);
	m_ipmInfo.ipmTop = ptVP.y + eps;// MAX(ptVanish.y+eps, m_ipmInfo.ipmTop);//动态转化大小
	m_ipmInfo.ipmBottom = MIN(v - 1, m_ipmInfo.ipmBottom);

	FLOAT_MAT_ELEM_TYPE uvLimitsp[] = { ptVP.x,				m_ipmInfo.ipmRight,		m_ipmInfo.ipmLeft,		ptVP.x,
		m_ipmInfo.ipmTop,	m_ipmInfo.ipmTop,		m_ipmInfo.ipmTop,		m_ipmInfo.ipmBottom };

	CvMat uvLimits = cvMat(2, 4, FLOAT_MAT_TYPE, uvLimitsp);

	//get these points on the ground plane
	CvMat *xyLimitsp = cvCreateMat(2, 4, FLOAT_MAT_TYPE);
	CvMat xyLimits = *xyLimitsp;

	transformImage2Ground(&uvLimits, &xyLimits, m_cam);

	//get extent on the ground plane
	CvMat row1, row2;
	cvGetRow(&xyLimits, &row1, 0);
	cvGetRow(&xyLimits, &row2, 1);
	double xfMax, xfMin, yfMax, yfMin;
	cvMinMaxLoc(&row1, (double*)&xfMin, (double*)&xfMax, 0, 0, 0);
	cvMinMaxLoc(&row2, (double*)&yfMin, (double*)&yfMax, 0, 0, 0);

	int outRow = m_IpmImg.rows; //设定512*512
	int outCol = m_IpmImg.cols;

	//cout<<"x:"<<xfMax<<" "<<xfMin<<endl<<"y:"<<yfMax<<" "<<yfMin<<endl;
	FLOAT_MAT_ELEM_TYPE stepRow = (yfMax - yfMin) / outRow;
	FLOAT_MAT_ELEM_TYPE stepCol = (xfMax - xfMin) / outCol;

	CvMat *xyGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
	INT i, j;
	FLOAT_MAT_ELEM_TYPE x, y;


	//fill it with x-y values on the ground plane in world frame
	for (i = 0, y = yfMax - .5*stepRow; i < outRow; i++, y -= stepRow)
	{
		for (j = 0, x = xfMin + .5*stepCol; j < outCol; j++, x += stepCol)
		{
			//cout<<"x:"<<x<<"y:"<<y<<endl;
			CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol + j) = x;
			CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol + j) = y;
		}
	}


	//get their pixel values in image frame //获取每个像素的真实像素值，创建输出2行，outRow*outCol大小的矩阵
	CvMat *uvGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
	transformGround2Image(xyGrid, uvGrid, m_cam);
	//now loop and find the nearest pixel value for each position
	//that's inside the image, otherwise put it zero
	FLOAT_MAT_ELEM_TYPE ui, vi;

	//get mean of the input image
	//cout<<m_srcImg.type()<<endl;

	CvMat *inImage = cvCreateMat(imgSrc.rows, imgSrc.cols, imgSrc.type());
	CvMat tempImg = imgSrc;
	cvCopy(&tempImg, inImage);
	CvScalar means = cvAvg(inImage);
	double mean = means.val[0];

	for (i = 0; i < outRow; i++)
	{
		for (j = 0; j < outCol; j++)
		{
			/*get pixel coordiantes*/
			ui = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol + j);
			vi = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol + j);

			if (ui<m_ipmInfo.ipmLeft || ui>m_ipmInfo.ipmRight || vi<m_ipmInfo.ipmTop || vi>m_ipmInfo.ipmBottom)
			{
				m_IpmImg.at<uchar>(i, j) = 0;
			}
			/*not out of bounds, then get nearest neighbor*/
			else
			{
				/*Bilinear interpolation 双线性插值*/
				if (m_ipmInfo.ipmInterpolation == 0)
				{
					int x1 = int(ui);
					int x2 = int(ui + 0.5);
					int y1 = int(vi);
					int y2 = int(vi + 0.5);
					float x = ui - x1;
					float y = vi - y1;
					float val = CV_MAT_ELEM(*inImage, uchar, y1, x1) * (1 - x) * (1 - y) + CV_MAT_ELEM(*inImage, uchar, y1, x2) * x * (1 - y) + CV_MAT_ELEM(*inImage, uchar, y2, x1) * (1 - x) * y + CV_MAT_ELEM(*inImage, uchar, y2, x2) * x * y;
					m_IpmImg.at<uchar>(i, j) = (float)val;
				}
				/*nearest-neighbor interpolation最近邻插值*/
				else
				{
					m_IpmImg.at<uchar>(i, j) = CV_MAT_ELEM(*inImage, uchar, int(vi + .5), int(ui + .5));
				}
			}
		}
	}


	m_ipmInfo.xLimits[0] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, 0);
	m_ipmInfo.xLimits[1] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, (outRow - 1)*outCol + outCol - 1);
	m_ipmInfo.yLimits[1] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, 0);
	m_ipmInfo.yLimits[0] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, (outRow - 1)*outCol + outCol - 1);
	m_ipmInfo.xScale = 1 / stepCol;
	m_ipmInfo.yScale = 1 / stepRow;
	m_ipmInfo._width = outCol;
	m_ipmInfo._height = outRow;

	cout << stepCol << endl;
	cout << stepRow << endl;

	//clean
	cvReleaseMat(&xyLimitsp);
	cvReleaseMat(&xyGrid);
	cvReleaseMat(&uvGrid);
	cvReleaseMat(&inImage);

}

bool AdaptiveIPM::run()
{
	if (m_strDir.empty() || m_strGPSFile.empty())
	{
		return false;
	}


	//m_vecImgPaths = fc::listCurFiles(m_strDir);


	//解析GPS And Pose
	ifstream iGPS(m_strGPSFile.c_str());

	if (!iGPS.is_open())
	{
		return false;
	}

	string strGPSLine;
	GPSandPose gpsPose;

	//计数
	int i = 0;

	//原始图像
	//*****逐行解析GPS ，加载对应影像****//
	while (getline(iGPS, strGPSLine))
	{

		vector<string> vecGPSLine = split(strGPSLine, " ");

		gpsPose.sLon = vecGPSLine[9];// .c_str();
		gpsPose.sLat = vecGPSLine[8];// .c_str();
		gpsPose.sAlt = vecGPSLine[7];// .c_str();
		gpsPose.sRoll = vecGPSLine[10];// .c_str();
		gpsPose.sPitch = vecGPSLine[11];// .c_str();
		gpsPose.sYaw = vecGPSLine[12];// .c_str();

		m_cam.SetCx(1651.52);
		m_cam.SetCy(2496.88);
		m_cam.SetFx(1695.11);
		m_cam.SetFy(1699.32);
		m_cam.SetPitch(-atof(gpsPose.sPitch.c_str())* DEG2RAD);
		m_cam.SetRoll(atof(gpsPose.sRoll.c_str()) * DEG2RAD);
		m_cam.SetYaw(atof(gpsPose.sYaw.c_str()) * DEG2RAD);
		string strImgPath = m_vecImgPaths[i];
		i++;
		Mat imgSrc = imread(strImgPath.c_str());

		//根据灭点进行自适应反投影变换
		CvPoint2D32f ptVp = getVanishPoint(m_cam);

		cout << ptVp.x << " " << ptVp.y << endl;

		line(imgSrc, Point(ptVp.x + 50, ptVp.y), Point(ptVp.x - 50, ptVp.y), Scalar(0, 0, 255), 8);
		line(imgSrc, Point(ptVp.x, ptVp.y + 50), Point(ptVp.x, ptVp.y - 50), Scalar(0, 0, 255), 8);

		resize(imgSrc, imgSrc, Size(1200, 700));
		namedWindow("ipm", WINDOW_FREERATIO);
		imshow("ipm", imgSrc);
		waitKey(0);

	}

	return true;
}


CvPoint2D32f AdaptiveIPM::getVanishPoint(Camera & cam)
{

	//检查到 struct 的初始化
	float vpp[] = { std::sin(cam.m_YawAngle) / std::cos(cam.m_PitchAngle),
		std::cos(cam.m_YawAngle) / std::cos(cam.m_PitchAngle),
		0 };

	CvMat vp = cvMat(3, 1, CV_32FC1, vpp);
	//rotation matrix for yaw
	FLOAT_MAT_ELEM_TYPE tyawp[] = { cos(cam.m_YawAngle),-sin(cam.m_YawAngle),0,
		sin(cam.m_YawAngle),cos(cam.m_YawAngle),0,
		0,0,1 };
	CvMat tyaw = cvMat(3, 3, FLOAT_MAT_TYPE, tyawp);

	//rotation matrix for pitch
	FLOAT_MAT_ELEM_TYPE tpitchp[] = { 1,0,0,
		0,-sin(cam.m_PitchAngle),-cos(cam.m_PitchAngle),
		0,cos(cam.m_PitchAngle),-sin(cam.m_PitchAngle) };
	CvMat transform = cvMat(3, 3, FLOAT_MAT_TYPE, tpitchp);
	cvMatMul(&transform, &tyaw, &transform);

	//matrix to shift optical center and focal length
	FLOAT_MAT_ELEM_TYPE tlp[] = { cam.m_Fx,       0,     cam.m_Cx,
		0,       cam.m_Fy,     cam.m_Cy,
		0,              0,            1 };

	CvMat tl = cvMat(3, 3, FLOAT_MAT_TYPE, tlp);
	//combine transform
	cvMatMul(&tl, &transform, &transform);
	//transform
	cvMatMul(&transform, &vp, &vp);

	//clean and return
	FLOAT_POINT2D resultPoint;
	resultPoint.x = cvGetReal1D(&vp, 0);
	resultPoint.y = cvGetReal1D(&vp, 1);

	//cout<<"the Vanish Point is:"<<endl;
	//cout<<"x："<<resultPoint.x<<endl<<"y："<<resultPoint.y<<endl;
	return resultPoint;
}


void AdaptiveIPM::transformImage2Ground(const CvMat *inPoints, CvMat *outPoints, Camera cameraInfo)
{

	//add two rows to the input points
	CvMat *inPoints4 = cvCreateMat(inPoints->rows + 2, inPoints->cols, cvGetElemType(inPoints));

	//copy inPoints to first two rows
	CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;

	cvGetRows(inPoints4, &inPoints2, 0, 2); //起始行，行数
	cvGetRows(inPoints4, &inPoints3, 0, 3);
	cvGetRow(inPoints4, &inPointsr3, 2);
	cvGetRow(inPoints4, &inPointsr4, 3);
	cvSet(&inPointsr3, cvRealScalar(1));
	cvCopy(inPoints, &inPoints2);
	//create the transformation matrix
	float c1 = cos(cameraInfo.m_PitchAngle);
	float s1 = sin(cameraInfo.m_PitchAngle);
	float c2 = cos(cameraInfo.m_YawAngle);
	float s2 = sin(cameraInfo.m_YawAngle);
	float matp[] = {
		-cameraInfo.m_Height*c2 / cameraInfo.m_Fx,
		cameraInfo.m_Height*s1*s2 / cameraInfo.m_Fy,
		(cameraInfo.m_Height*c2*cameraInfo.m_Cx /
		cameraInfo.m_Fx) - (cameraInfo.m_Height *s1*s2* cameraInfo.m_Cy /
			cameraInfo.m_Fy) - cameraInfo.m_Height *c1*s2,

		cameraInfo.m_Height *s2 / cameraInfo.m_Fx,
		cameraInfo.m_Height *s1*c2 / cameraInfo.m_Fy,
		(-cameraInfo.m_Height *s2* cameraInfo.m_Cx
			/ cameraInfo.m_Fx) - (cameraInfo.m_Height *s1*c2*
				cameraInfo.m_Cy / cameraInfo.m_Fy) -
		cameraInfo.m_Height *c1*c2,

		0,		cameraInfo.m_Height *c1 / cameraInfo.m_Fy,
		(-cameraInfo.m_Height *c1* cameraInfo.m_Cy /
			cameraInfo.m_Fy) + cameraInfo.m_Height *s1,

		0,		-c1 / cameraInfo.m_Fy,(c1* cameraInfo.m_Cy / cameraInfo.m_Fy) - s1,
	};

	CvMat mat = cvMat(4, 3, CV_32FC1, matp);
	//multiply
	cvMatMul(&mat, &inPoints3, inPoints4);
	//divide by last row of inPoints4
	for (int i = 0; i < inPoints->cols; i++)
	{
		float div = CV_MAT_ELEM(inPointsr4, float, 0, i);
		CV_MAT_ELEM(*inPoints4, float, 0, i) = CV_MAT_ELEM(*inPoints4, float, 0, i) / div;
		CV_MAT_ELEM(*inPoints4, float, 1, i) = CV_MAT_ELEM(*inPoints4, float, 1, i) / div;
	}
	//put back the result into outPoints
	cvCopy(&inPoints2, outPoints);
	//clear
	cvReleaseMat(&inPoints4);

}



void AdaptiveIPM::transformGround2Image(const CvMat *inPoints, CvMat *outPoints, Camera cameraInfo)
{
	//add two rows to the input points
	CvMat *inPoints3 = cvCreateMat(inPoints->rows + 1, inPoints->cols,
		cvGetElemType(inPoints));

	//copy inPoints to first two rows
	CvMat inPoints2, inPointsr3;
	cvGetRows(inPoints3, &inPoints2, 0, 2);
	cvGetRow(inPoints3, &inPointsr3, 2);
	cvSet(&inPointsr3, cvRealScalar(-cameraInfo.m_Height));
	cvCopy(inPoints, &inPoints2);
	//create the transformation matrix
	float c1 = cos(cameraInfo.m_PitchAngle);
	float s1 = sin(cameraInfo.m_PitchAngle);
	float c2 = cos(cameraInfo.m_YawAngle);
	float s2 = sin(cameraInfo.m_YawAngle);
	float matp[] = {
		cameraInfo.m_Fx * c2 + c1 * s2* cameraInfo.m_Cx,
		-cameraInfo.m_Fx * s2 + c1 * c2* cameraInfo.m_Cx,
		-s1 * cameraInfo.m_Cx,

		s2 * (-cameraInfo.m_Fy * s1 + c1 * cameraInfo.m_Cy),
		c2 * (-cameraInfo.m_Fy * s1 + c1 * cameraInfo.m_Cy),
		-cameraInfo.m_Fy * c1 - s1 * cameraInfo.m_Cy,

		c1*s2,
		c1*c2,
		-s1
	};
	CvMat mat = cvMat(3, 3, CV_32FC1, matp);
	//multiply
	cvMatMul(&mat, inPoints3, inPoints3);
	//divide by last row of inPoints4
	for (int i = 0; i < inPoints->cols; i++)
	{
		float div = CV_MAT_ELEM(inPointsr3, float, 0, i);
		CV_MAT_ELEM(*inPoints3, float, 0, i) = CV_MAT_ELEM(*inPoints3, float, 0, i) / div;
		CV_MAT_ELEM(*inPoints3, float, 1, i) = CV_MAT_ELEM(*inPoints3, float, 1, i) / div;
	}
	//put back the result into outPoints
	cvCopy(&inPoints2, outPoints);
	//clear
	cvReleaseMat(&inPoints3);

}


vector<string> AdaptiveIPM::split(const string &s, const string &seperator) {
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		//找到字符串中首个不等于分隔符的字母；
		int flag = 0;
		while (i != s.size() && flag == 0) {
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[i] == seperator[x]) {
					++i;
					flag = 0;
					break;
				}
		}

		//找到又一个分隔符，将两个分隔符之间的字符串取出；
		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0) {
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[j] == seperator[x]) {
					flag = 1;
					break;
				}
			if (flag == 0)
				++j;
		}
		if (i != j) {
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}
	return result;
}


