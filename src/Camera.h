#pragma once

#include "stdafx.h"


class Camera
{

public:
	Camera();
	~Camera();

public:

	void SetCamera(Camera & cam);
	Camera operator= (Camera & cam);

	void SetK1(const double k1) { m_K1 = k1; }
	void SetK2(const double k2) { m_K2 = k2; }
	void SetK3(const double k3) { m_K3 = k3; }
	void SetK4(const double k4) { m_K4 = k4; }
	void SetP1(const double p1) { m_P1 = p1; }
	void SetP2(const double p2) { m_P2 = p2; }
	void SetFx(const double fx) { m_Fx = fx; }
	void SetFy(const double fy) { m_Fy = fy; }
	void SetCx(const double cx) { m_Cx = cx; }
	void SetCy(const double cy) { m_Cy = cy; }
	void SetRoll(const double roll) { m_RollAngle = roll; }
	void SetYaw(const double yaw) { m_YawAngle = yaw; }
	void SetPitch(const double pitch) { m_PitchAngle = pitch; }
	void SetHeight(const double height) { m_Height = height; }

private:
	void initCamera();




public:
	//内参
	double m_K1;
	double m_K2;
	double m_K3;
	double m_K4;
	double m_P1;
	double m_P2;

	double m_Fx;
	double m_Fy;

	double m_Cx;
	double m_Cy;

	double m_RollAngle;
	double m_YawAngle;
	double m_PitchAngle;
	//外参
	double m_Height;

private:
	std::string m_strCameraName;

};
