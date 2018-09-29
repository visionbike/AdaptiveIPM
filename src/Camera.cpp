
#include "stdafx.h"
#include "Camera.h"

using namespace std;

Camera::Camera()
{
	initCamera();
}

Camera::~Camera()
{

}

void Camera::initCamera()
{
	m_Cx = 639.5;
	m_Cy = 359.5;
	m_Fx = 1231.55;
	m_Fy = 1231.55;
	m_K1 = -0.28771449;
	m_K2 = 0.02502772;
	m_K3 = 0;
	m_K4 = 0;
	m_P1 = 0;
	m_P2 = 0;

	m_RollAngle = 0;
	m_YawAngle = 0;
	m_PitchAngle = 0;

	m_Height = 1450;//mm

}


void Camera::SetCamera(Camera & cam)
{

	this->m_Cx = cam.m_Cx;
	this->m_Cy = cam.m_Cy;
	this->m_Fx = cam.m_Fx;
	this->m_Fy = cam.m_Fy;

	this->m_Height = cam.m_Height;

	this->m_K1 = cam.m_K1;
	this->m_K2 = cam.m_K2;
	this->m_P1 = cam.m_P1;
	this->m_P2 = cam.m_P2;
	this->m_K3 = cam.m_K3;
	this->m_K4 = cam.m_K4;

	this->m_RollAngle = cam.m_RollAngle;
	this->m_YawAngle = cam.m_YawAngle;
	this->m_PitchAngle = cam.m_PitchAngle;
}



Camera Camera::operator=(Camera & cam)
{
	this->m_Cx = cam.m_Cx;
	this->m_Cy = cam.m_Cy;
	this->m_Fx = cam.m_Fx;
	this->m_Fy = cam.m_Fy;

	this->m_Height = cam.m_Height;

	this->m_K1 = cam.m_K1;
	this->m_K2 = cam.m_K2;
	this->m_P1 = cam.m_P1;
	this->m_P2 = cam.m_P2;
	this->m_K3 = cam.m_K3;
	this->m_K4 = cam.m_K4;

	this->m_RollAngle = cam.m_RollAngle;
	this->m_YawAngle = cam.m_YawAngle;
	this->m_PitchAngle = cam.m_PitchAngle;

	return *this;
}