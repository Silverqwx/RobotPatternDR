#pragma once
#include <math.h>
#include "PixelLineSegment.h"
#include "AIV_typedefine_lxl20200624.h"


//////////#define _CRTDBG_MAP_ALLOC
//////////#include <stdlib.h>
//////////#include <crtdbg.h>



class LineGenerator
{
public:
	LineGenerator(int w, int h);
	LineGenerator();
public:
	~LineGenerator(void);
public:
	int MapWidth, MapHeight;
public:
	int TwoEnds(int u1, int v1, int u2, int v2, int* SavePosition);//以两个端点为输入，生成线段各像素点坐标，存入数组,每个单元三位，两个图像坐标和相应的一维坐标
	int TwoEnds_Left2Right(int u1, int v1, int u2, int v2, int* SavePosition);//以两个端点为输入，生成线段各像素点坐标，数据存储顺序从左到右,每个单元三位
	int TwoEnds_Left2Right_1C(int u1, int v1, int u2, int v2, int* SavePosition);//以两个端点为输入，生成线段各像素点坐标，数据存储顺序从左到右,每个单元三位
	int TwoEnds_Sym(int u1, int v1, int u2, int v2, int* SavePosition);//以两个端点为输入，以线段中点为起点，左右点交替存储，形成对称线段偏移量数组
	int TwoEnds_Sym(float length, int u1, int v1, int u2, int v2, int* SavePosition);//重载，指定生成线段的长度
	int RelTwoEnds(int ustart, int vstart, int udes, int vdes, float length, int* SavePosition);//以两个端点为输入，生成从点1指向点2的线段的坐标偏移量，指定长度length
	int RelTwoEndsCenter(float length, int u1, int v1, int u2, int v2, int* SavePosition);//以两个端点为输入生成两侧型的线段，指定长度length
	int RayLength(int u, int v, float length, float angle, int* SavePosition);//从指定起点生成指定朝向和长度的射线各像素点坐标，存入数组
	int RayEnds(int us, int vs, int up, int vp, float length, int* SavePosition);//给定起点生成经过指定点的线段，存入数组
	int VerSegmentF(float length, float A, float B, int* SavePosition);//生成与给定直线垂直的线段坐标,给定直线方程
	int VerSegmentF_Sym(float length, float A, float B, int* SavePosition);//生成与给定直线垂直的线段坐标,给定直线方程
	int VerSegment(float length, int u1, int v1, int u2, int v2, int* SavePosition);//生成与给定直线垂直的线段坐标,给定直线经过的两点
	int VerSegment_Sym(float length, int u1, int v1, int u2, int v2, int* SavePosition);//生成与给定直线垂直的线段坐标,给定直线经过的两点,线段偏移量对称分布
	int ParSegmentF(float length, float A, float B, int* SavePosition);//生成沿给定直线方向的线段偏移量坐标,给定直线方程（两侧型）
	int ParSegment(float length, int u1, int v1, int u2, int v2, int* SavePosition);
	void IntersectTwoLine(double A1, double B1, double C1, double A2, double B2, double C2, double& u, double& v);//求两直线交点，输入一般式
	float DisTwoPoints(float x1, float y1, float x2, float y2);//求两点间距离
	void RayLineFunction(float A, float B, int length, int* SavePosition);
	float OriofTwoPoints(float xs, float ys, float xe, float ye);//求点s指向点e的射线的朝向角
	void Pedal(double A, double B, double C, double x, double y, double& u, double& v);//计算垂足位置，输入直线方程和一个点坐标，计算点到直线的垂线和直线的交点
	void Pedal(double x1, double y1, double x2, double y2, double x, double y, double& u, double& v);//重载，输入直线两点式
	//输出直线方程，可重载，该函数输入角度和直线经过的一点,角度单位为度而非弧度,角度为习惯逆时针角度，水平向右为0度，坐标是图像坐标系坐标
	void LineFunc(double&A, double&B, double&C, double angle, double u, double v);
	void FitLine(float* L_Data, int DataLength, double* SaveP);//直线拟合
	void LineSegNearPoint(double A, double B, double C, float X, float Y, float length, float*SavePosition);//根据直线方程给出一点附近的线段
	template <typename T>
	void ArryMinus(int length, T* input);
	void CliffAlong(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, int* output);//地址数组上的最大跳变沿
	bool PreciseCliff(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, float* output, char sign);//地址数组上的最大跳变沿的亚像素位置
	void SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result);//判断亚像素级别的跨界像素点位置，输入为三个点的坐标和各自的灰度

	int DifferAlongLine(int base_p, unsigned char* IData, PixelLineSegment* pls, int span, int* SavePosition);//沿着直线计算差分，参数为点的位置，图像，像素线段，差分跨度，返回线段中点差分值得存储位置
	void PreciseLine(double A, double B, double x, double y, unsigned char* IData, float length, float search_width, double* SavePosition);//根据粗略的直线方程在指定点附近确定精确的点，拟合直线

	float DisPointToLine(float A, float B, float C, float x, float y);//点到直线距离


};


template <typename T>
void LineGenerator::ArryMinus(int length, T* input)
{
	for (int i = 0; i < length; i++)
		input[i] = -input[i];
}