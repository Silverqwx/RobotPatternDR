
#pragma once


//////////#define _CRTDBG_MAP_ALLOC
//////////#include <stdlib.h>
//////////#include <crtdbg.h>


#include <math.h>
#define ParaFunc 1
#define VerFunc 2

extern class LineGenerator;
struct CoEle//坐标元素
{
	int u;
	int v;
	int imp;
};
class PixelLineSegment
{
public:
	PixelLineSegment(int w, int h, float length, float a, float b, unsigned char Mode);
	PixelLineSegment(int w, int h, float length, int u1, int v1, int u2, int v2, unsigned char Mode);
public:
	~PixelLineSegment();
public:
	int Width, Height, LengthInPixel, LengthofArry;
	int LineRec_h, LineRec_w;
	int CenterCount, CenterPosition;
	char LineMode;
	float LengthInMath;
	int* data;
	CoEle Left(int p);//返回左侧第p个元素
	CoEle Right(int p);//返回右侧第p个元素
	CoEle LeftHead();//返回左侧头元素
	CoEle RightHead();//返回右侧头元素
	void release();
	LineGenerator* LG;
};