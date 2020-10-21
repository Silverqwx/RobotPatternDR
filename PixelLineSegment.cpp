#include "PixelLineSegment.h"
#include "LineGenerator.h"


PixelLineSegment::PixelLineSegment(int w, int h, float length, float a, float b, unsigned char Mode)
:Width(w), Height(h), LineMode(Mode)
{
	LG = new LineGenerator(w,h);

	if (Mode == ParaFunc)
	{
		CenterCount = floor(length / 2);
		LengthInPixel = CenterCount * 2 + 1;
		LengthofArry = LengthInPixel * 3;
		data = new int[LengthInPixel * 3]();
		CenterPosition = CenterCount * 3;
		LG->ParSegmentF(length, a, b, data);
		LineRec_w = abs(data[0] - data[LengthofArry - 3]);
		LineRec_h = abs(data[1] - data[LengthofArry - 2]);
		LengthInMath = sqrt(float(LineRec_w*LineRec_w + LineRec_h*LineRec_h));
	}

	if (Mode == VerFunc)
	{
		CenterCount = floor(length / 2);
		LengthInPixel = CenterCount * 2 + 1;
		LengthofArry = LengthInPixel * 3;
		data = new int[LengthInPixel * 3]();
		CenterPosition = CenterCount * 3;
		LG->VerSegmentF(length, a, b, data);
		LineRec_w = abs(data[0] - data[LengthofArry - 3]);
		LineRec_h = abs(data[1] - data[LengthofArry - 2]);
		LengthInMath = sqrt(float(LineRec_w*LineRec_w + LineRec_h*LineRec_h));
	}
}
PixelLineSegment::PixelLineSegment(int w, int h, float length, int u1, int v1, int u2, int v2, unsigned char Mode)
:Width(w), Height(h), LineMode(Mode)
{
	LG = new LineGenerator(w, h);
	if (Mode == ParaFunc)
	{
		CenterCount = floor(length / 2);
		LengthInPixel = CenterCount * 2 + 1;
		LengthofArry = LengthInPixel * 3;
		data = new int[LengthInPixel * 3]();
		CenterPosition = CenterCount * 3;
		LG->ParSegment(length, u1, v1, u2, v2, data);
		LineRec_w = abs(data[0] - data[LengthofArry - 3]);
		LineRec_h = abs(data[1] - data[LengthofArry - 2]);
		LengthInMath = sqrt(float(LineRec_w*LineRec_w + LineRec_h*LineRec_h));
	}

	if (Mode == VerFunc)
	{
		CenterCount = floor(length / 2);
		LengthInPixel = CenterCount * 2 + 1;
		LengthofArry = LengthInPixel * 3;
		data = new int[LengthInPixel * 3]();
		CenterPosition = CenterCount * 3;
		LG->VerSegment(length, u1, v1, u2, v2, data);
		LineRec_w = abs(data[0] - data[LengthofArry - 3]);
		LineRec_h = abs(data[1] - data[LengthofArry - 2]);
		LengthInMath = sqrt(float(LineRec_w*LineRec_w + LineRec_h*LineRec_h));
	}
}

PixelLineSegment::~PixelLineSegment()
{
	if (data != NULL)
	{
		delete[] data;
		data = NULL;
	}
	delete LG;
	//////////_CrtDumpMemoryLeaks();
}
void PixelLineSegment::release()
{
	if (data != NULL)
	{
		delete[] data;
		data = NULL;
	}
}
CoEle PixelLineSegment::Left(int p)
{
	CoEle ce;
	int P = p * 3;
	ce.u = data[CenterPosition - P];
	ce.v = data[CenterPosition - P + 1];
	ce.imp = data[CenterPosition - P + 2];
	return ce;
}
CoEle PixelLineSegment::Right(int p)
{
	CoEle ce;
	int P = p * 3;
	ce.u = data[CenterPosition + P];
	ce.v = data[CenterPosition + P + 1];
	ce.imp = data[CenterPosition + P + 2];
	return ce;
}
CoEle PixelLineSegment::LeftHead()//返回左侧头元素
{
	CoEle ce;
	ce.u = data[0];
	ce.v = data[1];
	ce.imp = data[2];
	return ce;
}
CoEle PixelLineSegment::RightHead()//返回左侧头元素
{
	CoEle ce;
	ce.u = data[LengthofArry - 3];
	ce.v = data[LengthofArry - 2];
	ce.imp = data[LengthofArry - 1];
	return ce;
}