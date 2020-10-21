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
	int TwoEnds(int u1, int v1, int u2, int v2, int* SavePosition);//�������˵�Ϊ���룬�����߶θ����ص����꣬��������,ÿ����Ԫ��λ������ͼ���������Ӧ��һά����
	int TwoEnds_Left2Right(int u1, int v1, int u2, int v2, int* SavePosition);//�������˵�Ϊ���룬�����߶θ����ص����꣬���ݴ洢˳�������,ÿ����Ԫ��λ
	int TwoEnds_Left2Right_1C(int u1, int v1, int u2, int v2, int* SavePosition);//�������˵�Ϊ���룬�����߶θ����ص����꣬���ݴ洢˳�������,ÿ����Ԫ��λ
	int TwoEnds_Sym(int u1, int v1, int u2, int v2, int* SavePosition);//�������˵�Ϊ���룬���߶��е�Ϊ��㣬���ҵ㽻��洢���γɶԳ��߶�ƫ��������
	int TwoEnds_Sym(float length, int u1, int v1, int u2, int v2, int* SavePosition);//���أ�ָ�������߶εĳ���
	int RelTwoEnds(int ustart, int vstart, int udes, int vdes, float length, int* SavePosition);//�������˵�Ϊ���룬���ɴӵ�1ָ���2���߶ε�����ƫ������ָ������length
	int RelTwoEndsCenter(float length, int u1, int v1, int u2, int v2, int* SavePosition);//�������˵�Ϊ�������������͵��߶Σ�ָ������length
	int RayLength(int u, int v, float length, float angle, int* SavePosition);//��ָ���������ָ������ͳ��ȵ����߸����ص����꣬��������
	int RayEnds(int us, int vs, int up, int vp, float length, int* SavePosition);//����������ɾ���ָ������߶Σ���������
	int VerSegmentF(float length, float A, float B, int* SavePosition);//���������ֱ�ߴ�ֱ���߶�����,����ֱ�߷���
	int VerSegmentF_Sym(float length, float A, float B, int* SavePosition);//���������ֱ�ߴ�ֱ���߶�����,����ֱ�߷���
	int VerSegment(float length, int u1, int v1, int u2, int v2, int* SavePosition);//���������ֱ�ߴ�ֱ���߶�����,����ֱ�߾���������
	int VerSegment_Sym(float length, int u1, int v1, int u2, int v2, int* SavePosition);//���������ֱ�ߴ�ֱ���߶�����,����ֱ�߾���������,�߶�ƫ�����ԳƷֲ�
	int ParSegmentF(float length, float A, float B, int* SavePosition);//�����ظ���ֱ�߷�����߶�ƫ��������,����ֱ�߷��̣������ͣ�
	int ParSegment(float length, int u1, int v1, int u2, int v2, int* SavePosition);
	void IntersectTwoLine(double A1, double B1, double C1, double A2, double B2, double C2, double& u, double& v);//����ֱ�߽��㣬����һ��ʽ
	float DisTwoPoints(float x1, float y1, float x2, float y2);//����������
	void RayLineFunction(float A, float B, int length, int* SavePosition);
	float OriofTwoPoints(float xs, float ys, float xe, float ye);//���sָ���e�����ߵĳ����
	void Pedal(double A, double B, double C, double x, double y, double& u, double& v);//���㴹��λ�ã�����ֱ�߷��̺�һ�������꣬����㵽ֱ�ߵĴ��ߺ�ֱ�ߵĽ���
	void Pedal(double x1, double y1, double x2, double y2, double x, double y, double& u, double& v);//���أ�����ֱ������ʽ
	//���ֱ�߷��̣������أ��ú�������ǶȺ�ֱ�߾�����һ��,�Ƕȵ�λΪ�ȶ��ǻ���,�Ƕ�Ϊϰ����ʱ��Ƕȣ�ˮƽ����Ϊ0�ȣ�������ͼ������ϵ����
	void LineFunc(double&A, double&B, double&C, double angle, double u, double v);
	void FitLine(float* L_Data, int DataLength, double* SaveP);//ֱ�����
	void LineSegNearPoint(double A, double B, double C, float X, float Y, float length, float*SavePosition);//����ֱ�߷��̸���һ�㸽�����߶�
	template <typename T>
	void ArryMinus(int length, T* input);
	void CliffAlong(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, int* output);//��ַ�����ϵ����������
	bool PreciseCliff(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, float* output, char sign);//��ַ�����ϵ���������ص�������λ��
	void SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result);//�ж������ؼ���Ŀ�����ص�λ�ã�����Ϊ�����������͸��ԵĻҶ�

	int DifferAlongLine(int base_p, unsigned char* IData, PixelLineSegment* pls, int span, int* SavePosition);//����ֱ�߼����֣�����Ϊ���λ�ã�ͼ�������߶Σ���ֿ�ȣ������߶��е���ֵ�ô洢λ��
	void PreciseLine(double A, double B, double x, double y, unsigned char* IData, float length, float search_width, double* SavePosition);//���ݴ��Ե�ֱ�߷�����ָ���㸽��ȷ����ȷ�ĵ㣬���ֱ��

	float DisPointToLine(float A, float B, float C, float x, float y);//�㵽ֱ�߾���


};


template <typename T>
void LineGenerator::ArryMinus(int length, T* input)
{
	for (int i = 0; i < length; i++)
		input[i] = -input[i];
}