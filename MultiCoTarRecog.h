#pragma once
////#include <cxcore.h>
////#include <highgui.h>
////#include <stdlib.h>
////#include <stdio.h>
////#include <math.h>
////#include <cv.h>
////#include <opencv2/ml/ml.hpp>
#include "opencv_heads.h"
#include "AIV_typedefine_lxl20200624.h"

#include "LineGenerator.h"

class QWX_MultiCoTarRecog;

class MultiCoTarRecog
{
	friend class QWX_MultiCoTarRecog;
public:
	MultiCoTarRecog(void);
public:
	virtual ~MultiCoTarRecog(void);
public:
	int EightAdj[64];
	int Circle[1000];
	int CircleM[1000];
	int CannyCross[81];//��canny
	int rLength, Mrlength;
	double CrossLine[6];
	double Lines[18];
	double EdgeLine[12];
	cv::Point2d CenterPoint;
	double End[36], Result[36];
	int ��־����;
	double logo_inf[18][11];//��¼��־ʶ����Ϣ
	double LineFunc[36], IntSec[20];
	int LineNumber, Re, L, K;
public:
	void Initial(cv::Mat *S);//��ʼ��
	void InputData(cv::Mat *S, cv::Vec3i Last_Tar);//�������������
	cv::Mat ImageToRecogGray, ImageCanny;
public://���ٸ��ٺ���Ŀ�����
	int w_start, w_end, h_start, h_end;
	int crosslength;
public:
	void GenModel();
	virtual int GetInfor(cv::Mat Scr);
	int RCLength1, RCLength2;
private://Ŀ��ʶ����غ����ͱ���
	cv::Vec3i TarPosition[18];//����ʶ�𵽵ĵ�
	int Forw, Forw1, Forw2, Forwh, Forwh1, Forwh2;//���ڿ��������ı���

	int CircleCheck(int DataP, int cn);
public:
	int InterSecHough(cv::Mat Scr, int u, int v);
private://Ŀ�������غ���
	int NLines, NLines1, NLines2;

	bool GetLines(cv::Point2d Center, double* CrossLine, int line_length = 200);//�������ε�4���ߣ�������˳ʱ��˳�򣬴ӱ�ǵ㿪ʼ����ĸ������ͼ������
	virtual int FiveKeyPoints(int u_center, int v_center);//������ȡ����ֱ�ߺ����ֶ���ȷ�������ĸ�����ͱ�־���ĵ��ͼ�������˳�����б�־���ĵ�Ϊ�������
	int ALongCross(cv::Point2i Center, int* PLine, int*VLine, float*SavePoints, int maxsteps);//��һ���洢λ�ô�����ͷ���ҵ���Ե�㣬�ڶ����洢λ�ô��ҵ��Ľ����߱�Ե��
	bool PreciseLine(int ustart, int vstart, int* ForLine, int* LaLine, double* line, cv::Point2i Center);//��ʼ�㡢ǰ���ߡ����������ߡ��洢��ַ������е�����
	void SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result);//�ж������ؼ���Ŀ�����ص�λ�ã�����Ϊ�����������͸��ԵĻҶ�
private:
	int ChainAlternate(int* Chain, int ChainLength, int Threshold, int DIntv, int* SavePosition);//�����趨�ĵ����������ֵ�жϻ����������Ľ���仯���
	bool TenPoints();
	int RN, RNH, RNT;//̽�����������������ʮ��֮һ
	bool logoLG;
private://ֱ�߲���
	//LineType* LT;
	LineGenerator* LG;
	void DrawLine(cv::Mat* Scr, int u1, int v1, int u2, int v2, int L_width, char R, char G, char B);//����ֱ�����ڹ۲�
private://ͼ����

	void EdgeMap(cv::Mat* Input, cv::Mat*Output, int threshold, int threshold_adj, int scale_length);//��һ����ֵ�ǿ��Ϊscale_length�ĻҶȲ�ڶ�����ֵ���ڽ����صĻҶȲ�
private://���߸���
	int determ_code(int p11, int p12, int p21, int p22, int p31, int p32, int p41, int p42, int&peak);//����ÿ�������λֵȷ����־����ֵ
	int determ_peak(int type1, int type2, int type3, int type4, int search_value);//ȷ����ǵ㣬�����search_valueһ�µı��
	void start_from_peak(double* end_points, double* result_points, int peak);//���ݱ�ǵ��־�Զ�����д�������

public:
	unsigned char* SData;
	unsigned char* RData;
	unsigned char* CData;
	unsigned char* TData;
	int width, width3, width5, width11, width17, height, widthstep, border, StrLengthV, StrLengthP, im_size;
};
