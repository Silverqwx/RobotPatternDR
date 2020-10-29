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
	int CannyCross[81];//用canny
	int rLength, Mrlength;
	double CrossLine[6];
	double Lines[18];
	double EdgeLine[12];
	cv::Point2d CenterPoint;
	double End[36], Result[36];
	int 标志个数;
	double logo_inf[18][11];//记录标志识别信息
	double LineFunc[36], IntSec[20];
	int LineNumber, Re, L, K;
public:
	void Initial(cv::Mat *S);//初始化
	void InputData(cv::Mat *S, cv::Vec3i Last_Tar);//输入待处理数据
	cv::Mat ImageToRecogGray, ImageCanny;
public://快速跟踪合作目标相关
	int w_start, w_end, h_start, h_end;
	int crosslength;
public:
	void GenModel();
	virtual int GetInfor(cv::Mat Scr);
	int RCLength1, RCLength2;
private://目标识别相关函数和变量
	cv::Vec3i TarPosition[18];//最终识别到的点
	int Forw, Forw1, Forw2, Forwh, Forwh1, Forwh2;//用于快速跳过的变量

	int CircleCheck(int DataP, int cn);
public:
	int InterSecHough(cv::Mat Scr, int u, int v);
private://目标测量相关函数
	int NLines, NLines1, NLines2;

	bool GetLines(cv::Point2d Center, double* CrossLine, int line_length = 200);//画出矩形的4条边，并按照顺时针顺序，从标记点开始输出四个顶点的图像坐标
	virtual int FiveKeyPoints(int u_center, int v_center);//根据提取到的直线和右手定则确定矩形四个顶点和标志中心点的图像坐标和顺序，其中标志中心点为第五个点
	int ALongCross(cv::Point2i Center, int* PLine, int*VLine, float*SavePoints, int maxsteps);//第一个存储位置存搜索头处找到边缘点，第二个存储位置存找到的交叉线边缘点
	bool PreciseLine(int ustart, int vstart, int* ForLine, int* LaLine, double* line, cv::Point2i Center);//起始点、前进线、侧向搜索线、存储地址、标记中点坐标
	void SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result);//判断亚像素级别的跨界像素点位置，输入为三个点的坐标和各自的灰度
private:
	int ChainAlternate(int* Chain, int ChainLength, int Threshold, int DIntv, int* SavePosition);//根据设定的导数间隔和阈值判断环形数据链的交替变化情况
	bool TenPoints();
	int RN, RNH, RNT;//探测辐条数量、半数、十分之一
	bool logoLG;
private://直线操作
	//LineType* LT;
	LineGenerator* LG;
	void DrawLine(cv::Mat* Scr, int u1, int v1, int u2, int v2, int L_width, char R, char G, char B);//画出直线用于观察
private://图像处理

	void EdgeMap(cv::Mat* Input, cv::Mat*Output, int threshold, int threshold_adj, int scale_length);//第一个阈值是跨度为scale_length的灰度差，第二个阈值是邻近像素的灰度差
private://工具感受
	int determ_code(int p11, int p12, int p21, int p22, int p31, int p32, int p41, int p42, int&peak);//根据每个顶点的位值确定标志的码值
	int determ_peak(int type1, int type2, int type3, int type4, int search_value);//确定标记点，输出与search_value一致的编号
	void start_from_peak(double* end_points, double* result_points, int peak);//根据标记点标志对顶点进行从新排序

public:
	unsigned char* SData;
	unsigned char* RData;
	unsigned char* CData;
	unsigned char* TData;
	int width, width3, width5, width11, width17, height, widthstep, border, StrLengthV, StrLengthP, im_size;
};
