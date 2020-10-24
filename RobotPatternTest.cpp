// RobotPatternTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include "QWX_MultiCoTarRecog.h"
#include "QWX_CalcPatternT.h"
#include "QWX_calcTraceShake.h"

#include <opencv2/opencv.hpp>

int main()
{
	//int a = 0x1FFFFFFF; //0xFFFE213B;

	std::vector<cv::Mat> Images;
	std::vector<QWX_CalcPatternT::camParam> camParams;
	{
		cv::Mat patternImage = cv::imread("Image_20201023182855376.bmp", 0);
		Images.push_back(patternImage);
		QWX_CalcPatternT::camParam camParam;
		camParam.camK = cv::Mat::eye(3, 3, CV_32FC1);
		camParam.camK.at<float>(2, 2) = 0.0;
		camParam.dist = cv::Mat::zeros(0, 5, CV_32FC1);
		camParam.T_c2g = cv::Mat::eye(4, 4, CV_32FC1);
		camParams.push_back(camParam);
	}

	std::vector<cv::Mat> origin_T_g2ms;
	{
		origin_T_g2ms.push_back(cv::Mat::eye(4, 4, CV_32FC1));
	}

	std::vector<cv::Point3f> points;
	{
		points.push_back(cv::Point3f(0, 0, 0));
		points.push_back(cv::Point3f(10, 0, 0));
		points.push_back(cv::Point3f(10, 10, 0));
		points.push_back(cv::Point3f(0, 10, 0));
	}

	std::vector<QWX_CalcPatternT::Pattern> cptPatterns;
	std::vector<cv::Mat> trace;
	std::vector<float> shake;


	std::map<int, int> mapCode2Type;
	mapCode2Type.insert(std::pair<int, int>(-64, 1));
	mapCode2Type.insert(std::pair<int, int>(1, 2));
	mapCode2Type.insert(std::pair<int, int>(-47, 3));
	mapCode2Type.insert(std::pair<int, int>(-67, 4));
	mapCode2Type.insert(std::pair<int, int>(-63, 5));
	mapCode2Type.insert(std::pair<int, int>(-48, 6));

	QWX_MultiCoTarRecog mctr;
	mctr.setMapCode2Type(mapCode2Type);
	/*mctr.Initial(&patternImage);

	int rst = mctr.GetInfor(patternImage);*/

	{
		QWX_CalcPatternT cpt;
		cpt.setTarRecoger(mctr);
		cpt.setObjectPoints(points);
		cpt.compute(cptPatterns, Images, camParams);
		cptPatterns = cpt.getPatterns();
	}

	{
		QWX_calcTraceShake cts;
		cts.setOringin_T_g2ms(origin_T_g2ms);
		cts.setMarkPt(cv::Point2f(5.0, 5.0));
		cts.compute(cptPatterns);
		trace = cts.getTrace();
		shake = cts.getShake();
	}


	return 0;
}


