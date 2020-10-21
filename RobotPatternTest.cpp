// RobotPatternTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include "QWX_MultiCoTarRecog.h"

#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat patternImage = cv::imread("6ada5bd5df3d93c3d0253c19f25fb43.jpg", 0);

	QWX_MultiCoTarRecog mctr;
	mctr.Initial(&patternImage);
	int rst = mctr.GetInfor(patternImage);

	return 0;
}


