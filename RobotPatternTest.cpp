// RobotPatternTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include "QWX_MultiCoTarRecog.h"

#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat patternImage = cv::imread("lueyuan2.jpg", 0);

	std::map<int, int> mapCode2Type;
	mapCode2Type.insert(std::pair<int, int>(-64, 1));
	mapCode2Type.insert(std::pair<int, int>(1, 2));
	mapCode2Type.insert(std::pair<int, int>(-47, 3));
	mapCode2Type.insert(std::pair<int, int>(-67, 4));
	mapCode2Type.insert(std::pair<int, int>(-63, 5));
	mapCode2Type.insert(std::pair<int, int>(-48, 6));

	QWX_MultiCoTarRecog mctr;
	mctr.Initial(&patternImage);
	mctr.setMapCode2Type(mapCode2Type);
	int rst = mctr.GetInfor(patternImage);

	return 0;
}


