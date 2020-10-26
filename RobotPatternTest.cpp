// RobotPatternTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include "QWX_MultiCoTarRecog.h"
#include "QWX_CalcPatternT.h"
#include "QWX_calcTraceShake.h"
#include "QWX_MotionEstimation.h"

#include <opencv2/opencv.hpp>

void readxml(const std::string &filename, cv::Mat &camK, cv::Mat &camDistCoeffs);

int main()
{
	//数据准备
	std::vector<std::vector<cv::Mat>> Images;
	std::vector<std::vector<int>> times;
	for (size_t i = 0; i < 5; i++)
	{
		std::vector<cv::Mat> subImages;
		std::vector<int> subTimes;
		for (size_t j = 0; j < 1; j++)
		{
			cv::Mat patternImage = cv::imread("Image_1.bmp", 0);
			subImages.push_back(patternImage);
			subTimes.push_back(j);
		}
		Images.push_back(subImages);
		times.push_back(subTimes);
	}
	std::vector<QWX_CalcPatternT::camParam> camParams_origin;
	cv::Mat T_w2g_origin = cv::Mat::eye(4, 4, CV_32FC1);
	cv::Mat T_b2c1 = cv::Mat::eye(4, 4, CV_32FC1);
	{
		cv::Mat camK, dist;
		readxml("procamcalib.xml", camK, dist);
		QWX_CalcPatternT::camParam camParam;
		camK.convertTo(camParam.camK, CV_32FC1);
		dist.convertTo(camParam.dist, CV_32FC1);
		camParam.T_c2w = cv::Mat::eye(4, 4, CV_32FC1);
		camParams_origin.push_back(camParam);
	}
	std::vector<cv::Mat> T_bn2w_origin;
	for (size_t i = 0; i < 4; i++)
	{
		T_bn2w_origin.push_back(cv::Mat::eye(4, 4, CV_32FC1));
	}

	std::vector<cv::Mat> origin_T_g2ms;
	{
		origin_T_g2ms.push_back(cv::Mat::eye(4, 4, CV_32FC1));
	}

	std::vector<cv::Point3f> points;
	{
		points.push_back(cv::Point3f(0, 0, 0));
		points.push_back(cv::Point3f(100, 0, 0));
		points.push_back(cv::Point3f(100, 100, 0));
		points.push_back(cv::Point3f(0, 100, 0));
	}

	std::vector<QWX_CalcPatternT::Pattern> cptPatterns;
	std::vector<cv::Mat> trace;
	std::vector<float> shake;

	float squareSize = 20.0;
	std::vector<cv::Point3f> objectCorners;
	for (size_t row = 0; row < 9; row++)
	{
		for (size_t col = 0; col < 11; col++)
		{
			objectCorners.push_back(cv::Point3f(col*squareSize, row*squareSize, 0.0));
		}
	}
	cv::Mat rVec, tVec;

	std::map<int, int> mapCode2Type;
	mapCode2Type.insert(std::pair<int, int>(-64, 1));
	mapCode2Type.insert(std::pair<int, int>(1, 2));
	mapCode2Type.insert(std::pair<int, int>(-47, 3));
	mapCode2Type.insert(std::pair<int, int>(-67, 4));
	mapCode2Type.insert(std::pair<int, int>(-63, 5));
	mapCode2Type.insert(std::pair<int, int>(-48, 6));
	mapCode2Type.insert(std::pair<int, int>(0, 7));


	std::vector<QWX_CalcPatternT::camParam> camParams = camParams_origin;
	/*{

		for (size_t imgIdx = 0; imgIdx < Images.size(); imgIdx++)
		{
			std::vector<cv::Point2f> corners;
			if (!cv::findChessboardCorners(Images[imgIdx], cv::Size(11, 9), corners))
				continue;

			camParams = camParams_origin;

			cv::solvePnP(objectCorners, corners, camParams[imgIdx].camK, camParams[imgIdx].dist, rVec, tVec);
			cv::Mat T_b2c2 = cv::Mat::eye(4, 4, CV_64FC1);

			cv::Rodrigues(rVec, T_b2c2(cv::Rect(0, 0, 3, 3)));
			tVec.copyTo(T_b2c2(cv::Rect(3, 0, 1, 3)));

			T_b2c2.convertTo(T_b2c2, CV_32FC1);

			cv::Mat T_c22c1 = T_b2c1 * T_b2c2.inv();
			T_b2c1 = T_b2c2;
			for (size_t i = 0; i < camParams.size(); i++)
			{
				camParams[i].T_c2w = camParams[i].T_c2w*T_c22c1;
			}
			break;
		}
	}*/

	std::vector<std::vector<cv::Point3f>> pointsInBoards;
	for (size_t i = 0; i < 1; i++)
	{
		pointsInBoards.push_back(points);
	}
	std::vector<std::vector<cv::Point3f>> pointsInMarkers;
	for (size_t i = 0; i < 1; i++)
	{
		pointsInMarkers.push_back(points);
	}

	QWX_MultiCoTarRecog mctr;
	mctr.setMapCode2Type(mapCode2Type);

	QWX_CalcPatternT cpt;
	cpt.setTarRecoger(mctr);
	//cpt.setObjectPoints(points);
	//cpt.compute(cptPatterns, Images, camParams);
	//cptPatterns = cpt.getPatterns();

	QWX_calcTraceShake cts;
	/*cts.setOringin_T_g2ms(origin_T_g2ms);
	cts.setMarkPt(cv::Point2f(5.0, 5.0));
	cts.compute(cptPatterns);
	trace = cts.getTrace();
	shake = cts.getShake();*/


	QWX_MotionEstimation me;
	me.setCalcPatternT(cpt);
	me.setCalcTraceShake(cts);
	me.setCamParamsTcn2wOrigin(camParams_origin);
	//me.setTbn2wOrigin(T_bn2w_origin);
	me.setTw2gOrigin(T_w2g_origin);
	me.setPointsInBoards(pointsInBoards);
	me.setPointsInMarkers(pointsInMarkers);
	me.init();
	me.initTg2mnOrigin(Images[0]);
	for (size_t i = 0; i < Images.size(); i++)
	{
		me.addImages(Images[i], times[i]);
	}






	return 0;
}

void readxml(const std::string &filename, cv::Mat &camK, cv::Mat &camDistCoeffs)
{
	cv::FileStorage fs;

	if (filename.empty())
		return;
	fs.open(filename, cv::FileStorage::READ);
	fs["camIntrinsics"] >> camK;
	fs["camDistCoeffs"] >> camDistCoeffs;
	fs.release();
}
