#include "QWX_MotionEstimation.h"

QWX_MotionEstimation::QWX_MotionEstimation()
	:boardNum(0)
{
}

QWX_MotionEstimation::~QWX_MotionEstimation()
{
}

bool QWX_MotionEstimation::setCalcPatternT(QWX_CalcPatternT _cpt)
{
	cpt = _cpt;

	return true;
}

bool QWX_MotionEstimation::setCalcTraceShake(QWX_calcTraceShake _cts)
{
	cts = _cts;

	return true;
}

bool QWX_MotionEstimation::setTw2gOrigin(cv::Mat _T_w2g_origin)
{
	T_w2g_origin = _T_w2g_origin;

	return true;
}

bool QWX_MotionEstimation::setCamParamsTcn2wOrigin(std::vector<QWX_CalcPatternT::camParam> _camParams_T_cn2w_origin)
{
	camParams_T_cn2w_origin = _camParams_T_cn2w_origin;

	return true;
}

//bool QWX_MotionEstimation::setTbn2wOrigin(std::vector<cv::Mat> _T_bn2w_origin)
//{
//	T_bn2w_origin = _T_bn2w_origin;
//
//	return true;
//}

bool QWX_MotionEstimation::setBoardNum(int _num)
{
	boardNum = _num;

	return true;
}

bool QWX_MotionEstimation::setPointsInMarkers(std::vector<std::vector<cv::Point3f>> _pointsInMarkers)
{
	pointsInMarkers = _pointsInMarkers;

	return true;
}

bool QWX_MotionEstimation::setPointsInBoards(std::vector<std::vector<cv::Point3f>> _pointsInBoards)
{
	pointsInBoards = _pointsInBoards;

	return true;
}

bool QWX_MotionEstimation::init()
{


	return true;
}

bool QWX_MotionEstimation::initTg2mnOrigin(const std::vector<cv::Mat>& _inputImages)
{
	std::vector<cv::Mat> tempT_w2mn_origin;
	std::vector<cv::Mat> tempT_bn2w_origin;
	for (size_t i = 0; i < pointsInMarkers.size(); i++)
	{
		tempT_w2mn_origin.push_back(cv::Mat(0, 0, CV_32FC1));
	}
	for (size_t i = 0; i < camParams_T_cn2w_origin.size(); i++)
	{
		tempT_bn2w_origin.push_back(cv::Mat(0, 0, CV_32FC1));
	}


	std::vector<QWX_CalcPatternT::Pattern> cptPatterns;
	cpt.compute(cptPatterns, _inputImages, camParams_T_cn2w_origin);
	std::vector<std::vector<QWX_CalcPatternT::Pattern>> patternsInImages = cpt.getPatternsInImages();

	//std::vector<QWX_CalcPatternT::Pattern> markers(pointsInMarkers.size());
	for (size_t camIdx = 0; camIdx < patternsInImages.size(); camIdx++)
	{
		std::vector<QWX_CalcPatternT::Pattern> &patterns = patternsInImages[camIdx];
		for (size_t patternIdx = 0; patternIdx < patterns.size(); patternIdx++)
		{
			QWX_CalcPatternT::Pattern &pattern = patterns[patternIdx];

			if (pattern.RgPattern_.patternType == 7 && tempT_bn2w_origin[camIdx].rows != 0)
				continue;
			if (tempT_w2mn_origin[pattern.RgPattern_.patternType - 1].rows != 0)
				continue;

			const cv::Mat &camK = camParams_T_cn2w_origin[camIdx].camK;
			const cv::Mat &dist = camParams_T_cn2w_origin[camIdx].dist;
			std::vector<cv::Point3f> point3fs;
			std::vector<cv::Point2f> point2fs;
			for (size_t ptIdx = 0; ptIdx < pattern.RgPattern_.featurePoints.size(); ptIdx++)
			{
				point2fs.push_back(cv::Point2f(pattern.RgPattern_.featurePoints[ptIdx].u, pattern.RgPattern_.featurePoints[ptIdx].v));
			}
			if (pattern.RgPattern_.patternType == 7)
			{
				point3fs = pointsInBoards[camIdx];
			}
			else
			{
				point3fs = pointsInMarkers[pattern.RgPattern_.patternType - 1];
			}

			cv::Mat rVec, tVec;
			cv::solvePnP(point3fs, point2fs, camK, dist, rVec, tVec);



			cv::Mat T_mb2cn = cv::Mat::eye(4, 4, CV_64FC1);

			cv::Rodrigues(rVec, T_mb2cn(cv::Rect(0, 0, 3, 3)));
			tVec.copyTo(T_mb2cn(cv::Rect(3, 0, 1, 3)));

			T_mb2cn.convertTo(T_mb2cn, CV_32FC1);

			const cv::Mat &T_cn2w = camParams_T_cn2w_origin[camIdx].T_c2w;

			pattern.T_ = T_cn2w * T_mb2cn;

			if (pattern.RgPattern_.patternType == 7)
				tempT_bn2w_origin[camIdx] = pattern.T_;
			else
				tempT_w2mn_origin[pattern.RgPattern_.patternType - 1] = pattern.T_.inv();

		}
	}



	for (size_t markerIdx = 0; markerIdx < tempT_w2mn_origin.size(); markerIdx++)
	{
		if (tempT_w2mn_origin[markerIdx].rows == 0)
			return false;
	}
	int validBoardNum = 0;
	for (size_t boardIdx = 0; boardIdx < tempT_bn2w_origin.size(); boardIdx++)
	{
		if (tempT_bn2w_origin[boardIdx].rows != 0)
			validBoardNum++;
	}
	if (validBoardNum != boardNum)
		return false;

	T_bn2w_origin = tempT_bn2w_origin;
	T_w2mn_origin = tempT_w2mn_origin;

	return true;
}

bool QWX_MotionEstimation::addImages(const std::vector<cv::Mat>& _inputImages, const std::vector<int>& times)
{
	std::vector<QWX_CalcPatternT::Pattern> cptPatterns;
	cpt.compute(cptPatterns, _inputImages, camParams_T_cn2w_origin, times);
	std::vector<std::vector<QWX_CalcPatternT::Pattern>> patternsInImages = cpt.getPatternsInImages();
	patternsInCamsInOrder_T_mb2w.push_back(patternsInImages);

	return true;
}

bool QWX_MotionEstimation::process()
{
	calcT();


	return true;
}

bool QWX_MotionEstimation::calcT()
{
	for (size_t orderIdx = 0; orderIdx < patternsInCamsInOrder_T_mb2w.size(); orderIdx++)
	{
		std::vector<std::vector<QWX_CalcPatternT::Pattern>> &patternsInCams = patternsInCamsInOrder_T_mb2w[orderIdx];
		for (size_t camIdx = 0; camIdx < patternsInCams.size(); camIdx++)
		{
			std::vector<QWX_CalcPatternT::Pattern> &patterns = patternsInCams[camIdx];
			for (size_t patternIdx = 0; patternIdx < patterns.size(); patternIdx++)
			{
				QWX_CalcPatternT::Pattern &pattern = patterns[patternIdx];

				const cv::Mat &camK = camParams_T_cn2w_origin[camIdx].camK;
				const cv::Mat &dist = camParams_T_cn2w_origin[camIdx].dist;
				std::vector<cv::Point3f> point3fs;
				std::vector<cv::Point2f> point2fs;
				for (size_t ptIdx = 0; ptIdx < pattern.RgPattern_.featurePoints.size(); ptIdx++)
				{
					point2fs.push_back(cv::Point2f(pattern.RgPattern_.featurePoints[ptIdx].u, pattern.RgPattern_.featurePoints[ptIdx].v));
				}
				if (pattern.RgPattern_.patternType == 7)
				{
					point3fs = pointsInBoards[camIdx];
				}
				else
				{
					point3fs = pointsInMarkers[pattern.RgPattern_.patternType];
				}

				cv::Mat rVec, tVec;
				cv::solvePnP(point3fs, point2fs, camK, dist, rVec, tVec);



				cv::Mat T_mb2cn = cv::Mat::eye(4, 4, CV_64FC1);

				cv::Rodrigues(rVec, T_mb2cn(cv::Rect(0, 0, 3, 3)));
				tVec.copyTo(T_mb2cn(cv::Rect(3, 0, 1, 3)));

				T_mb2cn.convertTo(T_mb2cn, CV_32FC1);

				const cv::Mat &T_cn2w = camParams_T_cn2w_origin[camIdx].T_c2w;

				pattern.T_ = T_cn2w * T_mb2cn;
			}
		}
	}

	return true;
}

bool QWX_MotionEstimation::calcTraceShake()//有点问题，暂时测试，等改
{
	cts.setOringin_T_g2ms(T_w2mn_origin);
	cts.setMarkPt(cv::Point2f(5.0, 5.0));
	for (size_t orderIdx = 0; orderIdx < patternsInCamsInOrder_T_mb2w.size(); orderIdx++)
	{
		std::vector<std::vector<QWX_CalcPatternT::Pattern>> &patternsInCams = patternsInCamsInOrder_T_mb2w[orderIdx];
		cts.compute(patternsInCams[0]);
	}
	traceInCamsInOrder.push_back(cts.getTrace());
	shakeInCamsInOrder.push_back(cts.getShake());

	return true;
}
