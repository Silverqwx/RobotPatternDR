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

			if (pattern.RgPattern_.patternType == 7)
			{
				if (tempT_bn2w_origin[camIdx].rows != 0)
					continue;
			}
			else if (tempT_w2mn_origin[pattern.RgPattern_.patternType - 1].rows != 0)
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

	/*测试*/
	/*cv::Mat p1 = T_w2mn_origin[0].inv()(cv::Rect(3, 0, 1, 3)),
		p2 = T_w2mn_origin[1].inv()(cv::Rect(3, 0, 1, 3));
	float distance = cv::norm(p1, p2, CV_L2);*/

	return true;
}

bool QWX_MotionEstimation::addImages(const std::vector<cv::Mat>& _inputImages, const std::vector<int>& times)
{
	std::vector<QWX_CalcPatternT::Pattern> cptPatterns;
	cpt.compute(cptPatterns, _inputImages, camParams_T_cn2w_origin, times);
	std::vector<std::vector<QWX_CalcPatternT::Pattern>> patternsInImages = cpt.getPatternsInImages();
	patternsInCamsInOrder_T_mb2wp.push_back(patternsInImages);

	return true;
}

bool QWX_MotionEstimation::process()
{
	calcT();
	dataFusion();
	calcTraceShake();

	return true;
}

std::vector<std::vector<cv::Point3f>> QWX_MotionEstimation::getTraceInCamsInOrder() const
{
	return traceInCamsInOrder;
}

std::vector<std::vector<float>> QWX_MotionEstimation::getShakeInCamsInOrder() const
{
	return shakeInCamsInOrder;
}

bool QWX_MotionEstimation::calcT()
{
	for (size_t orderIdx = 0; orderIdx < patternsInCamsInOrder_T_mb2wp.size(); orderIdx++)
	{
		std::vector<std::vector<QWX_CalcPatternT::Pattern>> &patternsInCams = patternsInCamsInOrder_T_mb2wp[orderIdx];
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
			}
		}
	}

	return true;
}

bool QWX_MotionEstimation::dataFusion()
{
	for (size_t orderIdx = 0; orderIdx < patternsInCamsInOrder_T_mb2wp.size(); orderIdx++)
	{
		std::vector<std::vector<QWX_CalcPatternT::Pattern>> &patternsInCams = patternsInCamsInOrder_T_mb2wp[orderIdx];
		std::vector<std::vector<QWX_CalcPatternT::Pattern>> patternsInMBs(pointsInMarkers.size());
		std::vector<cv::Mat> T_wp2w_sum;
		std::vector<std::vector<cv::Mat>> markersT_mn2wp_sum(pointsInMarkers.size());
		//for (size_t markerIdx = 0; markerIdx < pointsInMarkers.size(); markerIdx++)
		//	markersT_mn2wp_sum.push_back(std::vector<cv::Mat()>());
		cv::Mat T_wp2w;
		std::vector<cv::Mat> markersT_mn2wp;
		for (size_t markerIdx = 0; markerIdx < pointsInMarkers.size(); markerIdx++)
			markersT_mn2wp.push_back(cv::Mat());

		for (size_t camIdx = 0; camIdx < patternsInCams.size(); camIdx++)
		{
			std::vector<QWX_CalcPatternT::Pattern> &patterns = patternsInCams[camIdx];
			for (size_t patternIdx = 0; patternIdx < patterns.size(); patternIdx++)
			{
				QWX_CalcPatternT::Pattern &pattern = patterns[patternIdx];
				if (pattern.RgPattern_.patternType == 7)
				{
					cv::Mat tempT_wp2w = T_bn2w_origin[camIdx] * pattern.T_.inv();
					T_wp2w_sum.push_back(tempT_wp2w);
				}
				else
				{
					patternsInMBs[pattern.RgPattern_.patternType - 1].push_back(pattern);
					markersT_mn2wp_sum[pattern.RgPattern_.patternType - 1].push_back(pattern.T_.clone());
				}
			}
		}
		T_wp2w = TFusion(T_wp2w_sum);
		for (size_t markerIdx = 0; markerIdx < markersT_mn2wp_sum.size(); markerIdx++)
			markersT_mn2wp[markerIdx] = TFusion(markersT_mn2wp_sum[markerIdx]);

		TInOrder_wp2w.push_back(T_wp2w);

		for (size_t markerIdx = 0; markerIdx < markersT_mn2wp.size(); markerIdx++)
		{
			markersT_mn2wp[markerIdx] = T_w2g_origin * T_wp2w*markersT_mn2wp[markerIdx];//转化为从m到g的转移位姿
		}

		markersTInOrder_m2g.push_back(markersT_mn2wp);
	}

	return true;
}

bool QWX_MotionEstimation::calcTraceShake()//有点问题，暂时测试，等改
{
	cts.setOringin_T_g2ms(T_w2mn_origin);
	//cts.setMarkPt(cv::Point2f(0.0, 0.0));
	for (size_t orderIdx = 0; orderIdx < markersTInOrder_m2g.size(); orderIdx++)
	{
		std::vector<cv::Mat> &Ts_mn2g = markersTInOrder_m2g[orderIdx];
		cts.compute(Ts_mn2g);
		traceInCamsInOrder.push_back(cts.getTrace());
		shakeInCamsInOrder.push_back(cts.getShake());
	}


	return true;
}

inline cv::Mat QWX_MotionEstimation::TFusion(const std::vector<cv::Mat>& Ts)
{
	cv::Mat rstT = cv::Mat::eye(4, 4, CV_32FC1);
	cv::Mat rVec = cv::Mat::zeros(3, 1, CV_32FC1),
		tVec = cv::Mat::zeros(3, 1, CV_32FC1);
	std::vector<cv::Mat> rVecs(Ts.size()),
		tVecs(Ts.size());

	for (size_t TIdx = 0; TIdx < Ts.size(); TIdx++)
	{
		cv::Rodrigues(Ts[TIdx](cv::Rect(0, 0, 3, 3)), rVecs[TIdx]);
		Ts[TIdx](cv::Rect(3, 0, 1, 3)).copyTo(tVecs[TIdx]);
	}
	for (size_t TIdx = 0; TIdx < Ts.size(); TIdx++)
	{
		rVec += rVecs[TIdx];
		tVec += tVecs[TIdx];
	}
	rVec = rVec / Ts.size();
	tVec = tVec / Ts.size();

	cv::Rodrigues(rVec, rstT(cv::Rect(0, 0, 3, 3)));
	tVec.copyTo(rstT(cv::Rect(3, 0, 1, 3)));

	return rstT;
}

