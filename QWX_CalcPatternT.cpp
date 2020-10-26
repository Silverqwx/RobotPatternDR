#include "QWX_CalcPatternT.h"

QWX_CalcPatternT::QWX_CalcPatternT()
{
}

QWX_CalcPatternT::~QWX_CalcPatternT()
{
}

bool QWX_CalcPatternT::setTarRecoger(const QWX_MultiCoTarRecog & _mctr)
{
	mctr = _mctr;

	return true;
}

bool QWX_CalcPatternT::setObjectPoints(const std::vector<cv::Point3f>& _objectPoints)
{
	objectPoints = _objectPoints;

	return true;
}

bool QWX_CalcPatternT::compute(std::vector<Pattern>& _patterns, const std::vector<cv::Mat>& _inputImages, const std::vector<camParam> &_camParams, const std::vector<int> &times)
{
	if (_inputImages.size() != _camParams.size())
		return false;


	std::vector<cv::Mat> images;

	patternsInImages.clear();
	patternsInImages.resize(_inputImages.size());

	for (size_t i = 0; i < _inputImages.size(); i++)
		images.push_back(cv::Mat());

	for (size_t i = 0; i < _inputImages.size(); i++)
	{
		if (_inputImages[i].channels() == 3)
			cv::cvtColor(_inputImages[i], images[i], CV_RGB2GRAY);
		else
			_inputImages[i].copyTo(images[i]);
	}

	for (size_t imgIdx = 0; imgIdx < images.size(); imgIdx++)
	{
		const camParam &tempCamParam = _camParams[imgIdx];
		mctr.Initial(&images[imgIdx]);
		int rst = mctr.GetInfor(images[imgIdx]);
		if (rst == 0)
			continue;

		std::vector<QWX_MultiCoTarRecog::Pattern> patterns_noT = mctr.getPatterns();

		for (size_t patternIdx = 0; patternIdx < patterns_noT.size(); patternIdx++)
		{
			Pattern p(patterns_noT[patternIdx]);
			if (!calcT(p, tempCamParam))
				continue;
			if(times.size()!=0)
				p.time=times[imgIdx];
			patternsInImages[imgIdx].push_back(p);
		}

	}

	if (!TFusion())
		return false;

	return true;
}

std::vector<QWX_CalcPatternT::Pattern> QWX_CalcPatternT::getPatterns() const
{
	std::vector<Pattern> outPatterns;
	for (size_t i = 0; i < patternsInImages.size(); i++)
	{
		if (patternsInImages[i].size() == 0)
		{
			outPatterns.push_back(Pattern());
			continue;
		}

		outPatterns.push_back(patternsInImages[i][0]);

	}

	return outPatterns;
}

std::vector<std::vector<QWX_CalcPatternT::Pattern>> QWX_CalcPatternT::getPatternsInImages() const
{
	return patternsInImages;
}

bool QWX_CalcPatternT::clear()
{
	patternsInImages.clear();

	return true;
}

bool QWX_CalcPatternT::calcT(Pattern & _p, const camParam & _camParam)
{
	_p.T_ = cv::Mat::eye(4, 4, CV_32FC1);
	/*std::vector<cv::Point2f> point2ds;
	for (size_t ptIdx = 0; ptIdx < _p.RgPattern_.featurePoints.size(); ptIdx++)
	{
		point2ds.push_back(cv::Point2f(_p.RgPattern_.featurePoints[ptIdx].u, _p.RgPattern_.featurePoints[ptIdx].v));
	}

	cv::Mat rVec, tVec;
	cv::solvePnP(objectPoints, point2ds, _camParam.camK, _camParam.dist, rVec, tVec);

	cv::Mat T_m2c = cv::Mat::eye(4, 4, CV_64FC1);

	cv::Rodrigues(rVec, T_m2c(cv::Rect(0, 0, 3, 3)));
	tVec.copyTo(T_m2c(cv::Rect(3, 0, 1, 3)));

	T_m2c.convertTo(T_m2c, CV_32FC1);

	_p.T_ = _camParam.T_c2w*T_m2c;*/

	return true;
}

bool QWX_CalcPatternT::TFusion()
{
	return true;
}
