#include "QWX_calcTraceShake.h"

QWX_calcTraceShake::QWX_calcTraceShake()
{
}

QWX_calcTraceShake::~QWX_calcTraceShake()
{
}

bool QWX_calcTraceShake::setOringin_T_g2ms(std::vector<cv::Mat> _origin_T_g2ms)
{
	origin_T_g2ms.clear();
	for (size_t i = 0; i < _origin_T_g2ms.size(); i++)
		origin_T_g2ms.push_back(_origin_T_g2ms[i]);

	return true;
}

bool QWX_calcTraceShake::setMarkPt(cv::Point2f _markPt)
{
	markPt = _markPt;

	return true;
}

bool QWX_calcTraceShake::compute(const std::vector<QWX_CalcPatternT::Pattern>& _patterns)
{
	trace.clear();
	shake.clear();
	for (size_t i = 0; i < _patterns.size(); i++)
		trace.push_back(cv::Mat());
	shake.resize(_patterns.size());

	cv::Mat v = cv::Mat::zeros(4, 1, CV_32FC1);
	v.at<float>(2, 0) = 1;
	v.at<float>(3, 0) = 1;

	for (size_t patternIdx = 0; patternIdx < _patterns.size(); patternIdx++)
	{
		const QWX_CalcPatternT::Pattern &p = _patterns[patternIdx];

		if (!p.flag_valid)
			continue;

		cv::Mat markPt4d = cv::Mat::zeros(4, 1, CV_32FC1);
		markPt4d.at<float>(0, 0) = markPt.x;
		markPt4d.at<float>(1, 0) = markPt.y;
		markPt4d.at<float>(3, 0) = 1.0;

		trace[patternIdx] = p.T_*markPt4d;

		cv::Mat T_g2m = p.T_.inv();
		const cv::Mat &origin_T_g2m = origin_T_g2ms[patternIdx];

		cv::Mat v14d = T_g2m * v;
		cv::Mat v24d = origin_T_g2m * v;

		cv::Mat v1 = v14d(cv::Rect(0, 0, 1, 3));
		cv::Mat v2 = v24d(cv::Rect(0, 0, 1, 3));

		cv::Mat inner = v1.t()*v2;
		float n1 = cv::norm(v1, CV_L2);
		float n2 = cv::norm(v2, CV_L2);
		float innerV = inner.at<float>(0, 0);

		float costheta = innerV / (n1*n2);
		float theta = acosf(costheta);

		shake[patternIdx] = theta;
	}

	return true;
}

std::vector<cv::Mat> QWX_calcTraceShake::getTrace() const
{
	return trace;
}

std::vector<float> QWX_calcTraceShake::getShake() const
{
	return shake;
}
