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

bool QWX_calcTraceShake::setMarkPt(std::vector<cv::Point3f> _markersPt)
{
	markersPt = _markersPt;

	return true;
}


bool QWX_calcTraceShake::compute(const std::vector<cv::Mat> &_Ts_mn2g)
{
	trace.clear();
	shake.clear();
	//for (size_t i = 0; i < _Ts_mn2g.size(); i++)
	//	trace.push_back(cv::Mat());
	trace.resize(_Ts_mn2g.size());
	shake.resize(_Ts_mn2g.size());

	cv::Mat v = cv::Mat::zeros(4, 1, CV_32FC1);
	v.at<float>(2, 0) = 1;
	v.at<float>(3, 0) = 1;

	for (size_t markerIdx = 0; markerIdx < _Ts_mn2g.size(); markerIdx++)
	{
		const cv::Mat &T = _Ts_mn2g[markerIdx];

		//if (!p.flag_valid)
		//	continue;

		cv::Mat markPt4d = cv::Mat::zeros(4, 1, CV_32FC1);
		markPt4d.at<float>(0, 0) = markersPt[markerIdx].x;
		markPt4d.at<float>(1, 0) = markersPt[markerIdx].y;
		markPt4d.at<float>(2, 0) = markersPt[markerIdx].z;
		markPt4d.at<float>(3, 0) = 1.0;

		cv::Mat traceMat = T * markPt4d;
		trace[markerIdx].x = traceMat.at<float>(0, 0);
		trace[markerIdx].y = traceMat.at<float>(1, 0);
		trace[markerIdx].z = traceMat.at<float>(2, 0);

		cv::Mat T_g2m = T.inv();
		const cv::Mat &origin_T_g2m = origin_T_g2ms[markerIdx];

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

		shake[markerIdx] = theta;
	}

	return true;
}

std::vector<cv::Point3f> QWX_calcTraceShake::getTrace() const
{
	return trace;
}

std::vector<float> QWX_calcTraceShake::getShake() const
{
	return shake;
}
