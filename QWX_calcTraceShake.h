#ifndef QWX_CALCTRACESHAKE_H_
#define QWX_CALCTRACESHAKE_H_

#include "QWX_CalcPatternT.h"

class QWX_calcTraceShake
{
	std::vector<cv::Point3f> trace;
	std::vector<float> shake;
	std::vector<cv::Mat> origin_T_g2ms;
	std::vector<cv::Point3f> markersPt;

public:
	QWX_calcTraceShake();
	~QWX_calcTraceShake();

	bool setOringin_T_g2ms(std::vector<cv::Mat> _origin_T_g2ms);
	bool setMarkPt(std::vector<cv::Point3f> _markersPt);

	bool compute(const std::vector<cv::Mat> &_Ts_mn2g);

	std::vector<cv::Point3f> getTrace() const;
	std::vector<float> getShake() const;

private:

};

#endif // !QWX_CALCTRACESHAKE_H_
