#ifndef QWX_CALCTRACESHAKE_H_
#define QWX_CALCTRACESHAKE_H_

#include "QWX_CalcPatternT.h"

class QWX_calcTraceShake
{
	std::vector<cv::Mat> trace;
	std::vector<float> shake;
	std::vector<cv::Mat> origin_T_g2ms;
	cv::Point2f markPt;

public:
	QWX_calcTraceShake();
	~QWX_calcTraceShake();

	bool setOringin_T_g2ms(std::vector<cv::Mat> _origin_T_g2ms);
	bool setMarkPt(cv::Point2f _markPt);

	bool compute(const std::vector<QWX_CalcPatternT::Pattern> &_patterns);

	std::vector<cv::Mat> getTrace() const;
	std::vector<float> getShake() const;

private:

};

#endif // !QWX_CALCTRACESHAKE_H_
