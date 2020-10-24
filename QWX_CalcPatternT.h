#ifndef QWX_CALCPATTERNT_H_
#define QWX_CALCPATTERNT_H_

#include "QWX_MultiCoTarRecog.h"

class QWX_CalcPatternT
{
public:
	struct Pattern
	{
		QWX_MultiCoTarRecog::Pattern RgPattern_;
		cv::Mat T_m2g;
		bool flag_valid;

		Pattern() :flag_valid(false) {}

		Pattern(const QWX_MultiCoTarRecog::Pattern &pattern_noT)
		{
			RgPattern_ = pattern_noT;
			T_m2g = cv::Mat::eye(4, 4, CV_32FC1);
			flag_valid = true;
		}

		Pattern operator=(const Pattern &another)
		{
			flag_valid = another.flag_valid;
			RgPattern_ = another.RgPattern_;
			another.T_m2g.copyTo(T_m2g);
		}
	};

	struct camParam
	{
		cv::Mat camK;
		cv::Mat dist;
		cv::Mat T_c2g;
	};

private:
	QWX_MultiCoTarRecog mctr;
	std::vector<std::vector<Pattern>> patternsInImages;
	std::vector<cv::Point3f> objectPoints;

public:
	QWX_CalcPatternT();
	~QWX_CalcPatternT();

	bool setTarRecoger(const QWX_MultiCoTarRecog &_mctr);
	bool setObjectPoints(const std::vector<cv::Point3f> &_objectPoints);

	bool compute(std::vector<Pattern> &_patterns, const std::vector<cv::Mat> &_inputImages, const std::vector<camParam> &_camParams);
	std::vector<Pattern> getPatterns() const;

	bool clear();

private:
	bool calcT(Pattern &_p, const camParam &_camParam);
	//ÈÚºÏÎ»×Ë
	bool TFusion();
};

#endif // !QWX_COORDCOMPUTE_H_

