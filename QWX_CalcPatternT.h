#ifndef QWX_CALCPATTERNT_H_
#define QWX_CALCPATTERNT_H_

#include "QWX_MultiCoTarRecog.h"

class QWX_CalcPatternT
{
public:
	struct Pattern
	{
		QWX_MultiCoTarRecog::Pattern RgPattern_;
		cv::Mat T_;
		bool flag_valid;
		int time;

		Pattern() :flag_valid(false) {}

		Pattern(const QWX_MultiCoTarRecog::Pattern &pattern_noT)
		{
			RgPattern_ = pattern_noT;
			T_ = cv::Mat::eye(4, 4, CV_32FC1);
			flag_valid = true;
			time = 0;
		}

		const Pattern& operator=(const Pattern &another)
		{
			flag_valid = another.flag_valid;
			RgPattern_ = another.RgPattern_;
			another.T_.copyTo(T_);
			time = another.time;

			return *this;
		}
	};

	struct camParam
	{
		cv::Mat camK;
		cv::Mat dist;
		cv::Mat T_c2w;

		const camParam& operator=(const camParam &another)
		{
			another.camK.copyTo(camK);
			another.dist.copyTo(dist);
			another.T_c2w.copyTo(T_c2w);

			return *this;
		}
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

	bool compute(std::vector<Pattern> &_patterns, const std::vector<cv::Mat> &_inputImages, const std::vector<camParam> &_camParams, const std::vector<int> &times = std::vector<int>());
	std::vector<Pattern> getPatterns() const;
	std::vector<std::vector<Pattern>> getPatternsInImages() const;

	bool clear();

private:
	bool calcT(Pattern &_p, const camParam &_camParam);
	//ÈÚºÏÎ»×Ë
	bool TFusion();
};

#endif // !QWX_COORDCOMPUTE_H_

