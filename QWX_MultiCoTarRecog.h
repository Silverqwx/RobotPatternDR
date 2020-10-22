#ifndef QWX_MULTICOTARRECOG_H_
#define QWX_MULTICOTARRECOG_H_

#include "MultiCoTarRecog.h"

struct featurePt
{
	double u, v;
	unsigned char markValue;
	double angle;
	int code;

	bool operator<(const featurePt &another)
	{
		return angle > another.angle;
	}
};

struct CenterPt
{
	double u, v;
};

struct Pattern
{
	std::vector<featurePt> featurePoints;
	CenterPt centerPt;
	int PatternCode;
	int patternType;
};

class QWX_MultiCoTarRecog : public MultiCoTarRecog
{
	std::map<int, int> mapCode2Type_;//从码值到类型的映射关系
	std::vector<Pattern> patterns_;

public:
	QWX_MultiCoTarRecog();
	virtual ~QWX_MultiCoTarRecog();

	bool setMapCode2Type(const std::map<int, int> &_mapCode2Type);

	virtual int GetInfor(cv::Mat Scr);

private:
	bool consPattern(Pattern &_pattern);
	bool orderPoints(Pattern &_pattern);
	bool normalizePattern(Pattern &_pattern);
	bool recognizePattern(Pattern &_pattern);

	inline bool determCode(unsigned char _p1, unsigned char _p2, int &_code);

};
#endif // !QWX_MULTICOTARRECOG_H_
