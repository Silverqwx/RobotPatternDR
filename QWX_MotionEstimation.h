#ifndef QWX_MOTIONESTIMATION_H_
#define QWX_MOTIONESTIMATION_H_

#include "QWX_calcTraceShake.h"

class QWX_MotionEstimation
{
	QWX_CalcPatternT cpt;
	QWX_calcTraceShake cts;
	cv::Mat T_w2g_origin;
	std::vector<QWX_CalcPatternT::camParam> camParams_T_cn2w_origin;
	std::vector<cv::Mat> T_bn2w_origin;
	std::vector<cv::Mat> T_w2mn_origin;

	int boardNum;

	std::vector<std::vector<cv::Point3f>> pointsInMarkers;
	std::vector<std::vector<cv::Point3f>> pointsInBoards;//�궨�������һһ��Ӧ����Ч�������Ϊ0

	std::vector<std::vector<std::vector<QWX_CalcPatternT::Pattern>>> patternsInCamsInOrder_T_mb2wp;//����������������еļ��ʶ��ģʽ���У���������mb��w����ת�ƾ���
	std::vector<std::vector<cv::Mat>> markersTInOrder_m2g;//��־��λ�ˣ�����������ϵ
	std::vector<cv::Mat> TInOrder_wp2w;//w'��w��ת�ƾ���   

	std::vector<std::vector<cv::Point3f>> traceInCamsInOrder;
	std::vector<std::vector<float>> shakeInCamsInOrder;

public:
	QWX_MotionEstimation();
	~QWX_MotionEstimation();

	bool setCalcPatternT(QWX_CalcPatternT _cpt);
	bool setCalcTraceShake(QWX_calcTraceShake _cts);
	bool setTw2gOrigin(cv::Mat _T_w2g_origin);
	bool setCamParamsTcn2wOrigin(std::vector<QWX_CalcPatternT::camParam> _camParams_T_cn2w_origin);
	//bool setTbn2wOrigin(std::vector<cv::Mat> _T_bn2w_origin);
	bool setBoardNum(int _num);
	bool setPointsInMarkers(std::vector<std::vector<cv::Point3f>> _pointsInMarkers);
	bool setPointsInBoards(std::vector<std::vector<cv::Point3f>> _pointsInBoards);

	bool init();
	bool initTg2mnOrigin(const std::vector<cv::Mat> &_inputImages);
	bool addImages(const std::vector<cv::Mat> &_inputImages, const std::vector<int> &times);
	bool process();

	std::vector<std::vector<cv::Point3f>> getTraceInCamsInOrder() const;
	std::vector < std::vector<float>> getShakeInCamsInOrder() const;


private:
	bool calcT();
	bool dataFusion();
	//bool updateT();
	bool calcTraceShake();

	inline cv::Mat TFusion(const std::vector<cv::Mat> &Ts);
};


#endif // !QWX_MOTIONESTIMATION_H_
