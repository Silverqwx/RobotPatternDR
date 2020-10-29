#include "QWX_MultiCoTarRecog.h"

QWX_MultiCoTarRecog::QWX_MultiCoTarRecog()
	:MultiCoTarRecog()
{
}

QWX_MultiCoTarRecog::~QWX_MultiCoTarRecog()
{
}

bool QWX_MultiCoTarRecog::setMapCode2Type(const std::map<int, int>& _mapCode2Type)
{
	mapCode2Type_ = _mapCode2Type;

	return true;
}

int QWX_MultiCoTarRecog::GetInfor(cv::Mat Scr)
{
	patterns_.clear();

	int we, he;//扫描结束的位置
	int ls, lr, k3, imp;//简化计算
	int temp, temp1, temp2, temp3, temp4;
	bool findtar = false;
	we = width - border;
	he = height - border;
	SData = Scr.data;
	EdgeMap(&Scr, &ImageCanny, 70, 40, 2);
	TData = new uchar[width*height]();
	标志个数 = 0;
	Pattern pattern;

	for (int l = h_start; l < h_end; l++)
	{
		ls = l * width;

		for (int k = w_start; k < w_end; k++)
		{
			L = l;
			K = k;
			imp = ls + k;//图像点位置
			if (1907 == k && l == 1743)
				l = l;
			/**********************************************************************************/
			//根据条件成块跳过不需要扫描的区域
			switch (TData[imp])
			{
			case 1:
				k = k + 5;//不靠近canny点
				continue;
			case 2:
				k = k + 4;
				continue;//圆环没检测到
			case 3:
				k = k + 3;//灰度差别不大
				continue;
			case 4:
				k = k + 17;//发现一个目标
				continue;
			default:
				break;
			}
			/**********************************************************************************/

			/**********************************************************************************/
			//首先用一个灰度阈值进行判断
			Re = SData[imp];
			if (Re > 200)//首先用一个灰度阈值进行判断
				goto FALSEPOINT;
			/**********************************************************************************/

			/**********************************************************************************/
			//canny快速跳过
#pragma unroll
			for (int i = 0; i < 22; i++)
			{
				if (CData[imp + CannyCross[i]] == 255 || CData[imp - CannyCross[i]] == 255)
					goto NEARCANNY;
			}
#pragma unroll
			for (int i = width; i < width5; i = i + width)
			{
				TData[imp + i] = 1;
			}
			k = k + 5;
			goto FALSEPOINT;
		NEARCANNY:
			/**********************************************************************************/
			/**********************************************************************************/
			//灰度差值判断
			temp1 = 0;
			temp2 = 255;
#pragma unroll
			for (int j = 0; j < 8; j++)//此处用的8邻域点距离3Interval,9个像素的距离
			{
				temp = SData[imp + EightAdj[16 + j]];
				temp1 = cv::max<int>(temp1, temp);
				temp2 = cv::min<int>(temp2, temp);
			}
			if (temp1 - temp2 < 40)//在上面的八个点中，最大灰度区别较小，显然不是中心点附近的点，所以成块跳过
			{
#pragma unroll
				for (int i = width; i < width3; i = i + width)
				{
					TData[imp + i] = 3;
				}
				k = k + 3;
				goto FALSEPOINT;
			}
			/**********************************************************************************/

			/**********************************************************************************/
			//圆环检测
			temp = CircleCheck(imp, 0);//圆环检测
			if (temp == 1)
			{
				temp1 = imp - 2;
				for (int i = width; i < width3; i = i + width)//5行
				{
					TData[temp1 + i] = 2;
				}
				k = k + 4;
				goto FALSEPOINT;
			}
			if (temp == 2)
				goto FALSEPOINT;
			temp = CircleCheck(imp, 1);//圆环检测
			if (temp == 1)
			{
				temp1 = imp - 2;
				for (int i = width; i < width3; i = i + width)//5行
				{
					TData[temp1 + i] = 2;
				}
				k = k + 4;
				goto FALSEPOINT;
			}
			if (temp == 2)
				goto FALSEPOINT;
			/**********************************************************************************/

			/**********************************************************************************/
			////霍夫变换
			temp = InterSecHough(ImageToRecogGray, k, l);
			if (temp != 0)
			{
				if (temp == 1)//一条线段都没有
				{
					for (int i = width; i < NLines2; i = i + width)
					{
						TData[imp + i] = 4;
					}
					k = k + 15;
					goto FALSEPOINT;
				}
				goto FALSEPOINT;
			}
			/**********************************************************************************/
			//////////////显示霍夫变换检测到的中心点
			////k3 = (TarPosition[tar_num][1] * width + TarPosition[tar_num][0]) * 3;
			//////////k3 = (l*width + k) * 3;


			/////////**********************************************************************************/
			if (!GetLines(CenterPoint, CrossLine, 500))
				goto FALSEPOINT;


			//new code
			pattern.centerPt.u = k;
			pattern.centerPt.v = l;
			if (!consPattern(pattern))
				continue;
			if (!orderPoints(pattern))
				continue;
			if (!normalizePattern(pattern))
				continue;
			if (!recognizePattern(pattern))
				continue;

			patterns_.push_back(pattern);

			//old code
			//temp = FiveKeyPoints(k, l);
			//if (temp < 0)
			//	goto FALSEPOINT;
			///////////**********************************************************************************/
			//TarPosition[标志个数][0] = k;
			//TarPosition[标志个数][1] = l;
			//TarPosition[标志个数][2] = temp;
			//logo_inf[标志个数][0] = temp;
			//for (int i = 0; i < 8; i++)
			//{
			//	logo_inf[标志个数][i + 1] = Result[i];
			//}
			//logo_inf[标志个数][9] = k;
			//logo_inf[标志个数][10] = l;
			标志个数 = 标志个数 + 1;
#pragma unroll
			for (int i = width; i < width11; i = i + width)
			{
				for (int j = -8; j < 9; j = j + 1)
				{
					TData[imp + i + j] = 4;
				}
			}
			k = k + 9;
		FALSEPOINT:
			continue;

		}
	}

	delete[] TData;
	return 标志个数;
}

std::vector<QWX_MultiCoTarRecog::Pattern> QWX_MultiCoTarRecog::getPatterns() const
{
	return patterns_;
}

bool QWX_MultiCoTarRecog::consPattern(Pattern & _pattern)
{
	_pattern.featurePoints.resize(4);
	_pattern.featurePoints[0].u = End[0];
	_pattern.featurePoints[0].v = End[1];
	_pattern.featurePoints[1].u = End[2];
	_pattern.featurePoints[1].v = End[3];
	_pattern.featurePoints[2].u = End[4];
	_pattern.featurePoints[2].v = End[5];
	_pattern.featurePoints[3].u = End[6];
	_pattern.featurePoints[3].v = End[7];

	//int peak;//标记点编号
	int OneThree1, OneThree2, OneThree3, OneThree4;
	int TwoThree1, TwoThree2, TwoThree3, TwoThree4;

	Result[8] = _pattern.centerPt.u;
	Result[9] = _pattern.centerPt.v;

	////LG->IntersectTwoLine(Lines[12], Lines[13], Lines[14], Lines[15], Lines[16], Lines[17], Result + 8);//线1线3
	//线1和线2对面不形成顶点，34同样
	//下面确定标记点（4个点的顺序是1、2、4、3）
	//点1和点4是对顶点没有公共边，2、3点如是
	OneThree1 = width * floor(Result[9] + (End[1] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[0] - Result[8]) / 3 + 0.5);//向点1走三分之一
	OneThree2 = width * floor(Result[9] + (End[3] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[2] - Result[8]) / 3 + 0.5);//向点2走三分之一
	OneThree3 = width * floor(Result[9] + (End[5] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[4] - Result[8]) / 3 + 0.5);//向点3走三分之一
	OneThree4 = width * floor(Result[9] + (End[7] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[6] - Result[8]) / 3 + 0.5);//向点4走三分之一
	if (OneThree1<0 || OneThree1>im_size ||
		OneThree2<0 || OneThree2>im_size ||
		OneThree3<0 || OneThree3>im_size ||
		OneThree4<0 || OneThree4>im_size)
		return false;

	TwoThree1 = width * floor(Result[9] + (End[1] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[0] - Result[8]) / 1.5 + 0.5);//向点1走三分之二
	TwoThree2 = width * floor(Result[9] + (End[3] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[2] - Result[8]) / 1.5 + 0.5);//向点2走三分之二
	TwoThree3 = width * floor(Result[9] + (End[5] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[4] - Result[8]) / 1.5 + 0.5);//向点3走三分之二
	TwoThree4 = width * floor(Result[9] + (End[7] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[6] - Result[8]) / 1.5 + 0.5);//向点4走三分之二
	if (TwoThree1<0 || TwoThree1>im_size ||
		TwoThree2<0 || TwoThree2>im_size ||
		TwoThree3<0 || TwoThree3>im_size ||
		TwoThree4<0 || TwoThree4>im_size)
		return false;

	_pattern.featurePoints[0].markValue = SData[TwoThree1];
	_pattern.featurePoints[1].markValue = SData[TwoThree2];
	_pattern.featurePoints[2].markValue = SData[TwoThree3];
	_pattern.featurePoints[3].markValue = SData[TwoThree4];

	determCode(SData[OneThree1], SData[TwoThree1], _pattern.featurePoints[0].code);
	determCode(SData[OneThree2], SData[TwoThree2], _pattern.featurePoints[1].code);
	determCode(SData[OneThree3], SData[TwoThree3], _pattern.featurePoints[2].code);
	determCode(SData[OneThree4], SData[TwoThree4], _pattern.featurePoints[3].code);

	_pattern.featurePoints[0].angle = LG->OriofTwoPoints(_pattern.centerPt.u, _pattern.centerPt.v, _pattern.featurePoints[0].u, _pattern.featurePoints[0].v);
	_pattern.featurePoints[1].angle = LG->OriofTwoPoints(_pattern.centerPt.u, _pattern.centerPt.v, _pattern.featurePoints[1].u, _pattern.featurePoints[1].v);
	_pattern.featurePoints[2].angle = LG->OriofTwoPoints(_pattern.centerPt.u, _pattern.centerPt.v, _pattern.featurePoints[2].u, _pattern.featurePoints[2].v);
	_pattern.featurePoints[3].angle = LG->OriofTwoPoints(_pattern.centerPt.u, _pattern.centerPt.v, _pattern.featurePoints[3].u, _pattern.featurePoints[3].v);

	return true;
}

bool QWX_MultiCoTarRecog::orderPoints(Pattern & _pattern)
{
	std::sort(_pattern.featurePoints.begin(), _pattern.featurePoints.end());

	return true;
}

bool QWX_MultiCoTarRecog::normalizePattern(Pattern & _pattern)
{
	int normalCode = 99999;
	int fstIdx = -1;
	for (size_t fst = 0; fst < 4; fst++)
	{
		int tempCode = 0;
		for (size_t num = 0; num < 4; num++)
		{
			int idx = (fst + num) % 4;
			tempCode += _pattern.featurePoints[idx].code*pow(4, 3 - num);
		}

		if (normalCode < tempCode)
			continue;

		normalCode = tempCode;
		fstIdx = fst;
	}

	//把无标记作为标定板
	if (normalCode == 0)
	{
		int tempSum = 999999;
		for (size_t fst = 0; fst < _pattern.featurePoints.size(); fst++)
		{
			int sum = _pattern.featurePoints[fst].u + _pattern.featurePoints[fst].v;
			if (sum > tempSum)
				continue;
			tempSum = sum;
			fstIdx = fst;
		}
	}

	std::vector<featurePt> tempFeaturePts = _pattern.featurePoints;

	int tempFstIdx = fstIdx;
	for (size_t i = 0; i < _pattern.featurePoints.size(); i++)
	{
		_pattern.featurePoints[i] = tempFeaturePts[tempFstIdx];
		tempFstIdx++;
		tempFstIdx %= _pattern.featurePoints.size();
	}

	_pattern.PatternCode = normalCode;

	return true;
}

bool QWX_MultiCoTarRecog::recognizePattern(Pattern & _pattern)
{
	std::map<int, int>::iterator iter = mapCode2Type_.find(_pattern.PatternCode);
	if (iter == mapCode2Type_.end())
	{
		_pattern.patternType = -1;

		return false;
	}

	_pattern.patternType = iter->second;

	return true;
}

inline bool QWX_MultiCoTarRecog::determCode(unsigned char _p1, unsigned char _p2, int & _code)
{
	if (_p1 - _p2 > 35)
	{
		_code = -1;
	}
	else if (_p1 - _p2 < -35)
	{
		_code = 1;
	}
	else
		_code = 0;

	return true;
}
