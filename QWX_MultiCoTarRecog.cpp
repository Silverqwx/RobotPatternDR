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
	int we, he;//ɨ�������λ��
	int ls, lr, k3, imp;//�򻯼���
	int temp, temp1, temp2, temp3, temp4;
	bool findtar = false;
	we = width - border;
	he = height - border;
	SData = Scr.data;
	EdgeMap(&Scr, &ImageCanny, 70, 40, 2);
	TData = new uchar[width*height]();
	��־���� = 0;
	for (int l = h_start; l < h_end; l++)
	{
		ls = l * width;

		for (int k = w_start; k < w_end; k++)
		{
			L = l;
			K = k;
			imp = ls + k;//ͼ���λ��
			if (1907 == k && l == 1743)
				l = l;
			/**********************************************************************************/
			//���������ɿ���������Ҫɨ�������
			switch (TData[imp])
			{
			case 1:
				k = k + 5;//������canny��
				continue;
			case 2:
				k = k + 4;
				continue;//Բ��û��⵽
			case 3:
				k = k + 3;//�ҶȲ�𲻴�
				continue;
			case 4:
				k = k + 17;//����һ��Ŀ��
				continue;
			default:
				break;
			}
			/**********************************************************************************/

			/**********************************************************************************/
			//������һ���Ҷ���ֵ�����ж�
			Re = SData[imp];
			if (Re > 200)//������һ���Ҷ���ֵ�����ж�
				goto FALSEPOINT;
			/**********************************************************************************/

			/**********************************************************************************/
			//canny��������
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
			//�ҶȲ�ֵ�ж�
			temp1 = 0;
			temp2 = 255;
#pragma unroll
			for (int j = 0; j < 8; j++)//�˴��õ�8��������3Interval,9�����صľ���
			{
				temp = SData[imp + EightAdj[16 + j]];
				temp1 = cv::max<int>(temp1, temp);
				temp2 = cv::min<int>(temp2, temp);
			}
			if (temp1 - temp2 < 40)//������İ˸����У����Ҷ������С����Ȼ�������ĵ㸽���ĵ㣬���Գɿ�����
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
			//Բ�����
			temp = CircleCheck(imp, 0);//Բ�����
			if (temp == 1)
			{
				temp1 = imp - 2;
				for (int i = width; i < width3; i = i + width)//5��
				{
					TData[temp1 + i] = 2;
				}
				k = k + 4;
				goto FALSEPOINT;
			}
			if (temp == 2)
				goto FALSEPOINT;
			temp = CircleCheck(imp, 1);//Բ�����
			if (temp == 1)
			{
				temp1 = imp - 2;
				for (int i = width; i < width3; i = i + width)//5��
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
			////����任
			temp = InterSecHough(ImageToRecogGray, k, l);
			if (temp != 0)
			{
				if (temp == 1)//һ���߶ζ�û��
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
			//////////////��ʾ����任��⵽�����ĵ�
			////k3 = (TarPosition[tar_num][1] * width + TarPosition[tar_num][0]) * 3;
			//////////k3 = (l*width + k) * 3;


			/////////**********************************************************************************/
			if (!GetLines(CenterPoint, CrossLine))
				goto FALSEPOINT;


			//new code
			Pattern pattern;
			pattern.centerPt.u = k;
			pattern.centerPt.v = l;
			consPattern(pattern);
			orderPoints(pattern);
			normalizePattern(pattern);
			recognizePattern(pattern);

			//old code
			temp = FiveKeyPoints(k, l);
			if (temp < 0)
				goto FALSEPOINT;
			/////////**********************************************************************************/
			TarPosition[��־����][0] = k;
			TarPosition[��־����][1] = l;
			TarPosition[��־����][2] = temp;
			logo_inf[��־����][0] = temp;
			for (int i = 0; i < 8; i++)
			{
				logo_inf[��־����][i + 1] = Result[i];
			}
			logo_inf[��־����][9] = k;
			logo_inf[��־����][10] = l;
			��־���� = ��־���� + 1;
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
	return ��־����;
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



	int peak;//��ǵ���
	int OneThree1, OneThree2, OneThree3, OneThree4;
	int TwoThree1, TwoThree2, TwoThree3, TwoThree4;
	float angle1, angle2;//�����жϵ�˳��

	////LG->IntersectTwoLine(Lines[12], Lines[13], Lines[14], Lines[15], Lines[16], Lines[17], Result + 8);//��1��3
	//��1����2���治�γɶ��㣬34ͬ��
	//����ȷ����ǵ㣨4�����˳����1��2��4��3��
	//��1�͵�4�ǶԶ���û�й����ߣ�2��3������
	OneThree1 = width * floor(Result[9] + (End[1] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[0] - Result[8]) / 3 + 0.5);//���1������֮һ
	OneThree2 = width * floor(Result[9] + (End[3] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[2] - Result[8]) / 3 + 0.5);//���2������֮һ
	OneThree3 = width * floor(Result[9] + (End[5] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[4] - Result[8]) / 3 + 0.5);//���3������֮һ
	OneThree4 = width * floor(Result[9] + (End[7] - Result[9]) / 3 + 0.5) + floor(Result[8] + (End[6] - Result[8]) / 3 + 0.5);//���4������֮һ
	if (OneThree1<0 || OneThree1>im_size ||
		OneThree2<0 || OneThree2>im_size ||
		OneThree3<0 || OneThree3>im_size ||
		OneThree4<0 || OneThree4>im_size)
		return 0;

	TwoThree1 = width * floor(Result[9] + (End[1] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[0] - Result[8]) / 1.5 + 0.5);//���1������֮��
	TwoThree2 = width * floor(Result[9] + (End[3] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[2] - Result[8]) / 1.5 + 0.5);//���2������֮��
	TwoThree3 = width * floor(Result[9] + (End[5] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[4] - Result[8]) / 1.5 + 0.5);//���3������֮��
	TwoThree4 = width * floor(Result[9] + (End[7] - Result[9]) / 1.5 + 0.5) + floor(Result[8] + (End[6] - Result[8]) / 1.5 + 0.5);//���4������֮��
	if (TwoThree1<0 || TwoThree1>im_size ||
		TwoThree2<0 || TwoThree2>im_size ||
		TwoThree3<0 || TwoThree3>im_size ||
		TwoThree4<0 || TwoThree4>im_size)
		return 0;

	return true;
}

bool QWX_MultiCoTarRecog::orderPoints(Pattern & _pattern)
{
	return true;
}

bool QWX_MultiCoTarRecog::normalizePattern(Pattern & _pattern)
{
	return true;
}

bool QWX_MultiCoTarRecog::recognizePattern(Pattern & _pattern)
{
	return true;
}
