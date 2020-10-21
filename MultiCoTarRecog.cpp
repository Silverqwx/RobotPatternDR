#include "MultiCoTarRecog.h"

MultiCoTarRecog::MultiCoTarRecog(void)
{
	logoLG = false;
}
MultiCoTarRecog::~MultiCoTarRecog(void)
{
	//////////_CrtDumpMemoryLeaks();
	if (logoLG)
		delete LG;
}
void MultiCoTarRecog::Initial(cv::Mat *S)//��ʼ��
{
	int size;
	width = S->cols;
	height = S->rows;
	widthstep = S->step;
	im_size = width * height;
	size = max(width, height);
	border = 25;
	Forw = 8;//8�ڵ�͵�ǰ�жϵĵ�ҶȲ��첻����ʱ�ɿ�������������Ŀ��
	Forw1 = 2;//Բ����ⲻͨ��ʱ�ɿ�������������Ŀ��
	Forw2 = 8;//8�ڵ�Ҷ����������ʱ�ɿ�������������Ŀ��
	Forwh = Forw / 2;
	Forwh1 = 2;
	Forwh2 = 10;
	RCLength1 = size / 96;
	RCLength2 = size / 43;
	//GoOnLength=max<int>(6,size/360*3);
	for (int i = 0; i < 36; i++)
		End[i] = 0;
	for (int i = 0; i < 20; i++)
		IntSec[i] = 0;
	NLines = width * Forwh;//N�е�������
	NLines1 = width * Forwh1;//N�е�������
	NLines2 = width * Forwh2;//N�е�������
	StrLengthV = 19;//��ֱģ�峤��
	StrLengthP = 18;//ƽ��ģ�峤��

	width3 = width * 3;
	width5 = width * 5;
	width11 = width * 11;
	width17 = width * 17;
	crosslength = height / 2;//������������󳤶Ȳ�����ͼ���ȵ�һ��
	LineNumber = 0;
	GenModel();
	if (logoLG)
		delete LG;
	LG = new LineGenerator(width, height);
	logoLG = true;
	w_start = border;
	w_end = width - border;
	h_start = border;
	h_end = height - border;
	ImageCanny = cv::Mat(height, width, CV_8U, cv::Scalar::all(0));
	CData = ImageCanny.data;

}
void MultiCoTarRecog::InputData(cv::Mat *S, cv::Vec3i Last_Tar)//�������������
{
	ImageToRecogGray = *S;

	float runtime;

	EdgeMap(S, &ImageCanny, 70, 40, 2);


	if (Last_Tar[0] < 0)
	{
		w_start = border;
		w_end = width - border;
		h_start = border;
		h_end = height - border;
	}
	else
	{
		w_start = Last_Tar[0] - width / 6;
		w_end = Last_Tar[0] + width / 6;
		h_start = Last_Tar[1] - height / 6;
		h_end = Last_Tar[1] + height / 6;
		if (w_start < border)
			w_start = border;
		if (w_end > width - border)
			w_end = width - border;
		if (h_start < border)
			h_start = border;
		if (h_end > height - border)
			h_end = height - border;
	}
	SData = S->data;
	CData = ImageCanny.data;
}
void MultiCoTarRecog::GenModel()
{
	//����8����ģ��
	int Interval = 3, ii = 0, iw;
	for (int i = 0; i < 64; i = i + 8)//8�������Interval��2Interval����8Interval
	{
		ii = ii + Interval;
		iw = ii * width;
		EightAdj[i] = ii;//�����ҿ�ʼ����ʱ��
		EightAdj[i + 1] = ii - iw;
		EightAdj[i + 2] = -iw;
		EightAdj[i + 3] = -ii - iw;
		EightAdj[i + 4] = -ii;
		EightAdj[i + 5] = iw - ii;
		EightAdj[i + 6] = iw;
		EightAdj[i + 7] = iw + ii;
	}
	int chead = 0;
	for (int l = -5; l <= 5; l = l + 1)
	{
		CannyCross[l + 5] = (5 * width) + l;//�º���
		CannyCross[l + 16] = l * width + 5;//�Һ���
		chead++;
	}
	//����Բ��ģ��
	int radius[3];
	int Model_Step, Point_Number, Head = 4, um, vm, uf, vf, image_posi, cc;
	float angle_interval = 6.283;
	radius[0] = 11;
	radius[1] = 13;
	radius[2] = 33;
	Circle[0] = 4;
	for (int i = 0; i < 3; i++)
	{
		Model_Step = radius[i] * 24;
		angle_interval = 6.283 / Model_Step;
		Point_Number = 0;
		cc = 0;
		uf = floor(radius[i] + 0.5);
		vf = 0;
		for (int l = 0; l < Model_Step; l++)
		{
			um = floor(radius[i] * cos(angle_interval*l) + 0.5);
			vm = floor(radius[i] * sin(angle_interval*l) + 0.5);
			if (um != uf && vm != vf)
			{
				image_posi = uf + vm * width;
				cc = cc + 1;
				Circle[Head] = image_posi;
				Head = Head + 1;
			}
			image_posi = um + vm * width;
			if (image_posi != Circle[Head - 1])
			{
				cc = cc + 1;
				Circle[Head] = image_posi;
				Head = Head + 1;
			}
			uf = um;
			vf = vm;
		}
		Circle[i + 1] = cc + Circle[i];
	}
	//��������������ȡ��ͬ��Բ
	int radiusM[7];
	Head = 8;
	radiusM[0] = 6;
	radiusM[1] = 7;
	radiusM[2] = 8;
	radiusM[3] = 9;
	radiusM[4] = 10;
	radiusM[5] = 11;
	radiusM[6] = 12;
	CircleM[0] = 8;//��һ��Բ�洢��λ�õ��׵�ַ��ǰ7λ��ķֱ��Ǹ�Բ���׵�ַ����7λ����ǵ�7��Բ���׵�ַ����Ȼ��������
	for (int i = 0; i < 7; i++)
	{
		Model_Step = radiusM[i] * 6;
		angle_interval = 6.283 / Model_Step;
		Point_Number = 0;
		cc = 0;
		for (int l = 0; l < Model_Step; l++)
		{
			um = floor(radiusM[i] * cos(angle_interval*l) + 0.5);
			vm = floor(radiusM[i] * sin(angle_interval*l) + 0.5);
			image_posi = um + vm * width;
			if (image_posi != CircleM[Head - 1])
			{
				cc = cc + 1;
				CircleM[Head] = image_posi;
				Head = Head + 1;
			}
		}
		CircleM[i + 1] = cc + CircleM[i];
	}
}
int MultiCoTarRecog::GetInfor(cv::Mat Scr)
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
bool MultiCoTarRecog::GetLines(cv::Point2d Center, double* CrossLine)//�������ε�4���ߣ�������˳ʱ��˳�򣬴ӱ�ǵ㿪ʼ����ĸ������ͼ������
{
	cv::Point2i centerpixel = Center;
	float length1, length2, length3, length4;
	float Points[200];//���Ľ�������ͷ�ҵ���40����
	////float CrossEdgePoints[2400];//�����߾����ĵ�
	int CrossLength;
	int PLine1[200], PLine2[200];
	int VLine[200];
	LG->RayLineFunction(CrossLine[0], CrossLine[1], 50, PLine1);//����ǰ������ģ��,CrossLine���ɻ���任�õ��������߷���
	LG->RayLineFunction(CrossLine[3], CrossLine[4], 50, PLine2);//���ɴ�ֱģ��
	////LG->ParSegmentF(50,CrossLine[3], CrossLine[4], VLine);//���ɴ�ֱģ��
	int temp_crosslength;
	memcpy(VLine, PLine2, 150 * sizeof(int));
	CrossLength = ALongCross(centerpixel, PLine1, VLine, Points, crosslength);
	if (CrossLength <= 0)
	{
		crosslength = height / 2;
		return false;
	}
	temp_crosslength = max(CrossLength, 0);
	LG->ArryMinus(150, PLine1);

	CrossLength = ALongCross(centerpixel, PLine1, VLine, Points + 20, crosslength);
	if (CrossLength <= 0)
	{
		crosslength = height / 2;
		return false;
	}
	temp_crosslength = max(CrossLength, temp_crosslength);

	memcpy(VLine, PLine1, 150 * sizeof(int));
	CrossLength = ALongCross(centerpixel, PLine2, VLine, Points + 40, crosslength);
	if (CrossLength <= 0)
	{
		crosslength = height / 2;
		return false;
	}
	temp_crosslength = max(CrossLength, temp_crosslength);

	LG->ArryMinus(150, PLine2);

	CrossLength = ALongCross(centerpixel, PLine2, VLine, Points + 60, crosslength);
	if (CrossLength <= 0)
	{
		crosslength = height / 2;
		return false;
	}
	temp_crosslength = max(CrossLength, temp_crosslength);
	crosslength = min(height / 2, (int)(temp_crosslength*1.5));
	LG->FitLine(Points, 10, Lines);//��1���������������ı�Ե��������ֱ��
	LG->FitLine(Points + 20, 10, Lines + 3);//��2
	LG->FitLine(Points + 40, 10, Lines + 6);//��3
	LG->FitLine(Points + 60, 10, Lines + 9);//��4
	////////////////LG->FitLine(CrossEdgePoints, (CrossLength1 + CrossLength2) / 2, Lines + 12);//��5,���뽻����1
	////////////////LG->FitLine(CrossEdgePoints + CrossLength1 + CrossLength2, (CrossLength3 + CrossLength4) / 2, Lines + 15);//��6,���뽻����2
	////////
	LG->RelTwoEnds(Points[8], Points[9], Points[18], Points[19], 50, PLine1);
	LG->RelTwoEnds((Points[0] + Points[10]) / 2, (Points[1] + Points[11]) / 2, centerpixel.x, centerpixel.y, 50, VLine);
	PreciseLine(Points[0], Points[1], PLine1, VLine, Lines, centerpixel);

	LG->RelTwoEnds(Points[28], Points[29], Points[38], Points[39], 50, PLine1);
	LG->RelTwoEnds((Points[20] + Points[30]) / 2, (Points[21] + Points[31]) / 2, centerpixel.x, centerpixel.y, 50, VLine);
	PreciseLine(Points[20], Points[21], PLine1, VLine, Lines + 3, centerpixel);

	LG->RelTwoEnds(Points[48], Points[49], Points[58], Points[59], 50, PLine1);
	LG->RelTwoEnds((Points[40] + Points[50]) / 2, (Points[41] + Points[51]) / 2, centerpixel.x, centerpixel.y, 50, VLine);
	PreciseLine(Points[40], Points[41], PLine1, VLine, Lines + 6, centerpixel);

	LG->RelTwoEnds(Points[68], Points[69], Points[78], Points[79], 50, PLine1);
	LG->RelTwoEnds((Points[60] + Points[70]) / 2, (Points[61] + Points[71]) / 2, centerpixel.x, centerpixel.y, 50, VLine);
	PreciseLine(Points[60], Points[61], PLine1, VLine, Lines + 9, centerpixel);

	LG->IntersectTwoLine(Lines[0], Lines[1], Lines[2], Lines[6], Lines[7], Lines[8], End[0], End[1]);//��1��3
	LG->IntersectTwoLine(Lines[0], Lines[1], Lines[2], Lines[9], Lines[10], Lines[11], End[2], End[3]);//��1��4
	LG->IntersectTwoLine(Lines[3], Lines[4], Lines[5], Lines[6], Lines[7], Lines[8], End[4], End[5]);//��2��3
	LG->IntersectTwoLine(Lines[3], Lines[4], Lines[5], Lines[9], Lines[10], Lines[11], End[6], End[7]);//��2��4

	length1 = LG->DisTwoPoints(End[0], End[1], End[2], End[3]);
	length2 = LG->DisTwoPoints(End[4], End[5], End[6], End[7]);
	length3 = LG->DisTwoPoints(End[0], End[1], End[4], End[5]);
	length4 = LG->DisTwoPoints(End[2], End[3], End[6], End[7]);
	if (length1 / length2 > 3 || length2 / length1 > 3
		|| length3 / length4 > 3 || length4 / length3 > 3
		|| length1 / length3 > 3 || length3 / length1 > 3
		|| length1 / length4 > 3 || length4 / length1 > 3
		|| length2 / length3 > 3 || length3 / length2 > 3
		|| length2 / length4 > 3 || length4 / length2 > 3)
		return false;
	return true;
}
bool MultiCoTarRecog::PreciseLine(int ustart, int vstart, int* ForLine, int* LaLine, double* line, cv::Point2i Center)
{
	int ub, vb, ubs, vbs, impb, impbs, uf, vf, impf, ut, vt;
	int temp1, temp2, temp3;
	int ForInStep;
	int LsearchLine[33];
	bool searchline1, searchline2;
	float ratio;
	float EdgePoints[800];
	int EHead = 0;
	if (LG->DisTwoPoints(ustart + LaLine[12], vstart + LaLine[13], Center.x, Center.y) > LG->DisTwoPoints(ustart, vstart, Center.x, Center.y))
	{
		//����һ������ָ���ڵĲ���������
		for (int i = 0; i < 15; i = i + 3)
		{
			LsearchLine[i] = LaLine[12 - i];
			LsearchLine[i + 1] = LaLine[13 - i];
			LsearchLine[i + 2] = LaLine[14 - i];
			LsearchLine[i + 18] = -LaLine[i];
			LsearchLine[i + 19] = -LaLine[i + 1];
			LsearchLine[i + 20] = -LaLine[i + 2];
		}
	}
	else
	{
		for (int i = 0; i < 15; i = i + 3)
		{
			LsearchLine[i] = -LaLine[12 - i];
			LsearchLine[i + 1] = -LaLine[13 - i];
			LsearchLine[i + 2] = -LaLine[14 - i];
			LsearchLine[i + 18] = LaLine[i];
			LsearchLine[i + 19] = LaLine[i + 1];
			LsearchLine[i + 20] = LaLine[i + 2];
		}
	}
	LsearchLine[15] = 0;
	LsearchLine[16] = 0;
	LsearchLine[17] = 0;
	ubs = ustart;
	vbs = vstart;
	impbs = vbs * width + ubs;
	ForInStep = 0;
	for (int l = 0; l < 200; l = l + 1)//�򵥲���������������200��
	{
		ub = ubs + ForLine[ForInStep];
		vb = vbs + ForLine[ForInStep + 1];
		impb = impbs + ForLine[ForInStep + 2];
		if (ub < 0 || ub >= width || vb < 0 || vb >= height)
			goto OtherSide;

		/*********************************************************/
		//�ж��Ƿ���ֹ
		if (l < 10)
			goto NotEnd1;
		uf = ub + LsearchLine[30];//�ڲ���λ��ǰ�������ڲ�ƫ��5�����ص�λ�ã�LsearchLine[15]���е㣩
		vf = vb + LsearchLine[31];//�ڲ��������
		impf = impb + LsearchLine[32];
		searchline1 = false;
		for (int i = 0; i < 21; i = i + 3)//�����߳�7,���ڲ��������Ƿ������ڰ�ͻ��
		{
			ut = uf + ForLine[i + 6];
			vt = vf + ForLine[i + 7];
			if (ut < 0 || ut >= width || vt < 0 || vt >= height)
				break;
			if (SData[impf + ForLine[i + 8]] > 60 && SData[impf + ForLine[i + 2]] < 80 && SData[impf + ForLine[i + 8]] - SData[impf + ForLine[i + 2]]>40)
			{
				searchline1 = true;
				break;
			}
		}
		if (searchline1)
		{
			uf = ub + LsearchLine[24];//�ڲ���λ��ǰ�������ڲ�ƫ��3�����ص�λ�ã�LsearchLine[15]���е㣩
			vf = vb + LsearchLine[25];//�ڲ��������
			impf = impb + LsearchLine[26];
			for (int i = 0; i < 21; i = i + 3)//�����߳�7,���ڲ��������Ƿ������ڰ�ͻ��
			{
				ut = uf + ForLine[i + 6];
				vt = vf + ForLine[i + 7];
				if (ut < 0 || ut >= width || vt < 0 || vt >= height)
					break;
				if (SData[impf + ForLine[i + 8]] > 60 && SData[impf + ForLine[i + 2]] < 80 && SData[impf + ForLine[i + 8]] - SData[impf + ForLine[i + 2]]>40)
					goto OtherSide;
			}
		}
		/*********************************************************/
	NotEnd1:
		ut = ub + LsearchLine[0];
		vt = vb + LsearchLine[1];
		if (ut < 0 || ut >= width || vt < 0 || vt >= height)
			continue;
		for (int i = 3; i < 24; i = i + 3)
		{
			temp1 = SData[impb + LsearchLine[i + 5]] - SData[impb + LsearchLine[i - 1]];
			temp2 = SData[impb + LsearchLine[i + 8]] - SData[impb + LsearchLine[i + 2]];
			ut = ub + LsearchLine[i + 6];
			vt = vb + LsearchLine[i + 7];
			temp3 = SData[impb + LsearchLine[i + 11]] - SData[impb + LsearchLine[i + 5]];
			if (SData[impb + LsearchLine[i + 2]] > 40 && temp2 < -20 && temp2 <= temp1 && temp2 <= temp3)
			{
				SubPixelPosition(ub + LsearchLine[i], vb + LsearchLine[i + 1], ub + LsearchLine[i + 3],
					vb + LsearchLine[i + 4], ub + LsearchLine[i + 6], vb + LsearchLine[i + 7], SData[impb + LsearchLine[i + 2]],
					SData[impb + LsearchLine[i + 5]], SData[impb + LsearchLine[i + 8]], EdgePoints + EHead);

				/*ratio = (SData[impb + LsearchLine[i + 5]] - SData[impb + LsearchLine[i + 2]]) / (float)temp2;
				EdgePoints[EHead] = ratio*(ub + LsearchLine[i]) + (1-ratio)*(ub + LsearchLine[i + 6]);
				EdgePoints[EHead+1] = ratio*(vb + LsearchLine[i + 1]) + (1-ratio)*(vb + LsearchLine[i + 7]);*/
				/////*temp1 = floor(EdgePoints[EHead + 1] + 0.5)*width + EdgePoints[EHead];
				////RData[temp1 * 3] = 0;
				////RData[temp1 * 3 + 1] = 255;
				////RData[temp1 * 3 + 2] = 0;*/
				EHead = EHead + 2;
				break;
			}
		}
		if (ForInStep == 33)
		{
			ubs = floor(EdgePoints[EHead - 2] + 0.5);
			vbs = floor(EdgePoints[EHead - 1] + 0.5);
			impbs = vbs * width + ubs;
			ForInStep = 0;
		}
		else
			ForInStep = ForInStep + 3;
	}
OtherSide:
	ubs = ustart;
	vbs = vstart;
	impbs = vbs * width + ubs;
	ForInStep = 0;
	for (int l = 0; l < 200; l = l + 1)//�򵥲���������������200��
	{
		ub = ubs - ForLine[ForInStep];
		vb = vbs - ForLine[ForInStep + 1];
		impb = impbs - ForLine[ForInStep + 2];
		if (ub < 0 || ub >= width || vb < 0 || vb >= height)
			goto SearchFinish;

		/*********************************************************/
		//�ж��Ƿ���ֹ
		if (l < 10)
			goto NotEnd2;
		uf = ub + LsearchLine[30];//�ڲ���λ��ǰ�������ڲ�ƫ��3�����ص�λ�ã�LsearchLine[15]���е㣩
		vf = vb + LsearchLine[31];//�ڲ��������
		impf = impb + LsearchLine[32];
		searchline1 = false;
		for (int i = 0; i < 21; i = i + 3)//�����߳�7,���ڲ��������Ƿ������ڰ�ͻ��
		{
			ut = uf - ForLine[i + 6];
			vt = vf - ForLine[i + 7];
			if (ut < 0 || ut >= width || vt < 0 || vt >= height)
				break;
			if (SData[impf - ForLine[i + 8]] > 60 && SData[impf - ForLine[i + 2]] < 80 && SData[impf - ForLine[i + 8]] - SData[impf - ForLine[i + 2]]>40)
			{
				searchline1 = true;
				break;
			}
		}
		if (searchline1)
		{
			uf = ub + LsearchLine[24];//�ڲ���λ��ǰ�������ڲ�ƫ��3�����ص�λ�ã�LsearchLine[15]���е㣩
			vf = vb + LsearchLine[25];//�ڲ��������
			impf = impb + LsearchLine[26];
			for (int i = 0; i < 21; i = i + 3)//�����߳�7,���ڲ��������Ƿ������ڰ�ͻ��
			{
				ut = uf - ForLine[i + 6];
				vt = vf - ForLine[i + 7];
				if (ut < 0 || ut >= width || vt < 0 || vt >= height)
					break;
				if (SData[impf - ForLine[i + 8]] > 60 && SData[impf - ForLine[i + 2]] < 80 && SData[impf - ForLine[i + 8]] - SData[impf - ForLine[i + 2]]>40)
					goto SearchFinish;
			}
		}
		/*********************************************************/
	NotEnd2:
		for (int i = 3; i < 24; i = i + 3)
		{
			ut = ub + LsearchLine[i - 3];
			vt = vb + LsearchLine[i - 2];
			if (ut < 0 || ut >= width || vt < 0 || vt >= height)
				break;
			temp1 = SData[impb + LsearchLine[i + 5]] - SData[impb + LsearchLine[i - 1]];
			temp2 = SData[impb + LsearchLine[i + 8]] - SData[impb + LsearchLine[i + 2]];
			ut = ub + LsearchLine[i + 6];
			vt = vb + LsearchLine[i + 7];
			temp3 = SData[impb + LsearchLine[i + 11]] - SData[impb + LsearchLine[i + 5]];
			if (SData[impb + LsearchLine[i + 2]] > 40 && temp2 < -20 && temp2 <= temp1 && temp2 <= temp3)
			{
				SubPixelPosition(ub + LsearchLine[i], vb + LsearchLine[i + 1], ub + LsearchLine[i + 3], vb + LsearchLine[i + 4], ub + LsearchLine[i + 6], vb + LsearchLine[i + 7], SData[impb + LsearchLine[i + 2]], SData[impb + LsearchLine[i + 5]], SData[impb + LsearchLine[i + 8]], EdgePoints + EHead);
				/*ratio = (SData[impb + LsearchLine[i + 5]] - SData[impb + LsearchLine[i + 2]]) / (float)temp2;
				EdgePoints[EHead] = ratio*(ub + LsearchLine[i]) + (1 - ratio)*(ub + LsearchLine[i + 6]);
				EdgePoints[EHead + 1] = ratio*(vb + LsearchLine[i + 1]) + (1 - ratio)*(vb + LsearchLine[i + 7]);*/
				///////*temp1 = floor(EdgePoints[EHead + 1] + 0.5)*width + EdgePoints[EHead];
				//////RData[temp1 * 3] = 0;
				//////RData[temp1 * 3 + 1] = 255;
				//////RData[temp1 * 3 + 2] = 0;*/
				EHead = EHead + 2;
				break;
			}
		}
		if (ForInStep == 33)
		{
			ubs = floor(EdgePoints[EHead - 2] + 0.5);
			vbs = floor(EdgePoints[EHead - 1] + 0.5);
			impbs = vbs * width + ubs;
			ForInStep = 0;
		}
		else
			ForInStep = ForInStep + 3;
	}
SearchFinish:
	if (EHead < 30)
		return false;
	LG->FitLine(EdgePoints, EHead / 2, line);//��1���������������ı�Եʮ����������ֱ��
	return true;
}
int MultiCoTarRecog::ALongCross(cv::Point2i Center, int* PLine, int*VLine, float*SavePoints, int maxsteps)//����������Ѱ�ұ�Ե����ʼ��
{
	int total_steps = -1;
	int ub, vb, impb;//ǰ������ʱ�Ļ����ϵ�����
	int ublack, vblack, impblack, ublack7, vblack7, impblack7, uend, vend, impend;
	int uwhite, vwhite, impwhite, uwhite5, vwhite5, impwhite5;
	int updateP[3];
	int bp = Center.y*width + Center.x;
	int temp, temp1, temp2;
	int phead;//������������ľ��α߿�㣬ÿ���ҵ�ʮ�����������ĵ㼴��Ϊok
	int ratio1, ratio2;
	int VerdiTwo[39];
	int white_ok = 0;
	for (int i = 0; i < 18; i = i + 3)
	{
		VerdiTwo[i] = -VLine[15 - i];
		VerdiTwo[i + 1] = -VLine[15 - i + 1];
		VerdiTwo[i + 2] = -VLine[15 - i + 2];
		VerdiTwo[i + 21] = VLine[i];
		VerdiTwo[i + 22] = VLine[i + 1];
		VerdiTwo[i + 23] = VLine[i + 2];
	}
	VerdiTwo[18] = 0;
	VerdiTwo[19] = 0;
	VerdiTwo[20] = 0;
	//���������������ǰ���߷�������������ǰ���ߺ�ʵ���߼�ض���������������������������ӣ�ƫ�����������
	//һ�������ۼ����ķ�ʽ�ǲ��ϸ���������㣬��ÿ���жϴ���һ���жϽ������������ɴ�����������������ͷ���ױ�
	//��������,һ�ֽ��˼·�Ƿֶ���ǰ��ֱ������������ÿǰ��ʮ���ص�������
	int ForInStep = 0;
	int ubs, vbs, impbs;//ÿ����������ʼλ�õ�����
	ubs = Center.x + PLine[21];
	vbs = Center.y + PLine[22];
	impbs = bp + PLine[23];
	int sumleft = 0, sumright = 0;
	for (int i = 2; i < 18; i = i + 3)
		sumleft = sumleft + SData[impbs + VerdiTwo[i]];
	for (int i = 23; i < 39; i = i + 3)
		sumright = sumright + SData[impbs + VerdiTwo[i]];
	if (sumleft > sumright)//VerdiTwo��VLine�ڰ�ָ����ͬ
		LG->ArryMinus(39, VerdiTwo);//��֤��ɫ�������������
	for (int count = 0; count < maxsteps; count = count + 1)
	{
		//ub ����ǰ����ubsÿʮ�θ���һ��
		ub = ubs + PLine[ForInStep];
		vb = vbs + PLine[ForInStep + 1];
		impb = impbs + PLine[ForInStep + 2];
		ublack = ub + VerdiTwo[0];//�ڵ�Ϊǰ���������ɫ�����ƶ�6����
		vblack = vb + VerdiTwo[1];
		impblack = impb + VerdiTwo[2];
		ublack7 = ublack + PLine[21];//��7Ϊ�ڵ���ǰ�ƶ�7����
		vblack7 = vblack + PLine[22];
		impblack7 = impblack + PLine[23];

		if (white_ok == 0)
		{
			//��ɫ������ǰA����̽�⵽��Ϊ֮��Ԥ����B�κ�ɫ̽����ᡣ
			//��ɫ������ǰC����̽�⡣�׵�̽�����ĺ�ɫ��ȳ���C-A+B�ͻᷢ��ʧ�ܡ�
			//ѡȡA=5 ��C=7��B=11�������Ϊ13
			uwhite = ub + VerdiTwo[33];//�׵�Ϊ�������ɫ�����ƶ�6����
			vwhite = vb + VerdiTwo[34];
			impwhite = impb + VerdiTwo[35];
			uwhite5 = uwhite + PLine[15];
			vwhite5 = vwhite + PLine[16];
			impwhite5 = impwhite + PLine[17];
			if (uwhite5 < 0 || uwhite5 >= width || vwhite5 < 0 || vwhite5 >= height)
			{
				return -1;
				break;
			}
			if (SData[impwhite + PLine[2]] - SData[impwhite5] > 40)
			{
				white_ok = 11;
			}
		}


		if (ublack7 < 0 || ublack7 >= width || vblack7 < 0 || vblack7 >= height)
		{
			return -1;
			break;
		}
		if (white_ok > 0 && SData[impblack7] - SData[impblack + PLine[17]] > 40)
		{
			uend = ublack + PLine[18];
			vend = vblack + PLine[19];
			impend = impblack + PLine[20];
			total_steps = count;
			break;
		}
		if (white_ok > 0 && SData[impblack + PLine[8]] - SData[impblack + PLine[2]] > 40)
		{
			uend = ublack + PLine[3];
			vend = vblack + PLine[4];
			impend = impblack + PLine[5];
			total_steps = count;
			break;
		}
		if (white_ok > 0)
			white_ok = white_ok - 1;
		ForInStep = ForInStep + 3;
		if (ForInStep == 33)
		{
			LG->CliffAlong(ub, vb, impb, 13, SData, VerdiTwo, width, height, updateP);
			ubs = updateP[0];
			vbs = updateP[1];
			impbs = updateP[2];
			ForInStep = 0;
		}
	}
	///*if (total_steps != -1)
	//{
	//	circle(ImageResult, cvPoint(uend, vend), 10, CV_RGB(0, 0, 255), 2, 8, 0);
	//}*/
	if (total_steps == -1)
		return total_steps;

	int iter = 0, iter_save = 0;
	int PTwo[33];
	for (int i = 0; i < 15; i = i + 3)
	{
		PTwo[i] = -PLine[12 - i];
		PTwo[i + 1] = -PLine[12 - i + 1];
		PTwo[i + 2] = -PLine[12 - i + 2];
		PTwo[i + 18] = PLine[i];
		PTwo[i + 19] = PLine[i + 1];
		PTwo[i + 20] = PLine[i + 2];
	}
	PTwo[15] = 0;
	PTwo[16] = 0;
	PTwo[17] = 0;
	for (int i = 0; i < 5; i++)
	{
		if (sumleft > sumright)//����VLine�ǴӰ�ָ���
		{
			ubs = uend + VLine[iter];
			vbs = vend + VLine[iter + 1];
			impbs = impend + VLine[iter + 2];
		}
		else
		{
			ubs = uend - VLine[iter];
			vbs = vend - VLine[iter + 1];
			impbs = impend - VLine[iter + 2];
		}

		if (!LG->PreciseCliff(ubs, vbs, impbs, 11, SData, PTwo, width, height, SavePoints + iter_save, 'P'))
			return -1;
		iter_save = iter_save + 2;
		iter = iter + 3;
	}
	//uend,vend�Ǻ�ɫ����6�����ǰԵ������Ҫ����������ɫ�����ٲɼ����ɵ�
	if (sumleft > sumright)
	{
		uend = uend - VLine[36];
		vend = vend - VLine[37];
		impend = impend - VLine[38];
	}
	else
	{
		uend = uend + VLine[36];
		vend = vend + VLine[37];
		impend = impend + VLine[38];
	}
	ubs = uend - PLine[12];
	vbs = vend - PLine[13];
	impbs = impend - PLine[14];
	int to_white = 0;
	bool got_new_end = false;
NEW_END:
	for (int i = 2; i < 32; i = i + 3)
	{
		temp = SData[impbs + PLine[i + 6]] - SData[impbs + PLine[i]];
		if (temp > 35 && temp > SData[impbs + PLine[i + 9]] - SData[impbs + PLine[i + 3]])
		{
			uend = ubs + PLine[i + 1];
			vend = vbs + PLine[i + 2];
			impend = impbs + PLine[i + 3];
			got_new_end = true;
			break;
		}
	}
	if (!got_new_end)
	{
		if (to_white < 18)
		{
			if (sumleft > sumright)
			{
				ubs = ubs - VLine[to_white];
				vbs = vbs - VLine[to_white + 1];
				impbs = impbs - VLine[to_white + 2];
			}
			else
			{
				ubs = ubs + VLine[to_white];
				vbs = vbs + VLine[to_white + 1];
				impbs = impbs + VLine[to_white + 2];
			}
			to_white = to_white + 3;
			goto NEW_END;
		}
		else
			return -1;
	}
	iter = 0;
	for (int i = 0; i < 5; i++)
	{
		if (sumleft > sumright)//����VLine�ǴӰ�ָ���
		{
			ubs = uend - VLine[iter];
			vbs = vend - VLine[iter + 1];
			impbs = impend - VLine[iter + 2];
		}
		else
		{
			ubs = uend + VLine[iter];
			vbs = vend + VLine[iter + 1];
			impbs = impend + VLine[iter + 2];
		}

		if (!LG->PreciseCliff(ubs, vbs, impbs, 11, SData, PTwo, width, height, SavePoints + iter_save, 'P'))
			return -1;
		iter_save = iter_save + 2;
		iter = iter + 3;
	}
	return total_steps;
}
int MultiCoTarRecog::CircleCheck(int DataP, int cn)
{
	int CircleS, CircleL;
	int see[150];
	int Block[5], BHead = 0;
	int HTineffective[10], HHead = 0;//���ÿһ�ε���β��Ч��
	int Semmtry[5];
	int uu, vv, checkp, checkp1;
	float thresh1, thresh2;
	Block[0] = 0;
	Block[1] = 0;
	Block[2] = 0;
	Block[3] = 0;
	Block[4] = 0;

	Semmtry[0] = 0;
	Semmtry[1] = 0;
	Semmtry[2] = 0;
	Semmtry[3] = 0;
	Semmtry[4] = 0;

	CircleS = Circle[cn];//Բ��ģ�����ʼλ��
	CircleL = Circle[cn + 1] - Circle[cn] - 1;//��ѡ��ģ������ݳ���
	thresh1 = max((CircleL + 1) / 20.1, 4.0);
	thresh2 = max((CircleL + 1) / 5.1, 4.0);
	int jumps = 0;
	if ((CData[DataP + Circle[CircleS + CircleL]] != 0) && (CData[DataP + Circle[CircleS]] == 0))
	{
		HTineffective[0] = 0;
		HHead = 1;
	}
	for (int i = 0; i < CircleL; i++)
	{
		checkp = DataP + Circle[i + CircleS];
		checkp1 = DataP + Circle[i + 1 + CircleS];
		see[i] = SData[checkp];
		/*uu = checkp % width;
		vv = checkp / width;*/
		if (CData[checkp] == 0)
		{
			Block[BHead] = Block[BHead] + 1;
			Semmtry[BHead] = Semmtry[BHead] + SData[checkp];
		}
		if ((CData[checkp] == 0) && (CData[checkp1] != 0))
		{
			jumps++;
			BHead++;
			HTineffective[HHead] = i;
			HHead++;
		}
		if ((CData[checkp] != 0) && (CData[checkp1] == 0))
		{
			HTineffective[HHead] = -i - 1;
			HHead++;
		}
		if (jumps > 4)
			return 2;
	}
	see[CircleL] = SData[DataP + Circle[CircleS + CircleL]];
	checkp1 = DataP + Circle[CircleS];
	if ((CData[DataP + Circle[CircleS + CircleL]] == 0) && (CData[checkp1] != 0))
	{
		jumps++;
		HTineffective[HHead] = CircleL;
	}
	else
	{
		Block[0] = Block[0] + Block[4];
		Semmtry[0] = Semmtry[0] + Semmtry[4];
	}
	if (jumps < 4)
		return 1;
	if (jumps != 4)
		return 2;
	if (abs(Block[0] - Block[2]) > thresh1 || abs(Block[1] - Block[3]) > thresh1)//�ж϶Խ�����������Ƿ�ӽ�
		return 2;
	if (abs(Block[0] - Block[1]) > thresh2 || abs(Block[1] - Block[3]) > thresh2)
		return 2;

	if (HTineffective[1] < 0)//˵����һ������0��Ϊ0��255��
	{
		Semmtry[0] = Semmtry[0] - see[HTineffective[0]] - see[-HTineffective[7]];
		Semmtry[1] = Semmtry[1] - see[HTineffective[2]] - see[-HTineffective[1]];
		Semmtry[2] = Semmtry[2] - see[HTineffective[4]] - see[-HTineffective[3]];
		Semmtry[3] = Semmtry[3] - see[HTineffective[6]] - see[-HTineffective[5]];
	}
	else
	{
		Semmtry[0] = Semmtry[0] - see[HTineffective[1]] - see[-HTineffective[0]];
		Semmtry[1] = Semmtry[1] - see[HTineffective[3]] - see[-HTineffective[2]];
		Semmtry[2] = Semmtry[2] - see[HTineffective[5]] - see[-HTineffective[4]];
		Semmtry[3] = Semmtry[3] - see[HTineffective[7]] - see[-HTineffective[6]];
	}
	Block[0] = Block[0] - 2;
	Block[1] = Block[1] - 2;
	Block[2] = Block[2] - 2;
	Block[3] = Block[3] - 2;
	if (abs(Semmtry[0] / Block[0] - Semmtry[2] / Block[2]) > 40)
		return 2;
	if (abs(Semmtry[1] / Block[1] - Semmtry[3] / Block[3]) > 40)
		return 2;
	if (abs(Semmtry[0] / Block[0] - Semmtry[1] / Block[1]) < 40)
		return 2;
	if (abs(Semmtry[2] / Block[2] - Semmtry[3] / Block[3]) < 40)
		return 2;

	return 0;
}

int MultiCoTarRecog::ChainAlternate(int* Chain, int ChainLength, int Threshold, int DIntv, int* SavePosition)
{
	int hc, D1[200], D2[200], temp, temp1, hD, hD2;
	hc = ChainLength - DIntv;
	hD = 0;
	//��ȡ��������
	for (int i = 0; i < hc; i++)
	{
		temp = Chain[i] - Chain[i + DIntv];
		if (abs(temp) > Threshold)//�������ͻ�䣬��������������λ�ô���D1��
		{
			D1[hD] = temp;
			D1[hD + 1] = i;
			hD = hD + 2;
			if (hD > 150)//������̫��ͻ���϶�����
				return 0;
		}
	}
	for (int i = hc; i < ChainLength; i++)//��������ѭ���ģ�����β��������ĩ���������⴦��
	{
		temp = Chain[i] - Chain[i - hc];
		if (abs(temp) > Threshold)
		{
			D1[hD] = temp;
			D1[hD + 1] = i;
			hD = hD + 2;
		}
	}
	//���ϵõ���ͻ���ش��ĵ�������õ���������
	//������������й�����������ͬ�ŵ�����ֻ������������
	hD2 = 0;
	temp = D1[0];
	temp1 = D1[1];
	for (int i = 2; i <= hD; i = i + 2)
	{
		if (i == hD)
			goto Last;
		if (temp > 0 == D1[i] > 0)//�ж����ڵ�����ͻ���Ƿ�ͬ�ţ����ͬ�Ž��й鲢
		{
			if (temp > 0)
			{
				temp = max(temp, D1[i]);
				temp1 = (temp > D1[i]) ? temp1 : D1[i + 1];
			}
			else
			{
				temp = min(temp, D1[i]);
				temp1 = (temp < D1[i]) ? temp1 : D1[i + 1];
			}
		}//���ǰ��ͬ�ţ�����������ͻ��λ��
		else
		{
		Last:
			SavePosition[hD2] = temp;
			SavePosition[hD2 + 1] = temp1;
			temp = D1[i];
			temp1 = D1[i + 1];
			hD2 = hD2 + 2;
			if (hD2 > 10)
				return 0;
		}//�����ж��µ�һ��
	}
	if (SavePosition[0] > 0 == SavePosition[hD2 - 2] > 0)//��β����
	{
		if (SavePosition[0] > 0)
		{
			SavePosition[0] = max(SavePosition[0], SavePosition[hD2 - 2]);
			SavePosition[1] = (SavePosition[0] > SavePosition[hD2 - 2]) ? SavePosition[1] : SavePosition[hD2 - 1];
		}
		else
		{
			SavePosition[0] = min(SavePosition[0], SavePosition[hD2 - 2]);
			SavePosition[1] = (SavePosition[0] < SavePosition[hD2 - 2]) ? SavePosition[1] : SavePosition[hD2 - 1];
		}
		hD2 = hD2 - 2;
	}
	return hD2;
}
bool MultiCoTarRecog::TenPoints()
{
	//float Length;
	int /*PixLength,*/OneThree, TwoThree/*,CenterP*/;
	//int StucLine[3000];
	/**********************************************************************************/
	//���ĵ�1�ŵ�
	LG->IntersectTwoLine(LineFunc[0], LineFunc[1], LineFunc[2], LineFunc[3], LineFunc[4], LineFunc[5], IntSec[0], IntSec[1]);
	/**********************************************************************************/

	/**********************************************************************************/
	//ʮ�ֵ�
	LG->IntersectTwoLine(LineFunc[0], LineFunc[1], LineFunc[2], LineFunc[6], LineFunc[7], LineFunc[8], IntSec[2], IntSec[3]);//1��3���߽���2�ŵ�
	LG->IntersectTwoLine(LineFunc[0], LineFunc[1], LineFunc[2], LineFunc[9], LineFunc[10], LineFunc[11], IntSec[4], IntSec[5]);//1��4���߽���3�ŵ�

	LG->IntersectTwoLine(LineFunc[3], LineFunc[4], LineFunc[5], LineFunc[12], LineFunc[13], LineFunc[14], IntSec[6], IntSec[7]);//2��5���߽���4�ŵ�
	LG->IntersectTwoLine(LineFunc[3], LineFunc[4], LineFunc[5], LineFunc[15], LineFunc[16], LineFunc[17], IntSec[8], IntSec[9]);//2��6���߽���5�ŵ�
	/**********************************************************************************/

	/**********************************************************************************/
	//ʮ�ֵ��Ķ���
	LG->IntersectTwoLine(LineFunc[6], LineFunc[7], LineFunc[8], LineFunc[12], LineFunc[13], LineFunc[14], IntSec[10], IntSec[11]);//3��5���߽���6�ŵ�
	LG->IntersectTwoLine(LineFunc[6], LineFunc[7], LineFunc[8], LineFunc[15], LineFunc[16], LineFunc[17], IntSec[12], IntSec[13]);//3��6���߽���7�ŵ�

	LG->IntersectTwoLine(LineFunc[9], LineFunc[10], LineFunc[11], LineFunc[12], LineFunc[13], LineFunc[14], IntSec[14], IntSec[15]);//4��5���߽���8�ŵ�
	LG->IntersectTwoLine(LineFunc[9], LineFunc[10], LineFunc[11], LineFunc[15], LineFunc[16], LineFunc[17], IntSec[16], IntSec[17]);//4��6���߽���9�ŵ�
	/**********************************************************************************/

	/**********************************************************************************/
	//�е�ָ��6�ŵ�
	OneThree = width * floor(IntSec[1] + (IntSec[11] - IntSec[1]) / 3 + 0.5) + floor(IntSec[0] + (IntSec[10] - IntSec[0]) / 3 + 0.5);
	TwoThree = width * floor(IntSec[1] + (IntSec[11] - IntSec[1]) * 2 / 3 + 0.5) + floor(IntSec[0] + (IntSec[10] - IntSec[0]) * 2 / 3 + 0.5);
	if (SData[OneThree] - SData[TwoThree] > 35)
	{
		IntSec[18] = IntSec[10];
		IntSec[19] = IntSec[11];
		return true;
	}
	/**********************************************************************************/

	/**********************************************************************************/
	//�е�ָ��7�ŵ�
	OneThree = width * floor(IntSec[1] + (IntSec[13] - IntSec[1]) / 3 + 0.5) + floor(IntSec[0] + (IntSec[12] - IntSec[0]) / 3 + 0.5);
	TwoThree = width * floor(IntSec[1] + (IntSec[13] - IntSec[1]) * 2 / 3 + 0.5) + floor(IntSec[0] + (IntSec[12] - IntSec[0]) * 2 / 3 + 0.5);
	if (SData[OneThree] - SData[TwoThree] > 40)
	{
		IntSec[18] = IntSec[12];
		IntSec[19] = IntSec[13];
		return true;
	}
	/**********************************************************************************/

	/**********************************************************************************/
	//�е�ָ��8�ŵ�
	OneThree = width * floor(IntSec[1] + (IntSec[15] - IntSec[1]) / 3 + 0.5) + floor(IntSec[0] + (IntSec[14] - IntSec[0]) / 3 + 0.5);
	TwoThree = width * floor(IntSec[1] + (IntSec[15] - IntSec[1]) * 2 / 3 + 0.5) + floor(IntSec[0] + (IntSec[14] - IntSec[0]) * 2 / 3 + 0.5);
	if (SData[OneThree] - SData[TwoThree] > 40)
	{
		IntSec[18] = IntSec[14];
		IntSec[19] = IntSec[15];
		return true;
	}
	/**********************************************************************************/

	/**********************************************************************************/
	//�е�ָ��9�ŵ�
	OneThree = width * floor(IntSec[1] + (IntSec[17] - IntSec[1]) / 3 + 0.5) + floor(IntSec[0] + (IntSec[16] - IntSec[0]) / 3 + 0.5);
	TwoThree = width * floor(IntSec[1] + (IntSec[17] - IntSec[1]) * 2 / 3 + 0.5) + floor(IntSec[0] + (IntSec[16] - IntSec[0]) * 2 / 3 + 0.5);
	if (SData[OneThree] - SData[TwoThree] > 40)
	{
		IntSec[18] = IntSec[16];
		IntSec[19] = IntSec[17];
		return true;
	}
	/**********************************************************************************/



	return false;
}

int MultiCoTarRecog::InterSecHough(cv::Mat Scr, int u, int v)
{
	//CString strPath;
	//Mat temp27;
	cv::Mat contours;
	//temp27 = Scr(Range(v - 15, v + 16), Range(u - 15, u + 16));
	contours = ImageCanny(cv::Range(v - 15, v + 16), cv::Range(u - 15, u + 16));
	//��Ե���   
	//cv::namedWindow("contours");
	//cv::imshow("contours", contours);
	/*strPath = "D:\\�Ӿ�����ϵͳ\\ͼ��ɼ�\\cannyblock.bmp";
	imwrite(strPath.GetBuffer(0), contours);*/
	std::vector<cv::Vec2f> lines;
	//����任,���һ�鼫���������rho��theta��,ÿһ�Զ�Ӧһ��ֱ�ߣ����浽lines   
	//��3,4��������ʾ�ڣ�rho��theta)����ϵ������������С��λ��������   
	cv::HoughLines(contours, lines, 1, 0.017, 12);
	std::vector<cv::Vec2f>::const_iterator it = lines.begin();

	if (lines.size() < 2)
		return 1;
	double LineRe[8];
	double tempsave[100];
	int thead = 0, lhead = 0;
	//float temp;
	/*LineRe[0] = (*it)[0];
	LineRe[1] = (*it)[1];*/
	int linenumber = lines.end() - lines.begin();
	double ro, theta;
	for (int i = 0; i < linenumber - 1; i++)
	{
		if (lines[i][1] == -10)
			continue;
		if (lhead > 4)
			return 2;
		tempsave[0] = lines[i][0];
		tempsave[1] = lines[i][1];
		thead = 2;
		for (int ii = i + 1; ii < linenumber; ii++)
		{
			if (lines[ii][1] == -10)
				continue;
			if (fabs(lines[i][1] - lines[ii][1]) < 0.2)
			{
				tempsave[thead] = lines[ii][0];
				tempsave[thead + 1] = lines[ii][1];
				thead = thead + 2;
				lines[ii][1] = -10;
				continue;
			}
			if (fabs(lines[i][1] - lines[ii][1]) > 2.94159)
			{
				tempsave[thead] = -lines[ii][0];
				if (lines[ii][1] > 1.5708)
					tempsave[thead + 1] = lines[ii][1] - 3.1415926;
				else
					tempsave[thead + 1] = lines[ii][1] + 3.1415926;
				thead = thead + 2;
				lines[ii][1] = -10;
				continue;
			}
		}
		ro = 0;
		theta = 0;
		for (int k = 0; k < thead; k = k + 2)
		{
			ro = ro + tempsave[k];
			theta = theta + tempsave[k + 1];
		}
		LineRe[lhead] = ro * 2 / thead;
		LineRe[lhead + 1] = theta * 2 / thead;
		lhead = lhead + 2;
	}
	if (lines[linenumber - 1][1] != -10)
	{
		LineRe[2] = lines[linenumber - 1][0];
		LineRe[3] = lines[linenumber - 1][1];
		lhead = lhead + 2;
	}
	//ֱ�߷���Ϊr=xcos+ysin
	double temp = fabs(LineRe[3] - LineRe[1]);
	temp = min(temp, double(3.1416 - temp));
	if (temp < 0.5)
		return 6;
	double interpoint[2];
	LG->IntersectTwoLine(cos(LineRe[1]), sin(LineRe[1]), -LineRe[0], cos(LineRe[3]), sin(LineRe[3]), -LineRe[2], interpoint[0], interpoint[1]);
	float dis = LG->DisTwoPoints(interpoint[0], interpoint[1], 15, 15);
	if (dis > 100)
		return 7;
	if (dis > 4)
		return 8;
	CenterPoint.x = u - 15 + interpoint[0];
	CenterPoint.y = v - 15 + interpoint[1];
	TarPosition[0] = CenterPoint.x + 0.5;
	TarPosition[1] = CenterPoint.y + 0.5;

	CrossLine[0] = cos(LineRe[1]);
	CrossLine[1] = sin(LineRe[1]);
	CrossLine[2] = LineRe[0] - (u - 15)*CrossLine[0] - (v - 15)*CrossLine[1];
	CrossLine[3] = cos(LineRe[3]);
	CrossLine[4] = sin(LineRe[3]);
	CrossLine[5] = LineRe[2] - (u - 15)*CrossLine[3] - (v - 15)*CrossLine[4];
	return 0;
}
void MultiCoTarRecog::DrawLine(cv::Mat* Scr, int u1, int v1, int u2, int v2, int L_width, char R, char G, char B)
{
	int Du, Dv, uH, vH, aDu, Su, ustart, udes, vstart, vdes, a, b, i, m, w_h, width, widthstep, height;
	unsigned char* ImageData;
	ImageData = Scr->data;
	width = Scr->cols;
	widthstep = Scr->step;
	height = Scr->rows;
	w_h = L_width / 2;
	if (v1 < v2 || (v1 == v2 && u1 <= u2))
	{
		vstart = v1;
		ustart = u1;
		vdes = v2;
		udes = u2;
	}
	if (v1 > v2 || (v1 == v2 && u1 > u2))
	{
		vstart = v2;
		ustart = u2;
		vdes = v1;
		udes = u1;
	}
	Du = udes - ustart;
	Dv = vdes - vstart;
	aDu = abs(Du);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;
	uH = ustart;
	vH = vstart;
	if (aDu >= Dv)
	{
		for (i = 0; i < aDu + 1; i++)
		{
			a = abs((uH - ustart) * Dv - (vH - vstart) * Du);
			b = abs((uH - ustart) * Dv - (vH + 1 - vstart) * Du);
			if (a > b)
				vH = vH + 1;
			for (m = -w_h; m <= w_h; m++)
			{
				a = vH + m;
				if (a < 0 || a >= height)
					continue;
				b = a * widthstep + uH * 3;
				if (a < 0 || a >= height || uH < 0 || uH >= width)
					continue;
				ImageData[b] = B;
				ImageData[b + 1] = G;
				ImageData[b + 2] = R;
			}
			uH = uH + Su;
		}
	}
	if (aDu < Dv)
	{
		for (i = 0; i < Dv + 1; i++)
		{
			if (vH >= height)
				break;
			a = abs((uH - ustart) * Dv - (vH - vstart) * Du);
			b = abs((uH + Su - ustart) * Dv - (vH - vstart) * Du);
			if (a > b)
				uH = uH + Su;
			for (m = -w_h; m <= w_h; m++)
			{
				a = uH + m;
				if (a < 0 || a >= width)
					continue;
				b = vH * widthstep + a * 3;
				if (vH < 0 || vH >= height || a < 0 || a >= width)
					continue;
				ImageData[b] = B;
				ImageData[b + 1] = G;
				ImageData[b + 2] = R;
			}
			vH = vH + 1;
		}
	}
}
void MultiCoTarRecog::SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result)
{
	float ub1, vb1, ub2, vb2, ratio1, ratio2;
	float GrayStep;
	ub1 = (u1 + u2) / 2;
	vb1 = (v1 + v2) / 2;
	ub2 = (u2 + u3) / 2;
	vb2 = (v2 + v3) / 2;
	GrayStep = gray1 - gray3;
	if (GrayStep == 0)
		return;
	ratio1 = (gray1 - gray2) / GrayStep;
	ratio2 = (gray2 - gray3) / GrayStep;
	Result[0] = ub1 * ratio1 + ub2 * ratio2;
	Result[1] = vb1 * ratio1 + vb2 * ratio2;
}
int MultiCoTarRecog::FiveKeyPoints(int u_center, int v_center)//������ȡ����ֱ�ߺ����ֶ���ȷ�������ĸ�����ͱ�־���ĵ��ͼ�������˳�����б�־���ĵ�Ϊ�������
{
	int logo_type;//��־���
	int peak;//��ǵ���
	int OneThree1, OneThree2, OneThree3, OneThree4;
	int TwoThree1, TwoThree2, TwoThree3, TwoThree4;
	float angle1, angle2;//�����жϵ�˳��

	////LG->IntersectTwoLine(Lines[12], Lines[13], Lines[14], Lines[15], Lines[16], Lines[17], Result + 8);//��1��3
	Result[8] = u_center;
	Result[9] = v_center;
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

	logo_type = determ_code(SData[OneThree1], SData[TwoThree1], SData[OneThree2], SData[TwoThree2],
		SData[OneThree3], SData[TwoThree3], SData[OneThree4], SData[TwoThree4], peak);

	start_from_peak(End, Result, peak);
	return logo_type;

}
void MultiCoTarRecog::EdgeMap(cv::Mat* Input, cv::Mat*Output, int threshold, int threshold_adj, int scale_length)
{
	int in_w, in_h;
	in_w = Input->cols;
	in_h = Input->rows;
	cv::Mat V_Tab = cv::Mat(in_h, in_w, CV_8U, cv::Scalar::all(0));
	cv::Mat H_Tab = cv::Mat(in_h, in_w, CV_8U, cv::Scalar::all(0));
	uchar* IData = Input->data;
	uchar* OData = Output->data;
	uchar* VData = V_Tab.data;
	uchar* HData = H_Tab.data;
	int w_end, h_end, iter;
	int scale_lw = scale_length * in_w;
	int index_lk;

	w_end = in_w - scale_length;
	h_end = in_h - scale_length;
	iter = scale_lw;
	for (int l = scale_length; l < h_end; l++)
	{
		for (int k = scale_length; k < w_end; k++)
		{
			index_lk = iter + k;
			if (IData[index_lk] > 150)
				continue;
			///////*if (max(IData[index_lk - scale_length], IData[index_lk + scale_length]) < 100 &&
			//////	max(IData[index_lk - scale_lw], IData[index_lk + scale_lw]) < 100)
			//////	continue;*/
			/*//////if (min(IData[index_lk - scale_length], IData[index_lk + scale_length]) > 90 &&
			//////	min(IData[index_lk - scale_lw], IData[index_lk + scale_lw]) > 90)
			//////	continue;*/
			///*HData[index_lk] = abs(IData[index_lk] - IData[index_lk - scale_length]) + abs(IData[index_lk] - IData[index_lk + scale_length]);
			//VData[index_lk] = abs(IData[index_lk] - IData[index_lk - scale_lw]) + abs(IData[index_lk] - IData[index_lk + scale_lw]);*/
			if (abs(IData[index_lk - scale_length] - IData[index_lk + scale_length]) > threshold)
				HData[index_lk] = abs(IData[index_lk - 1] - IData[index_lk + 1]);
			if (abs(IData[index_lk - scale_lw] - IData[index_lk + scale_lw]) > threshold)
				VData[index_lk] = abs(IData[index_lk - in_w] - IData[index_lk + in_w]);
		}
		iter = iter + in_w;
	}
	iter = scale_lw;
	for (int l = scale_length; l < h_end; l++)
	{
		for (int k = scale_length; k < w_end; k++)
		{
			index_lk = iter + k;
			if (IData[index_lk] > 150)
			{
				OData[index_lk] = 0;
				continue;
			}
			if (HData[index_lk] > threshold_adj&&HData[index_lk] >= HData[index_lk - 1] && HData[index_lk] >= HData[index_lk + 1])
			{
				OData[index_lk] = 255;
				continue;
			}
			if (VData[index_lk] > threshold_adj&&VData[index_lk] >= VData[index_lk - in_w] && VData[index_lk] >= VData[index_lk + in_w])
			{
				OData[index_lk] = 255;
				continue;
			}
			OData[index_lk] = 0;
		}
		iter = iter + in_w;
	}
}
int MultiCoTarRecog::determ_code(int p11, int p12, int p21, int p22, int p31, int p32, int p41, int p42, int&peak)//����ÿ�������λֵȷ����־����ֵ
{
	int type1, type2, type3, type4;
	int code;
	int num1 = 0,//���ߵ�����
		num_1 = 0;//���ߵ�����
	if (p11 - p12 > 35)
	{
		type1 = -1;
		num_1 = num_1 + 1;
	}
	else if (p11 - p12 < -35)
	{
		type1 = 1;
		num1 = num1 + 1;
	}
	else
		type1 = 0;

	if (p21 - p22 > 35)
	{
		type2 = -1;
		num_1 = num_1 + 1;
	}
	else if (p21 - p22 < -35)
	{
		type2 = 1;
		num1 = num1 + 1;
	}
	else
		type2 = 0;

	if (p31 - p32 > 35)
	{
		type3 = -1;
		num_1 = num_1 + 1;
	}
	else if (p31 - p32 < -35)
	{
		type3 = 1;
		num1 = num1 + 1;
	}
	else
		type3 = 0;

	if (p41 - p42 > 35)
	{
		type4 = -1;
		num_1 = num_1 + 1;
	}
	else if (p41 - p42 < -35)
	{
		type4 = 1;
		num1 = num1 + 1;
	}
	else
		type4 = 0;
	code = num_1 * 3 + num1;
	if (code == 3 || code == 5 || code == 6)
		peak = determ_peak(type1, type2, type3, type4, -1);
	else if (code == 1 || code == 4 || code == 7)
		peak = determ_peak(type1, type2, type3, type4, 1);
	else
		code = -1;
	return code;
}
int MultiCoTarRecog::determ_peak(int type1, int type2, int type3, int type4, int search_value)//ȷ����ǵ㣬�����search_valueһ�µı��
{
	if (type1 == search_value)
		return 1;
	if (type2 == search_value)
		return 2;
	if (type3 == search_value)
		return 3;
	if (type4 == search_value)
		return 4;
}
void MultiCoTarRecog::start_from_peak(double* end_points, double* result_points, int peak)//���ݱ�ǵ��־�Զ�����д�������
{
	float angle_peak, angle[8];
	float max_angle, min_angle;
	float temp1, temp2;
	int sort_code;//��ǵ��ڽ��������е����
	int temp_code, temp_code1;

	angle[0] = LG->OriofTwoPoints(result_points[8], result_points[9], end_points[0], end_points[1]);
	angle[1] = 1;
	angle[2] = LG->OriofTwoPoints(result_points[8], result_points[9], end_points[2], end_points[3]);
	angle[3] = 2;
	angle[4] = LG->OriofTwoPoints(result_points[8], result_points[9], end_points[4], end_points[5]);
	angle[5] = 3;
	angle[6] = LG->OriofTwoPoints(result_points[8], result_points[9], end_points[6], end_points[7]);
	angle[7] = 4;
	for (int i = 0; i < 3; i = i + 1)
	{
		for (int l = 0; l < 6; l = l + 2)
		{
			if (angle[l] < angle[l + 2])
			{
				temp1 = angle[l + 2];
				temp2 = angle[l + 3];
				angle[l + 2] = angle[l];
				angle[l + 3] = angle[l + 1];
				angle[l] = temp1;
				angle[l + 1] = temp2;
			}
		}
	}
	for (int i = 0; i < 8; i = i + 2)
	{
		if (peak == angle[i + 1])
		{
			sort_code = i;
			break;
		}
	}
	for (int i = 0; i < 8; i = i + 2)
	{
		temp_code = sort_code % 8;
		temp_code1 = angle[temp_code + 1] * 2 - 2;
		result_points[i] = end_points[temp_code1];
		result_points[i + 1] = end_points[temp_code1 + 1];
		sort_code = sort_code + 2;
	}
}
