#include "LineGenerator.h"

LineGenerator::LineGenerator(int w,int h)
:MapWidth(w), MapHeight(h)
{
}
LineGenerator::LineGenerator()
{
}
LineGenerator::~LineGenerator(void)
{
	////////_CrtDumpMemoryLeaks();
}
int LineGenerator::TwoEnds(int u1,int v1,int u2,int v2,int* SavePosition)//以两个端点为输入，生成线段各像素点坐标，存入数组
{
	int Du,Dv,uH,vH,aDu,Su,ustart,udes,vstart,vdes,a,c,e,tt,w_h;
	int SaveHead=0;
	float k,k2,b,ki;
	
	if(v1<v2||(v1==v2&&u1<=u2))
	{
		vstart=v1;
		ustart=u1;
		vdes=v2;
		udes=u2;
	}
	if(v1>v2||(v1==v2&&u1>u2))
	{
		vstart=v2;
		ustart=u2;
		vdes=v1;
		udes=u1;
	}//起始点的纵坐标小或者在纵坐标相同的情况下，横坐标小
	Du=udes-ustart;
	Dv=vdes-vstart;
	aDu=abs(Du);
	if(Du==0)
		Su=0;
	else
		Su=Du/aDu;//横坐标步进符号
	uH=ustart;
	vH=vstart;
	if(aDu>=Dv)//横坐标间隔大，每步有横坐标步进驱动
	{
		for(int i=0;i<aDu+1;i++)
		{
			uH=uH+Su;
			a=abs((uH-ustart)*Dv-(vH-vstart)*Du);
			b=abs((uH-ustart)*Dv-(vH+1-vstart)*Du);
			if(a>b)//判断v是否步进
				vH=vH+1;
			SavePosition[SaveHead]=uH;
			SavePosition[SaveHead+1]=vH;
			SavePosition[SaveHead+2]=vH*MapWidth+uH;
			SaveHead=SaveHead+3;
		}
	}
	else
	{
		for(int i=0;i<Dv+1;i++)
		{
			vH=vH+1;
			a=abs((uH-ustart)*Dv-(vH-vstart)*Du);
			b=abs((uH+Su-ustart)*Dv-(vH-vstart)*Du);
			if(a>b)
				uH=uH+Su;
			SavePosition[SaveHead]=uH;
			SavePosition[SaveHead+1]=vH;
			SavePosition[SaveHead+2]=vH*MapWidth+uH;
			SaveHead=SaveHead+3;
		}
	}
	return SaveHead;
}

int LineGenerator::TwoEnds_Left2Right(int u1, int v1, int u2, int v2, int* SavePosition)
{
	int Du, Dv, uH, vH, aDu, Su, ustart, udes, vstart, vdes, a, c, e, tt, w_h;
	int SaveHead = 0;
	float k, k2, b, ki;

	if ( u1 <= u2)
	{
		vstart = v1;
		ustart = u1;
		vdes = v2;
		udes = u2;
	}
	if (u1 > u2)
	{
		vstart = v2;
		ustart = u2;
		vdes = v1;
		udes = u1;
	}//横坐标小的作为起始点
	Du = udes - ustart;
	Dv = vdes - vstart;
	aDu = abs(Du);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;//横坐标步进符号
	uH = ustart;
	vH = vstart;
	if (aDu >= Dv)//横坐标间隔大，每步由横坐标步进驱动
	{
		for (int i = 0; i < aDu + 1; i++)
		{
			uH = uH + Su;
			a = abs((uH - ustart)*Dv - (vH - vstart)*Du);
			b = abs((uH - ustart)*Dv - (vH + 1 - vstart)*Du);
			if (a > b)//判断v是否步进
				vH = vH + 1;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH * MapWidth + uH;
			SaveHead = SaveHead + 3;
		}
	}
	else
	{
		for (int i = 0; i < Dv + 1; i++)
		{
			vH = vH + 1;
			a = abs((uH - ustart)*Dv - (vH - vstart)*Du);
			b = abs((uH + Su - ustart)*Dv - (vH - vstart)*Du);
			if (a > b)
				uH = uH + Su;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH * MapWidth + uH;
			SaveHead = SaveHead + 3;
		}
	}
	return SaveHead;
}
int LineGenerator::TwoEnds_Left2Right_1C(int u1, int v1, int u2, int v2, int* SavePosition)
{
	int Du, Dv, uH, vH, aDu, Su, ustart, udes, vstart, vdes, a, c, e, tt, w_h;
	int SaveHead = 0;
	float k, k2, b, ki;

	if (u1 <= u2)
	{
		vstart = v1;
		ustart = u1;
		vdes = v2;
		udes = u2;
	}
	if (u1 > u2)
	{
		vstart = v2;
		ustart = u2;
		vdes = v1;
		udes = u1;
	}//横坐标小的作为起始点
	Du = udes - ustart;
	Dv = vdes - vstart;
	aDu = abs(Du);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;//横坐标步进符号
	uH = ustart;
	vH = vstart;
	if (aDu >= Dv)//横坐标间隔大，每步由横坐标步进驱动
	{
		for (int i = 0; i < aDu + 1; i++)
		{
			uH = uH + Su;
			a = abs((uH - ustart)*Dv - (vH - vstart)*Du);
			b = abs((uH - ustart)*Dv - (vH + 1 - vstart)*Du);
			if (a > b)//判断v是否步进
				vH = vH + 1;
			SavePosition[SaveHead] = vH * MapWidth + uH;
			SaveHead = SaveHead + 1;
		}
	}
	else
	{
		for (int i = 0; i < Dv + 1; i++)
		{
			vH = vH + 1;
			a = abs((uH - ustart)*Dv - (vH - vstart)*Du);
			b = abs((uH + Su - ustart)*Dv - (vH - vstart)*Du);
			if (a > b)
				uH = uH + Su;
			SavePosition[SaveHead] = vH * MapWidth + uH;
			SaveHead = SaveHead + 1;
		}
	}
	return SaveHead;
}


int LineGenerator::TwoEnds_Sym(int u1, int v1, int u2, int v2, int* SavePosition)//以两个端点为输入，以线段中点为起点，左右点交替存储，形成对称线段偏移量数组
{
	int Du, Dv, uH, vH, aDu, Su, aDv, Sv, a, b;
	int SaveHead = 0;
	int length = max(abs(u1-u2),abs(v1-v2));
	length = length / 2;
	Du = u1 - u2;
	Dv = v1 - v2;
	aDu = abs(Du);
	aDv = abs(Dv);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;//横坐标步进符号
	aDv = abs(Dv);
	if (Dv == 0)
		Sv = 0;
	else
		Sv = Dv / aDv;//纵坐标步进符号
	SavePosition[0] = 0;
	SavePosition[1] = 0;
	SavePosition[2] = 0;
	uH = 0;
	vH = 0;
	if (aDu >= aDv)//横坐标间隔大，每步有横坐标步进驱动
	{
		for (int i = 1; i<=length; i++)
		{
			uH = uH + Su;
			a = abs(uH*Dv - vH*Du);
			b = abs(uH*Dv - (vH + Sv)*Du);
			if (a>b)//判断v是否步进
				vH = vH + Sv;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			SavePosition[SaveHead + 3] = -SavePosition[SaveHead];
			SavePosition[SaveHead + 4] = -SavePosition[SaveHead + 1];
			SavePosition[SaveHead + 5] = -SavePosition[SaveHead + 2];
			SaveHead = SaveHead + 6;
		}
	}
	else
	{
		for (int i = 1; i<=length; i++)
		{
			vH = vH + Sv;
			a = abs(uH*Dv - vH*Du);
			b = abs((uH + Su)*Dv - vH*Du);
			if (a>b)
				uH = uH + Su;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			SavePosition[SaveHead + 3] = -SavePosition[SaveHead];
			SavePosition[SaveHead + 4] = -SavePosition[SaveHead + 1];
			SavePosition[SaveHead + 5] = -SavePosition[SaveHead + 2];
			SaveHead = SaveHead + 6;
		}
	}
	return SaveHead;
}
int LineGenerator::TwoEnds_Sym(float length,int u1, int v1, int u2, int v2,int* SavePosition)//以两个端点为输入，以线段中点为起点，左右点交替存储，形成对称线段偏移量数组
{
	int Du, Dv, uH, vH, aDu, Su, aDv, Sv, a, b;
	int SaveHead = 0;
	int L = floor(length / 2+0.5);
	Du = u1 - u2;
	Dv = v1 - v2;
	aDu = abs(Du);
	aDv = abs(Dv);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;//横坐标步进符号
	aDv = abs(Dv);
	if (Dv == 0)
		Sv = 0;
	else
		Sv = Dv / aDv;//纵坐标步进符号
	SavePosition[0] = 0;
	SavePosition[1] = 0;
	SavePosition[2] = 0;
	uH = 0;
	vH = 0;
	if (aDu >= aDv)//横坐标间隔大，每步有横坐标步进驱动
	{
		for (int i = 1; i <= L; i++)
		{
			uH = uH + Su;
			a = abs(uH*Dv - vH*Du);
			b = abs(uH*Dv - (vH + Sv)*Du);
			if (a>b)//判断v是否步进
				vH = vH + Sv;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			SavePosition[SaveHead + 3] = -SavePosition[SaveHead];
			SavePosition[SaveHead + 4] = -SavePosition[SaveHead + 1];
			SavePosition[SaveHead + 5] = -SavePosition[SaveHead + 2];
			SaveHead = SaveHead + 6;
		}
	}
	else
	{
		for (int i = 1; i <= L; i++)
		{
			vH = vH + Sv;
			a = abs(uH*Dv - vH*Du);
			b = abs((uH + Su)*Dv - vH*Du);
			if (a>b)
				uH = uH + Su;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			SavePosition[SaveHead + 3] = -SavePosition[SaveHead];
			SavePosition[SaveHead + 4] = -SavePosition[SaveHead + 1];
			SavePosition[SaveHead + 5] = -SavePosition[SaveHead + 2];
			SaveHead = SaveHead + 6;
		}
	}
	return SaveHead;
}
int LineGenerator::RelTwoEnds(int ustart,int vstart,int udes,int vdes,float length,int* SavePosition)//以两个端点为输入，生成从点1指向点2的线段的坐标偏移量，指定长度length
{
	int Du,Dv,uH,vH,aDu,Su,aDv,Sv,a,b;
	int SaveHead=0;
	Du=udes-ustart;
	Dv=vdes-vstart;
	aDu=abs(Du);
	aDv=abs(Dv);
	if(Du==0)
		Su=0;
	else
		Su=Du/aDu;//横坐标步进符号
	aDv=abs(Dv);
	if(Dv==0)
		Sv=0;
	else
		Sv=Dv/aDv;//纵坐标步进符号
	uH=0;
	vH=0;
	if(aDu>=aDv)//横坐标间隔大，每步有横坐标步进驱动
	{
		for(int i=0;i<length+1;i++)
		{
			uH=uH+Su;
			a=abs(uH*Dv-vH*Du);
			b=abs(uH*Dv-(vH+Sv)*Du);
			if(a>b)//判断v是否步进
				vH=vH+Sv;
			SavePosition[SaveHead]=uH;
			SavePosition[SaveHead+1]=vH;
			SavePosition[SaveHead+2]=vH*MapWidth+uH;
			SaveHead=SaveHead+3;
		}
	}
	else
	{
		for(int i=0;i<length+1;i++)
		{
			vH=vH+Sv;
			a=abs(uH*Dv-vH*Du);
			b=abs((uH+Su)*Dv-vH*Du);
			if(a>b)
				uH=uH+Su;
			SavePosition[SaveHead]=uH;
			SavePosition[SaveHead+1]=vH;
			SavePosition[SaveHead+2]=vH*MapWidth+uH;
			SaveHead=SaveHead+3;
		}
	}
	return SaveHead;
}
int LineGenerator::RelTwoEndsCenter(float length,int u1, int v1, int u2, int v2, int* SavePosition)//以两个端点为输入生成两侧型的线段，指定长度length
{
	int Du, Dv, uH, vH, aDu, Su, aDv, Sv, a, b;
	int SaveHead = 0;
	int L = floor(length / 2 + 0.5);
	int u_m, v_m;
	u_m = (u1 + u2) / 2;
	v_m = (v1 + v2) / 2;
	Du = u1 - u_m;
	Dv = v1 - v_m;
	aDu = abs(Du);
	aDv = abs(Dv);
	if (Du == 0)
		Su = 0;
	else
		Su = Du / aDu;//横坐标步进符号
	aDv = abs(Dv);
	if (Dv == 0)
		Sv = 0;
	else
		Sv = Dv / aDv;//纵坐标步进符号
	
	
	if (aDu >= aDv)//横坐标间隔大，每步有横坐标步进驱动
	{
		uH = -L;
		vH = floor(-Dv / (float)Du*L+0.5);
		for (int i = -L; i <= L; i++)
		{
			uH = uH + Su;
			a = abs(uH*Dv - vH*Du);
			b = abs(uH*Dv - (vH + Sv)*Du);
			if (a>b)//判断v是否步进
				vH = vH + Sv;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			
			SaveHead = SaveHead + 3;
		}
	}
	else
	{
		uH = floor(-Du / (float)Dv*L + 0.5);
		vH = -L;
		for (int i = 1; i <= L; i++)
		{
			vH = vH + Sv;
			a = abs(uH*Dv - vH*Du);
			b = abs((uH + Su)*Dv - vH*Du);
			if (a>b)
				uH = uH + Su;
			SavePosition[SaveHead] = uH;
			SavePosition[SaveHead + 1] = vH;
			SavePosition[SaveHead + 2] = vH*MapWidth + uH;
			
			SaveHead = SaveHead + 3;
		}
	}
	return SaveHead;
}
int LineGenerator::RayLength(int u,int v,float length,float angle,int* SavePosition)//从指定起点生成指定朝向和长度的射线各像素点坐标，存入数组
{
	int ue,ve;
	angle=angle*3.14159265358/180;
	ue=floor(u+length*cos(angle)+0.5);
	ve=floor(v+length*sin(angle)+0.5);
	return TwoEnds(u,v,ue,ve,SavePosition);
}
int LineGenerator::RayEnds(int us,int vs,int up,int vp,float length,int* SavePosition)//给定起点生成经过指定点的线段，存入数组
{
	int ue,ve;
	float P;
	P=length/sqrt(float((up-us)*(up-us)+(vp-vs)*(vp-vs)));
	ue=floor(us+P*(up-us)+0.5);
	ve=floor(vs+P*(vp-vs)+0.5);
	return TwoEnds(us,vs,ue,ve,SavePosition);
}
int LineGenerator::VerSegmentF(float length,float A,float B,int* SavePosition)//生成与给定直线垂直的线段坐标相对于起点的偏移量,给定直线方程
{
	int L=floor(length/2);
	int uf,vf,SaveHead=0;
	if(fabs(A)>1)
	{
		B=B/fabs(A);
		A=A/fabs(A);
	}
	if(fabs(B)>1)
	{
		A=A/fabs(B);
		B=B/fabs(B);
	}
	for(int i=-L;i<=L;i++)
	{
		uf=floor(A*i+0.5);
		vf=floor(B*i+0.5);

		SavePosition[SaveHead]=uf;
		SavePosition[SaveHead+1]=vf;
		SavePosition[SaveHead+2]=vf*MapWidth+uf;
		SaveHead=SaveHead+3;
	}
	return SaveHead;
}
int LineGenerator::VerSegment(float length,int u1,int v1,int u2,int v2,int* SavePosition)//生成与给定直线垂直的线段坐标相对于起点的偏移量,给定直线经过的两点
{
	return VerSegmentF(length,v2-v1,u1-u2,SavePosition);
}
int LineGenerator::VerSegmentF_Sym(float length, float A, float B, int* SavePosition)//生成与给定直线垂直的线段坐标,给定直线方程
{
	int L = floor(length / 2);
	int uf, vf, SaveHead = 0;
	if (fabs(A)>1)
	{
		B = B / fabs(A);
		A = A / fabs(A);
	}
	if (fabs(B)>1)
	{
		A = A / fabs(B);
		B = B / fabs(B);
	}
	SavePosition[0] = 0;
	SavePosition[1] = 0;
	SavePosition[2] = 0;
	for (int i = 1; i <= L; i++)
	{
		uf = floor(A*i + 0.5);
		vf = floor(B*i + 0.5);

		SavePosition[SaveHead] = uf;
		SavePosition[SaveHead + 1] = vf;
		SavePosition[SaveHead + 2] = vf*MapWidth + uf;
		SavePosition[SaveHead + 3] = -SavePosition[SaveHead];
		SavePosition[SaveHead + 4] = -SavePosition[SaveHead + 1];
		SavePosition[SaveHead + 5] = -SavePosition[SaveHead + 2];
		SaveHead = SaveHead + 6;
	}
	return SaveHead;
}
int LineGenerator::VerSegment_Sym(float length, int u1, int v1, int u2, int v2, int* SavePosition)//生成与给定直线垂直的线段坐标相对于起点的偏移量,给定直线经过的两点
{
	return VerSegmentF_Sym(length, v2 - v1, u1 - u2, SavePosition);
}
int LineGenerator::ParSegmentF(float length, float A, float B, int* SavePosition)//生成沿给定直线方向的线段偏移量坐标,给定直线方程（两侧型）
{
	int L = floor(length / 2);
	int uf, vf, SaveHead = 0;
	if (fabs(A) >= fabs(B))
	{
		B = B / fabs(A);
		A = A / fabs(A);
	}
	else
	{
		A = A / fabs(B);
		B = B / fabs(B);
	}
	for (int i = -L; i <= L; i++)
	{
		uf = floor(-B*i + 0.5);
		vf = floor(A*i + 0.5);

		SavePosition[SaveHead] = uf;
		SavePosition[SaveHead + 1] = vf;
		SavePosition[SaveHead + 2] = vf*MapWidth + uf;
		SaveHead = SaveHead + 3;
	}
	return SaveHead;
}
int LineGenerator::ParSegment(float length, int u1, int v1, int u2, int v2, int* SavePosition)//生成沿给定直线方向的线段偏移量坐标,给定直线方程（两侧型）
{
	return ParSegmentF(length, v2 - v1, u1 - u2, SavePosition);
}
void LineGenerator::IntersectTwoLine(double A1, double B1, double C1, double A2, double B2, double C2, double& u, double& v)
{
	u = (B1*C2 - B2*C1) / (A1*B2 - A2*B1);
	v = (A2*C1 - A1*C2) / (A1*B2 - A2*B1);
}
float LineGenerator::DisTwoPoints(float x1,float y1,float x2,float y2)//求两点间距离
{
	float t1,t2;
	t1=x1-x2;
	t2=y1-y2;
	return sqrt(t1*t1+t2*t2);
}
void LineGenerator::RayLineFunction(float A, float B, int length, int* SavePosition)//给定直线方程，生成射线
{
	float temp, Anew, Bnew;
	int i3;
	temp = max(fabs(A),fabs(B));
	Anew = A / temp;
	Bnew = B / temp;
	for (int i = 1; i < length; i++)
	{
		i3 = (i-1) * 3;
		SavePosition[i3] = floor(- i*Bnew+0.5);
		SavePosition[i3+1] = floor(i*Anew+0.5);
		SavePosition[i3 + 2] = SavePosition[i3 + 1] * MapWidth + SavePosition[i3];
	}
}
float LineGenerator::OriofTwoPoints(float xs, float ys, float xe, float ye)//求点s指向点e的射线的朝向角
{
	float dx = xe - xs;
	float dy = ye - ys;
	float angle;
	if (dx == 0)
	{
		if (dy >= 0)
			return 270;
		else
			return 90;
	}
	angle = 57.3*atan(fabs(dy/dx));
	if (dx >= 0 && dy >= 0)
		return 360 - angle;
	if (dx > 0 && dy <= 0)
		return angle;
	if (dx <= 0 && dy >= 0)
		return 180+ angle;
	if (dx < 0 && dy <= 0)
		return 180 - angle;
}
void LineGenerator::Pedal(double A, double B, double C, double x, double y, double& u, double& v)//计算垂足位置，输入直线方程和一个点坐标，计算点到直线的垂线和直线的交点
{
	double a, b, c;//垂线方程
	a = B;
	b = -A;
	c = -B*x + A*y;
	IntersectTwoLine(A, B, C, a, b, c, u,v);
}
void LineGenerator::Pedal(double x1, double y1, double x2, double y2, double x, double y, double& u, double& v)//重载，输入直线两点式
{
	double A,B,C;//垂线方程
	A = y2 - y1;
	B = x1 - x2;
	C = x1*(y1 - y2)+y1*(x2-x1);
	Pedal(A, B, C, x, y, u,v);
}
void LineGenerator::LineFunc(double&A, double&B, double&C, double angle, double u, double v)//输出直线方程，可重载，该函数输入角度和直线经过的一点,角度单位为度而非弧度
{
	float k;
	while (angle > 180 || angle < 0)
	{
		if (angle>180)
			angle = angle - 180;
		if (angle<0)
			angle = angle + 180;
	}
	if (angle<45 || angle>135)
	{
		k = -tan(angle*3.14159/180);
		A = k;
		B = -1;
		C = -k*u + v;
	}
	else
	{
		k = -((90-angle)*3.14159 / 180);
		A = 1;
		B = -k;
		C = -u + k*v;
	}
}
void LineGenerator::PreciseLine(double A, double B, double x, double y, unsigned char* IData, float length, float search_width, double* SavePosition)//根据粗略的直线方程在指定点附近确定精确的点，拟合直线
{
	PixelLineSegment* Para = new PixelLineSegment(MapWidth, MapHeight, length, A, B, ParaFunc);
	PixelLineSegment* Ver = new PixelLineSegment(MapWidth, MapHeight, search_width * 2, A, B, VerFunc);
	int base_left, base_right, gray_left_base, gray_right_base;
	int base_p = floor(y + 0.5)*MapWidth + floor(x+0.5);
	int base_search, iter, check_code,check_p;
	int sample_length_h = floor(length / 10 + 0.5);
	int sample_length_h3 = sample_length_h*3;
	int sample_length = sample_length_h * 2 + 1;
	int search_width_h = floor(search_width / 2 + 0.5);
	int plus_minus;
	base_left = base_p + Ver->Left(5).imp;
	base_right = base_p + Ver->Right(5).imp;
	gray_left_base = 0;
	gray_right_base = 0;
	iter = Para->CenterPosition - sample_length_h * 3;
	for (int k = -sample_length_h; k <= sample_length_h; k = k + 1)
	{
		gray_left_base = gray_left_base + IData[base_left + Para->data[iter]];
		gray_right_base = gray_right_base + IData[base_right + Para->data[iter]];
		iter = iter + 3;
	}
	gray_left_base = gray_left_base / sample_length;
	gray_right_base = gray_right_base / sample_length;

	int leftsample_p, rightsample_p, leftpixel, rightpixel;
	int* Differ = new int[int(search_width) * 2]();
	float* linepoint = new float[int(length) * 3]();
	int linehead = 0,point_p;
	int dif_center;
	int u, v;
	for (int k = 2; k < Para->LengthofArry; k = k + 3)
	{
		base_search = base_p + Para->data[k];
		plus_minus = -1;
		dif_center = DifferAlongLine(base_search, IData, Ver, 1, Differ);
		for (int l = 0; l <= search_width_h; l = l + 1)
		{
			check_code = (l + 1) / 2 * plus_minus;//对于0 1 2 3 4 的循环，得到0 1 -1 2 -2 3 -3这样的序列，实现中心向两侧扩展的模式
			plus_minus = -plus_minus;
			check_p = Ver->CenterPosition + check_code * 3 + 2;
			leftsample_p = Ver->data[check_p - sample_length_h3];

			u = (base_search + leftsample_p) % MapWidth;
			v = (base_search + leftsample_p) / MapWidth;

			if (abs(IData[base_search + leftsample_p] - gray_left_base) > 10)
				continue;
			rightsample_p = Ver->data[check_p + sample_length_h3];

			u = (base_search + rightsample_p) % MapWidth;
			v = (base_search + rightsample_p) / MapWidth;

			if (abs(IData[base_search + rightsample_p] - gray_right_base) > 10)
				continue;
			if (abs(Differ[dif_center + check_code]) > 10&&abs(Differ[dif_center + check_code]) >= abs(Differ[dif_center + check_code + 1]) && abs(Differ[dif_center + check_code]) >= abs(Differ[dif_center + check_code - 1]))
			{
				point_p = base_search + Ver->data[check_p];
				linepoint[linehead] = point_p%MapWidth;
				linepoint[linehead+1] = point_p/MapWidth;
				linehead = linehead + 2;
				break;
			}
		}
	}
	if (linehead < 4)
	{
		SavePosition[0] = 1;
		SavePosition[1] = 1;
		SavePosition[2] = 1;
	}
	FitLine(linepoint, linehead, SavePosition);
	
	delete[]linepoint;
	delete[]Differ;
	delete Para;
	delete Ver;
}
float LineGenerator::DisPointToLine(float A, float B, float C, float x, float y)//点到直线距离
{
	return fabs(x*A + y * B + C) / sqrt(A*A + B * B);
}
int LineGenerator::DifferAlongLine(int base_p, unsigned char* IData, PixelLineSegment* pls, int span, int* SavePosition)//沿着直线计算差分，参数为点的位置，图像，像素线段，差分跨度，返回线段中点差分值得存储位置
{
	span = span / 2;
	if (span == 0)
		span = 1;
	int end_p = pls->LengthInPixel - span;
	int span3 = span * 3;
	int iter_p = span3 + 2, iter_s = 0;
	int cent_p=pls->CenterCount-span;
	for (int i = span; i < end_p; i++)
	{
		SavePosition[iter_s] = IData[base_p + pls->data[iter_p - span3]] - IData[base_p + pls->data[iter_p + span3]];
		iter_p = iter_p + 3;
		iter_s = iter_s + 1;
	}
	return cent_p;
}
void LineGenerator::LineSegNearPoint(double A, double B, double C, float X, float Y, float length, float*SavePosition)//根据直线方程给出一点附近的线段
{
	double pedal_u, pedal_v;
	Pedal(A, B, C, X, Y, pedal_u, pedal_v);
	if (A == 0)
	{
		SavePosition[0] = pedal_u + length / 2;
		SavePosition[1] = pedal_v;
		SavePosition[2] = pedal_u - length / 2;
		SavePosition[3] = pedal_v;
		return;
	}
	if (B == 0)
	{
		SavePosition[0] = pedal_u;
		SavePosition[1] = pedal_v + length / 2;
		SavePosition[2] = pedal_u;
		SavePosition[3] = pedal_v - length / 2;
		return;
	}
	double a, b, c,t1,t2;
	double temp, A2, B2;
	A2 = A*A;
	B2 = B*B;
	a = 1 / A2 + 1 / B2;
	b = -2 * X / A + 2 * C / B2 + 2 * Y / B;
	c = X*X + Y*Y + (C*C) / B2 + 2 * C*Y / B - length*length / 4;

	temp = sqrt(b*b - 4 * a*c);
	t1 = (-b + temp) / 2 / a;
	t2 = (-b - temp) / 2 / a;
	SavePosition[0] = t1 / A;
	SavePosition[1] = -(t1 + C)/B;
	SavePosition[2] = t2 / A;
	SavePosition[3] = -(t2 + C) / B;
}
void LineGenerator::FitLine(float* L_Data, int DataLength, double* SaveP)//最小二乘直线拟合
{
	double sigmaX = 0, sigmaX2 = 0, sigmaY = 0, sigmaY2 = 0, sigmaXY = 0;
	double *X = new double[DataLength]();
	double *Y = new double[DataLength]();

	double x, y;
	double temp1, temp2, temp3, a1H, a0H, a1V, a0V, A1, A2, B1, B2, C1, C2, E1, E2, Q1, Q2;
	int it, ArryLength, head_D = 0;
	if (DataLength < 2)
		return;
	ArryLength = DataLength * 2;//DataLength是点的个数，而ArryLength是数组长度，一个点两位，所以是点数的两倍
	for (int i = 0; i < ArryLength; i = i + 2)
	{
		x = L_Data[i];
		y = L_Data[i + 1];
		sigmaX = sigmaX + x;
		sigmaX2 = sigmaX2 + x*x;
		sigmaY = sigmaY + y;
		sigmaY2 = sigmaY2 + y*y;
		sigmaXY = sigmaXY + x*y;
		X[head_D] = x;
		Y[head_D] = y;
		head_D = head_D + 1;
	}
	head_D = head_D - 1;

	temp1 = DataLength*sigmaX2;
	temp2 = temp1 - sigmaX*sigmaX;
	if (temp2 == 0)
	{
		A1 = 1;
		B1 = 0;
		C1 = -x;
	}
	else
	{
		temp3 = DataLength*sigmaXY - sigmaX*sigmaY;
		a1H = temp3 / temp2;
		a0H = sigmaY / DataLength - a1H*sigmaX / DataLength;
		A1 = a1H;
		B1 = -1;
		C1 = a0H;
	}


	temp1 = DataLength*sigmaY2;
	temp2 = temp1 - sigmaY*sigmaY;
	if (temp2 == 0)
	{
		A2 = 0;
		B2 = 1;
		C2 = -y;
	}
	else
	{
		temp3 = DataLength*sigmaXY - sigmaX*sigmaY;
		a1V = temp3 / temp2;
		a0V = sigmaX / DataLength - a1V*sigmaY / DataLength;
		A2 = -1;
		B2 = a1V;
		C2 = a0V;
	}
	E1 = 0;
	E2 = 0;
	Q1 = sqrt(A1*A1 + B1*B1);
	Q2 = sqrt(A2*A2 + B2*B2);
	for (int i = 0; i <= head_D; i++)
	{
		E1 = E1 + fabs(A1*X[i] + B1*Y[i] + C1) / Q1;
		E2 = E2 + fabs(A2*X[i] + B2*Y[i] + C2) / Q2;
	}
	if (E1 > E2)
	{
		SaveP[0] = A2;
		SaveP[1] = B2;
		SaveP[2] = C2;
	}
	else
	{
		SaveP[0] = A1;
		SaveP[1] = B1;
		SaveP[2] = C1;
	}
	delete[] X;
	delete[] Y;
}
void LineGenerator::CliffAlong(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, int* output)
{
	int border = searchlength / 2 + 3;
	searchlength = searchlength * 3;
	if (ref_u < border || ref_u >= input_width - border || ref_v < border || ref_v >= input_height - border)
		return;
	int dg1, dg2, dg3;
	int maxcliff = 20;
	searchlength = searchlength - 12;
	for (int k = 0; k < searchlength; k = k + 3)
	{
		dg1 = abs(input[ref_p + position_arry[k + 8]] - input[ref_p + position_arry[k + 2]]);
		dg2 = abs(input[ref_p + position_arry[k + 11]] - input[ref_p + position_arry[k + 5]]);//此时中间像素的位置k+6,k+7
		dg3 = abs(input[ref_p + position_arry[k + 14]] - input[ref_p + position_arry[k + 8]]);
		if (dg2>maxcliff && dg2 >= dg1&&dg2 >= dg3)
		{
			maxcliff = dg2;
			output[0] = ref_u + position_arry[k + 6];
			output[1] = ref_v + position_arry[k + 7];
			output[2] = ref_p + position_arry[k + 8];
		}
	}
}
bool LineGenerator::PreciseCliff(int ref_u, int ref_v, int ref_p, int searchlength, unsigned char* input, int* position_arry, int input_width, int input_height, float* output, char sign)
{
	float u1, v1, u2, v2, u3, v3, gray1, gray2, gray3;
	bool findcliff = false;
	int border = searchlength / 2 + 3;
	searchlength = searchlength * 3;
	if (ref_u < border || ref_u >= input_width - border || ref_v < border || ref_v >= input_height - border)
		return false;
	int dg1, dg2, dg3;
	int maxcliff = 20;
	searchlength = searchlength - 12;
	for (int k = 0; k < searchlength; k = k + 3)
	{
		if (sign == 'P')//跳变沿极性为正，前面的大于后面的
		{
			dg1 = input[ref_p + position_arry[k + 8]] - input[ref_p + position_arry[k + 2]];
			dg2 = input[ref_p + position_arry[k + 11]] - input[ref_p + position_arry[k + 5]];//此时中间像素的位置k+6,k+7
			dg3 = input[ref_p + position_arry[k + 14]] - input[ref_p + position_arry[k + 8]];
		}
		if (sign == 'N')//跳变沿极性为正，前面的大于后面的
		{
			dg1 = input[ref_p + position_arry[k + 2]]-input[ref_p + position_arry[k + 8]];
			dg2 = input[ref_p + position_arry[k + 5]]-input[ref_p + position_arry[k + 11]];//此时中间像素的位置k+6,k+7
			dg3 = input[ref_p + position_arry[k + 8]]-input[ref_p + position_arry[k + 14]];
		}
		if (sign == 'R')//跳变沿极性为正，前面的大于后面的
		{
			dg1 = abs(input[ref_p + position_arry[k + 8]] - input[ref_p + position_arry[k + 2]]);
			dg2 = abs(input[ref_p + position_arry[k + 11]] - input[ref_p + position_arry[k + 5]]);//此时中间像素的位置k+6,k+7
			dg3 = abs(input[ref_p + position_arry[k + 14]] - input[ref_p + position_arry[k + 8]]);
		}
		
		if (dg2>maxcliff && dg2 >= dg1&&dg2 >= dg3)
		{
			maxcliff = dg2;
			u1 = ref_u + position_arry[k + 3];
			v1 = ref_v + position_arry[k + 4];
			gray1 = input[ref_p + position_arry[k + 5]];
			u2 = ref_u + position_arry[k + 6];
			v2 = ref_v + position_arry[k + 7];
			gray2 = input[ref_p + position_arry[k + 8]];
			u3 = ref_u + position_arry[k + 9];
			v3 = ref_v + position_arry[k + 10];
			gray3 = input[ref_p + position_arry[k + 11]];
			findcliff = true;
		}
	}
	if (findcliff)
		SubPixelPosition(u1, v1, u2, v2, u3, v3, gray1, gray2, gray3, output);
	return true;
}
void LineGenerator::SubPixelPosition(float u1, float v1, float u2, float v2, float u3, float v3, float gray1, float gray2, float gray3, float* Result)
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
	Result[0] = ub1*ratio1 + ub2*ratio2;
	Result[1] = vb1*ratio1 + vb2*ratio2;
}


