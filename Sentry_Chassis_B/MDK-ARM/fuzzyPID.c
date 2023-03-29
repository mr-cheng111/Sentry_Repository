#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "fuzzyPID.h"
#define u8 unsigned char     		//8-bit:0-255
#define u16 unsigned int  			//16-bit:0-65535
#define u32 unsigned long int   //32-bit:0-4294967295

void FuzzyPIDinit();//��ʼ��PID����
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float error, float error_c, float ki_max, float ki_min,float kd_max, float kd_min,float error_pre, float error_ppre);  //ģ��PID����ʵ�ֺ���
float Quantization(float maximum, float minimum, float x);	//��� error �����仯 error_c ӳ�䵽�����еĺ���
void Get_grad_membership(float error, float error_c);	//��������e��de/dt������
void GetSumGrad();// ��ȡ������� ��kp����ki����kd ����������
void GetOUT();    // ����������� ��kp����ki����kd ��Ӧ����ֵ
float Inverse_quantization(float maximum, float minimum, float qvalues);  //ȥģ����


const int  num_area = 8; //�����������
float e_membership_values[7]  = {-3,-2,-1,0,1,2,3};//����e������ֵ
float ec_membership_values[7] = {-3,-2,-1,0,1,2,3};//����de/dt������ֵ
float kp_menbership_values[7] = {-3,-2,-1,0,1,2,3};//�������kp������ֵ
float ki_menbership_values[7] = {-3,-2,-1,0,1,2,3};//�������ki������ֵ
float kd_menbership_values[7] = {-3,-2,-1,0,1,2,3};//�������kd������ֵ

float kp;                       //PID����kp
float ki;                       //PID����ki
float kd;                       //PID����kd
float qdetail_kp;               //����kp��Ӧ�����е�ֵ
float qdetail_ki;               //����ki��Ӧ�����е�ֵ
float qdetail_kd;               //����kd��Ӧ�����е�ֵ
float detail_kp;                //�������kp
float detail_ki;                //�������ki
float detail_kd;                //�������kd
float qerror;                   //����e��Ӧ�����е�ֵ
float qerror_c;                 //����de/dt��Ӧ�����е�ֵ             
float e_gradmembership[2];      //����e��������
float ec_gradmembership[2];     //����de/dt��������
int e_grad_index[2];            //����e�������ڹ���������
int ec_grad_index[2];           //����de/dt�������ڹ���������
float KpgradSums[7] = { 0,0,0,0,0,0,0 };   //�������kp�ܵ�������
float KigradSums[7] = { 0,0,0,0,0,0,0 };   //�������ki�ܵ�������
float KdgradSums[7] = { 0,0,0,0,0,0,0 };   //�������kd�ܵ�������

float e_max  =  150;    //������ֵ
float e_min  = -150;    //�����Сֵ
float ec_max =  300;    //���仯���ֵ
float ec_min = -300;    //���仯��Сֵ
float kp_max =  50;     //����ϵ�� kp ����ֵ
float kp_min = -50;    	//����ϵ�� kp ����ֵ
float ki_max =  0.1;    //����ϵ�� ki ����ֵ
float ki_min = -0.1;    //����ϵ�� ki ����ֵ
float kd_max =  0.01;   //΢��ϵ�� kd ����ֵ
float kd_min = -0.01;   //΢��ϵ�� kd ����ֵ
float error;        		//���ֵ
float error_c;      		//���仯ֵ
float error_pre = 0;    //��һ�����ֵ
float error_ppre = 0;   //���ϴ����ֵ

int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //��������ֵ


void FuzzyPIDinit()  //������ʼ��
{
	kp = 0;
	ki = 0;
	kd = 0;
	qdetail_kp = 0;
	qdetail_ki = 0;
	qdetail_kd = 0;
}

//ģ��PID����ʵ�ֺ���
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float error, float error_c,float ki_max,float ki_min,float kd_max,float kd_min,float error_pre,float error_ppre)
{
	qerror = Quantization(e_max, e_min, error);	   			//����� error ӳ�䵽������
	qerror_c = Quantization(ec_max, ec_min, error_c);	  //�����仯 error_c ӳ�䵽������
	Get_grad_membership(qerror, qerror_c);							//������� error �����仯 error_c��������
	GetSumGrad();																				//����������� ��kp����ki����kd ����������
	GetOUT();																						// ����������� ��kp����ki����kd ��Ӧ����ֵ
	detail_kp = Inverse_quantization(kp_max, kp_min, qdetail_kp);    //ȥģ�����õ����� ��kp
	detail_ki = Inverse_quantization(ki_max, ki_min, qdetail_ki);    //ȥģ�����õ����� ��ki
	detail_kd = Inverse_quantization(kd_max, kd_min, qdetail_kd);    //ȥģ�����õ����� ��kd
	qdetail_kd = 0;
	qdetail_ki = 0;
	qdetail_kp = 0;

	kp = kp + detail_kp;    //�õ����յ� kp ֵ
	ki = ki + detail_ki;    //�õ����յ� ki ֵ
	kd = kd + detail_kd;    //�õ����յ� kd ֵ
	if (kp < 0)
		kp = 0;
	if (ki < 0)
		ki = 0;
	if (kd < 0)
		kd = 0;
	detail_kp = 0;
   	detail_ki = 0;
    	detail_kd = 0;
	float output = kp*(error - error_pre) + ki * error + kd * (error - 2 * error_pre + error_ppre);    //�������յ����
	return output;
}

 
///����ӳ�亯��
float Quantization(float maximum,float minimum,float x)
{
	float qvalues= 6.0 *(x-minimum)/(maximum - minimum)-3;
	return qvalues;
}
 
//����e��de/dt�����ȼ��㺯��
void Get_grad_membership(float error,float error_c)   
{
	if (error > e_membership_values[0] && error < e_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (error >= e_membership_values[i] && error <= e_membership_values[i + 1])
			{
				e_gradmembership[0] = -(error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_gradmembership[1] = 1+(error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_grad_index[0] = i;
				e_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (error <= e_membership_values[0])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 0;
			e_grad_index[1] = -1;
		}
		else if (error >= e_membership_values[6])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 6;
			e_grad_index[1] = -1;
		}
	}
 
	if (error_c > ec_membership_values[0] && error_c < ec_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (error_c >= ec_membership_values[i] && error_c <= ec_membership_values[i + 1])
			{
				ec_gradmembership[0] = -(error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_gradmembership[1] = 1 + (error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_grad_index[0] = i;
				ec_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (error_c <= ec_membership_values[0])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 0;
			ec_grad_index[1] = -1;
		}
		else if (error_c >= ec_membership_values[6])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 6;
			ec_grad_index[1] = -1;
		}
	}
 
}
 
// ��ȡ�������kp,ki,kd����������
void GetSumGrad()
{
	int  Kp_rule_list[7][7] = {
														 {PB,PB,PM,PM,PS,ZO,ZO},        //kp�����
														 {PB,PB,PM,PS,PS,ZO,NS},
														 {PM,PM,PM,PS,ZO,NS,NS},
														 {PM,PM,PS,ZO,NS,NM,NM},
														 {PS,PS,ZO,NS,NS,NM,NM},
														 {PS,ZO,NS,NM,NM,NM,NB},
														 {ZO,ZO,NM,NM,NM,NB,NB} 
																										};

	int  Ki_rule_list[7][7] = { 
															{NB,NB,NM,NM,NS,ZO,ZO},     //ki�����
															{NB,NB,NM,NS,NS,ZO,ZO},
															{NB,NM,NS,NS,ZO,PS,PS},
															{NM,NM,NS,ZO,PS,PM,PM},
															{NM,NS,ZO,PS,PS,PM,PB},
															{ZO,ZO,PS,PS,PM,PB,PB},
															{ZO,ZO,PS,PM,PM,PB,PB} 
																										};

	int  Kd_rule_list[7][7] = {
															{PS,NS,NB,NB,NB,NM,PS},     //kd�����
														  {PS,NS,NB,NM,NM,NS,ZO},
														  {ZO,NS,NM,NM,NS,NS,ZO},
														  {ZO,NS,NS,NS,NS,NS,ZO},
														  {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
															{PB,NS,PS,PS,PS,PS,PB},
															{PB,PM,PM,PM,PS,PS,PB} 
																										};

    // ��ʼ�� Kp��Ki��Kd �ܵ�������ֵΪ 0
	for (int i = 0; i <= num_area - 1; i++)
	{
		KpgradSums[i] = 0;
		KigradSums[i] = 0;
        		KdgradSums[i] = 0;
	}

    	for (int i = 0; i < 2; i++)
   	 {
        		if (e_grad_index[i] == -1)
        		{
            			continue;
       		}
        		for (int j = 0; j < 2; j++)
       		{
            			if (ec_grad_index[j] != -1)
            			{
                			int indexKp = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
               				int indexKi = Ki_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
                			int indexKd = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
                			KpgradSums[indexKp]= KpgradSums[indexKp] + (e_gradmembership[i] * ec_gradmembership[j]);
               				KigradSums[indexKi] = KigradSums[indexKi] + (e_gradmembership[i] * ec_gradmembership[j]);
                			KdgradSums[indexKd] = KdgradSums[indexKd] + (e_gradmembership[i] * ec_gradmembership[j]);
            			}
            			else
            			{
                			continue;
            			}
        		}
 	 }
}
 
// �����������kp,kd,ki��Ӧ����ֵ
void GetOUT()
{
	for (int i = 0; i < num_area - 1; i++)
	{
		qdetail_kp += kp_menbership_values[i] * KpgradSums[i];
		qdetail_ki += ki_menbership_values[i] * KigradSums[i];
		qdetail_kd += kd_menbership_values[i] * KdgradSums[i];
	}
}
 
//������ӳ�亯��
float Inverse_quantization(float maximum, float minimum, float qvalues)
{
	float x = (maximum - minimum) *(qvalues + 3)/6 + minimum;
	return x;
}