#ifndef FUZZYPID
#define FUZZYPID







void FuzzyPIDinit();  //������ʼ��
//ģ��PID����ʵ�ֺ���
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float error, float error_c,float ki_max,float ki_min,float kd_max,float kd_min,float error_pre,float error_ppre);
//����ӳ�亯��
float Quantization(float maximum,float minimum,float x);
//����e��de/dt�����ȼ��㺯��
void Get_grad_membership(float error,float error_c);
//��ȡ�������kp,ki,kd����������
void GetSumGrad();
//�����������kp,kd,ki��Ӧ����ֵ
void GetOUT();
//������ӳ�亯��
float Inverse_quantization(float maximum, float minimum, float qvalues);


#endif



