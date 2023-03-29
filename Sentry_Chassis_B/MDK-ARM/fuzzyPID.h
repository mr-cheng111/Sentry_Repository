#ifndef FUZZYPID
#define FUZZYPID







void FuzzyPIDinit();  //参数初始化
//模糊PID控制实现函数
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float error, float error_c,float ki_max,float ki_min,float kd_max,float kd_min,float error_pre,float error_ppre);
//区间映射函数
float Quantization(float maximum,float minimum,float x);
//输入e与de/dt隶属度计算函数
void Get_grad_membership(float error,float error_c);
//获取输出增量kp,ki,kd的总隶属度
void GetSumGrad();
//计算输出增量kp,kd,ki对应论域值
void GetOUT();
//反区间映射函数
float Inverse_quantization(float maximum, float minimum, float qvalues);


#endif



