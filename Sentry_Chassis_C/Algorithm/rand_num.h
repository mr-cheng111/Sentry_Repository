#ifndef RAND_NUM
#define RAND_NUM

#include "main.h"



// RNG��ʼ��
void RNG_Init(void);
// ��ȡ�����
uint32_t RNG_Get_RandomNum(void);
// ��ȡָ����Χ�������
uint32_t RNG_Get_RandomRange(uint32_t min, uint32_t max);



#endif