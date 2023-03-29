#ifndef RAND_NUM
#define RAND_NUM

#include "main.h"



// RNG初始化
void RNG_Init(void);
// 获取随机数
uint32_t RNG_Get_RandomNum(void);
// 获取指定范围内随机数
uint32_t RNG_Get_RandomRange(uint32_t min, uint32_t max);



#endif