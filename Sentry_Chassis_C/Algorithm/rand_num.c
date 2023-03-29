#include "rand_num.h"
#include "main.h"
#include "rng.h"
RNG_HandleTypeDef rng_Handle;
 
// RNG初始化
void RNG_Init(void)
{
 
	rng_Handle.Instance = RNG;
	HAL_RNG_Init(&rng_Handle);
	
 
}
// 获取随机数
uint32_t RNG_Get_RandomNum(void)
{
	return	HAL_RNG_GetRandomNumber(&rng_Handle);
}
 
// 获取指定范围内随机数
uint32_t RNG_Get_RandomRange(uint32_t min, uint32_t max)
{
	uint32_t Range = HAL_RNG_GetRandomNumber(&rng_Handle)%(max-min+1)+min;
	return Range;
}

