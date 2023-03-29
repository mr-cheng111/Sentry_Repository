#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"



#define fp32 float

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
    
    int32_t series_ecd;
    fp32 angle;
    fp32 series_angle;
    int32_t rounds;
} motor_measure_t;

typedef struct
{
	uint8_t ID;
	uint8_t err;
	uint16_t pos;
	uint16_t speed;
	uint16_t T_M;
	uint8_t T_MOS;
	uint8_t T_Rotor;

}DM_RALE_DATA;





extern CAN_HandleTypeDef hcan;
extern void Can_Filter_Init(void);
void Can_Send_Data(CAN_HandleTypeDef *hcan,uint16_t frame,uint8_t Data_Length,uint8_t *data);


#endif