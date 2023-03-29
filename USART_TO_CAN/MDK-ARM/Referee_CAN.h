#ifndef REFEREE_CAN
#define REFEREE_CAN


#include "main.h"

#define Referee_CAN_ID 0x093

void Sentry_Referee_Data_Send(void);

typedef __PACKED_STRUCT
{
	uint8_t robot_id;
	uint8_t power_output;
	uint16_t remain_HP;
	uint16_t max_HP;
	
} robot_information_t;

void Get_Robot_Information(void);
void Referee_Send_Data(void);
void HERO_Referee_Data_Send(void);
void ENGINEER_Referee_Data_Send(void);
void Infantry_Referee_Data_Send(void);
void AERIAL_Referee_Data_Send(void);
void Sentry_Referee_Data_Send(void);



#endif