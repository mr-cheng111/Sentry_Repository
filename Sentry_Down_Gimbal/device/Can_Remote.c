#include "Can_Remote.h"
#include "Remote.h"
#include "CanPacket.h"
#include "usart.h"
#include "CalculateThread.h"

CAN_RC_ctrl_t Can_RC;
extern Gimbal_t Gimbal;
void CAN_RC_PACK(CAN_RC_DATA_t* RC)
{
	uint8_t test_num;
	Can_RC.ch[0] = 0;
	Can_RC.ch[1] = 0;
	Can_RC.ch[2] = RC->rx_data0<<8|RC->rx_data1;
	Can_RC.ch[3] = RC->rx_data2<<8|RC->rx_data3;
	Can_RC.ch[4] = RC->rx_data4<<8|RC->rx_data5;
	Can_RC.s[0]  = RC->rx_data6&0xF;
	Can_RC.s[1]  = RC->rx_data6>>4;
	
	switch(RC->rx_data7&1)
	{
		case 1:Game_State.Game_Start_State = Game_START;break;
		case 0:Game_State.Game_Start_State = Game_STOP; break;
		default: Game_State.Game_Start_State = Game_STOP;break;
	}
	switch((RC->rx_data7&(1<<1))>>1)
	{
		case 1:  Game_State.Defend_Mode = Sentry_Defend;break;
		case 0:  Game_State.Defend_Mode = Sentry_Escape;break;
		default: Game_State.Defend_Mode = Sentry_Defend;break;
	}
  switch((RC->rx_data7&(1<<2))>>2)
	{
		case 1:  Game_State.Point_Fire_Mode = Sentry_Point_Fire;break;
		case 0:  Game_State.Point_Fire_Mode = Sentry_Brust_Fire;break;
		default: Game_State.Point_Fire_Mode = Sentry_Brust_Fire;break;
	}
	
  switch((RC->rx_data7&(1<<3))>>3)
  {	
	  case 0:  break;
	  case 1:  break;
	  default :break;
  }
  
  //µ×ÅÌÔË¶¯×´Ì¬
  Gimbal.Chassis_Mode = RC->rx_data7&((1<<4)|(1<<5)|(1<<6)|(1<<7))>>4;

	
}

void CAN_RC_TO_RC(RC_ctrl_t* remote,CAN_RC_ctrl_t* can_data)
{
	remote->rc.ch[0] = can_data->ch[0];
	remote->rc.ch[1] = can_data->ch[1];
	remote->rc.ch[2] = can_data->ch[2];
	remote->rc.ch[3] = can_data->ch[3];
	remote->rc.ch[4] = can_data->ch[4];
	
	remote->rc.s[0]	 = can_data->s[0];
	remote->rc.s[1]	 = can_data->s[1];
}













