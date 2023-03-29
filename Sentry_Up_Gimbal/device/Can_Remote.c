#include "Can_Remote.h"
#include "Remote.h"
#include "CanPacket.h"
#include "usart.h"
#include "RefereeCan.h"

CAN_RC_ctrl_t Can_RC;


void CAN_RC_PACK(CAN_RC_DATA_t* RC)
{
	uint8_t test_num;
	test_num = RC->rx_data0 + RC->rx_data1 + RC->rx_data2 + RC->rx_data3 + \
			   RC->rx_data4 + RC->rx_data5 + RC->rx_data6;

	if(test_num ==  RC->rx_data7)
	{
		Can_RC.ch[0] = 0;
		Can_RC.ch[1] = 0;
		Can_RC.ch[2] = RC->rx_data0<<8|RC->rx_data1;
		Can_RC.ch[3] = RC->rx_data2<<8|RC->rx_data3;
		Can_RC.ch[4] = RC->rx_data4<<8|RC->rx_data5;
		Can_RC.s[0]  = RC->rx_data6&0xF;
		Can_RC.s[1]  = RC->rx_data6>>4;
	}
  
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












