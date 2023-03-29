#include "DebugThread.h"
#include "cmsis_os.h"
#include "RefereeCan.h"
#include "AttitudeThread.h"
#include "CalculateThread.h"
#include "CanPacket.h"
#include "usart.h"
#include "Usart_HMI.h"
uint16_t tx[3];
extern fp32 INS_angle[3];					 
extern Gimbal_t Gimbal;
extern RefereeInformation_t RefereeInformation;





uint16_t SetSpeedSet0;
char SetSpeedSet1[4];
uint16_t L_KP0;
char L_KP1[4];
uint16_t R_KP0;
char R_KP1[4];
uint16_t Speed_Ref0;
char Speed_Ref1[4];
void DebugThread(void const * pvParameters)
{
	osDelay(500);
	while(1)
	{
		SetSpeedSet0 =  SetSpeed;
		Speed_Ref0   = (int)100*RefereeInformation.Ammo2Speed;
		L_KP0        = (int)100*Lkp;
		R_KP0        = (int)100*Rkp;

		SetSpeedSet1[3] = SetSpeedSet0%10+48;
		SetSpeedSet0/=10;
		SetSpeedSet1[2] = SetSpeedSet0%10+48;
		SetSpeedSet0/=10;
		SetSpeedSet1[1] = SetSpeedSet0%10+48;
	  SetSpeedSet0/=10;
		SetSpeedSet1[0] = SetSpeedSet0+48;
		
		Speed_Ref1[3] = Speed_Ref0%10+48;
		Speed_Ref0/=10;
		Speed_Ref1[2] = Speed_Ref0%10+48;
		Speed_Ref0/=10;
		Speed_Ref1[1] = Speed_Ref0%10+48;
	  Speed_Ref0/=10;
		Speed_Ref1[0] = Speed_Ref0+48;
		
		L_KP1[3] = L_KP0%10+48;
		L_KP0/=10;
		L_KP1[2] = L_KP0%10+48;
		L_KP0/=10;
		L_KP1[1] = L_KP0%10+48;
	  L_KP0/=10;
		L_KP1[0] = L_KP0+48;
		
		R_KP1[3] = R_KP0%10+48;
		R_KP0/=10;
		R_KP1[2] = R_KP0%10+48;
		R_KP0/=10;
		R_KP1[1] = R_KP0%10+48;
	  R_KP0/=10;
		R_KP1[0] = R_KP0+48;
		
		char *setspeed = SetSpeedSet1;
		char *lkp = L_KP1;
		char *rkp = R_KP1;
		char *ref = Speed_Ref1;
		
		ReWriteHMI(lkp,rkp,setspeed,ref);
		osDelay(10);
	}
}


