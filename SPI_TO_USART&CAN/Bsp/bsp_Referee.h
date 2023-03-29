#ifndef BSP_REFEREE_H
#define BSP_REFEREE_H

#include "main.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,//����״̬����
    GAME_RESULT_CMD_ID                = 0x0002,//�����������
    GAME_ROBOT_HP_CMD_ID              = 0x0003,//����������Ѫ������
    FIELD_EVENTS_CMD_ID               = 0x0101,//�����¼�����
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,//���ز���վ������ʶ����
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,//���󲹸�վ�������ݣ�RM�Կ�����δ���ţ�
    REFEREE_WARNING_CMD_ID            = 0x0104,//���о�������
	DART_FIRE_COUNTDOWN_CMD_ID     	  = 0x0105,//���ڷ���ڵ���ʱ
    ROBOT_STATE_CMD_ID                = 0x0201,//������״̬����
    POWER_HEAT_DATA_CMD_ID            = 0x0202,//ʵʱ������������
    ROBOT_POS_CMD_ID                  = 0x0203,//������λ������
    BUFF_MUSK_CMD_ID                  = 0x0204,//��������������
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,//���л���������״̬����
    ROBOT_HURT_CMD_ID                 = 0x0206,//�˺�״̬����
    SHOOT_DATA_CMD_ID                 = 0x0207,//ʵʱ�������
	BULLET_REMAINING_CMD_ID           = 0x0208,//�ӵ�ʣ�෢����
	ROBOT_RFID_STATUS_ID			  = 0x0209,//������ RFID ״̬
	DART_CLIENT_CMD_ID           	  = 0x020A,//���ڿͻ����˻���ָ����
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,//�����˼佻������
	CUSTOM_CONTRROLL_INTERACTION_DATA =	0x0302,//�Զ���������������ݽӿ�
	CLIENT_SIDE_MAP_INTERACTION		  =	0x0303,//�ͻ���С��ͼ��������
	KEYBOARD_MOUSE_INFORMATION		  = 0x0304,//���̡������Ϣ
	CLIENT_SIDE_MAP_RC_INFORMATION	  = 0x0305,//�ͻ���С��ͼ������Ϣ
    IDCustomData,
}referee_cmd_id_t;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
	Game_Start = 1,
	Game_Stop,
	
}Referee_Pack_t;

#define Referee_Buffer_Length 512
#define REFEREE_FIFO_BUF_LENGTH 1024
extern uint8_t Referee_Buffer[2][Referee_Buffer_Length];
extern uint8_t Buffer_Choose;
extern uint16_t RX_Length[2];
void usart1_init(uint8_t *Dst, uint32_t Buffer_Length);
void USART_Data_Slove(void);
#endif
