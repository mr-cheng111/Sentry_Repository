#include "imu.h"
#include "usart.h"




//陀螺仪反馈值
imu_t imu;
//陀螺仪数据接收区
uint8_t imu_rcv_buf[100];



void imu_init(void)
{
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, imu_rcv_buf, IMU_BUFF_SIZE);
}

void imu_rate_process(uint8_t i,uint8_t *rcv)
{
    imu.rate.row    =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 3]<<8)|rcv[i*IMU_PACK_LENGTH + 2]) ) /32768*2000;
    imu.rate.pitch  =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 5]<<8)|rcv[i*IMU_PACK_LENGTH + 4]) ) /32768*2000;
    imu.rate.yaw    =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 7]<<8)|rcv[i*IMU_PACK_LENGTH + 6]) ) /32768*2000;
}

void imu_angle_process(uint8_t i,uint8_t *rcv)
{
    imu.angle.row   =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 3]<<8)|rcv[i*IMU_PACK_LENGTH + 2]) ) /32768*180;
    imu.angle.pitch =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 5]<<8)|rcv[i*IMU_PACK_LENGTH + 4]) ) /32768*180;
    imu.angle.yaw   =   (fp32) ((int16_t) ((rcv[i*IMU_PACK_LENGTH + 7]<<8)|rcv[i*IMU_PACK_LENGTH + 6]) ) /32768*180;
}























