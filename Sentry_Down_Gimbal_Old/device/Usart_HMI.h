#ifndef USARTHMI
#define USARTHMI

#include "usart.h"
#include "main.h"
#include "string.h"
#include "stdio.h"


extern float Lkp;
extern float Rkp;
extern uint16_t SetSpeed;


void Usart_HMI_init(void);
void Usart_HMI_Unpacked(uint8_t* Point);
uint8_t ReWriteHMI(char *L_KP,char *R_KP,char *SET_SPEED,char *FeedBack_SPEED);

																					 
																					 
																					 
																					 
#endif