#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "Chassis_Task.h"
#include "main.h"

extern float SupKp;
extern fp32 total_current_limit;
extern fp32 total_current;
extern uint8_t CalculCount;
void ChassisPowerInit();
void CapCharge(float Percentage);
void ChassisReduceRate();
fp32 chassis_powerloop(Chassis_Typedef *Chassis);
void ChassisPowerControl(Chassis_Typedef *Chassis);

#endif
