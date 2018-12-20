/*
 * drv8825.h
 *
 *  Created on: 2018Äê11ÔÂ4ÈÕ
 *      Author: wmy
 */

#ifndef DRV8825_H_
#define DRV8825_H_

#define MOTOR_PWM_CLOCK 2500000

extern void PWM0_GEN1_PB4PB5_Configure(void);
extern void PWM1_GEN1_PA6PA7_Configure(void);
extern void DRV8825_Init(void);
extern void DRV8825_Motor1_Control(double freq,char dir);
extern void DRV8825_Motor2_Control(double freq,char dir);
extern void Rotation_Angle_Motor1(double angle,char dir);
extern void Rotation_Angle_Motor2(double angle,char dir);
extern void Angle_Run(double angle,char dir);
extern void Opposite_Run(double angle,char dir);


#endif /* DRV8825_H_ */
