/*
 * drv8825.c
 *
 *  Created on: 2018Äê7ÔÂ8ÈÕ
 *      Author: wmy
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "time.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "string.h"
#include "driverlib/timer.h"

#define MOTOR_PWM_CLOCK 2500000

void PWM0_GEN1_PB4PB5_Configure(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 4000);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT|PWM_OUT_3_BIT,true);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_1)*50/100);//number=Numb*%x B4
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_1)*50/100);//number=Numb*%x B5
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void PWM1_GEN1_PA6PA7_Configure(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 4000);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMOutputState(PWM1_BASE,PWM_OUT_2_BIT|PWM_OUT_3_BIT,true);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM1_BASE,PWM_GEN_1)*50/100);//number=Numb*%x A6
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM1_BASE,PWM_GEN_1)*50/100);//number=Numb*%x A7
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void DRV8825_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    PWM0_GEN1_PB4PB5_Configure();
    PWM1_GEN1_PA6PA7_Configure();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_0|GPIO_PIN_1 );
    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, GPIO_PIN_2 );
    SysCtlDelay((SysCtlClockGet()/3000)*50);
    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, 0X0);
}

void DRV8825_Motor1_Control(double freq,char dir)
{
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, GPIO_PIN_2);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, 0x0);
    }
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, MOTOR_PWM_CLOCK/(freq*1.0));//Numb=system clock/PWM frequency
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_1)*50/100);//number=Numb*%x
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_1)*50/100);//number=Numb*%x
}

void DRV8825_Motor2_Control(double freq,char dir)
{
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_1, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, 0x0);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, GPIO_PIN_3);
    }
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, MOTOR_PWM_CLOCK/(freq*1.0));//Numb=system clock/PWM frequency
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM1_BASE,PWM_GEN_1)*50/100);//number=Numb*%x
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM1_BASE,PWM_GEN_1)*50/100);//number=Numb*%x
}

void Rotation_Angle_Motor1(double angle,char dir)
{
    int i;
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, GPIO_PIN_2);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, 0x0);
    }
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
    for(i=1;i<=(int)(angle/1.8);i++)
    {
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, 0X0);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
    }
}

void Rotation_Angle_Motor2(double angle,char dir)
{
    int i;
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_1, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, 0x0);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, GPIO_PIN_3);
    }
    PWMGenDisable(PWM1_BASE, PWM_GEN_1);
    for(i=1;i<=(int)(angle/1.8);i++)
    {
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, GPIO_PIN_6);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, 0X0);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
    }
}

void Angle_Run(double angle,char dir)
{
    int i;
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_1, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, 0x0);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, GPIO_PIN_3);
    }
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
    PWMGenDisable(PWM1_BASE, PWM_GEN_1);
    for(i=1;i<=(int)(angle/1.8);i++)
    {
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, GPIO_PIN_6);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, 0X0);
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, 0X0);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
    }
}

void Opposite_Run(double angle,char dir)
{
    int i;
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_1, 0x0);
    if(dir=='+')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, GPIO_PIN_3);
    }
    else if(dir=='-')
    {
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_3, 0x0);
    }
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
    PWMGenDisable(PWM1_BASE, PWM_GEN_1);
    for(i=1;i<=(int)(angle/1.8);i++)
    {
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, GPIO_PIN_6);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
        GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_4, 0X0);
        GPIOPinWrite(GPIO_PORTA_BASE,  GPIO_PIN_6, 0X0);
        SysCtlDelay((SysCtlClockGet()/3000)*1);
    }
}

