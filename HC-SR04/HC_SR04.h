/**
 * @Copyright   (c) 2021,mculover666 All rights reserved
 * @filename    HC_SR04.c
 * @breif       Drive HC_SR04 based on stm32 tim peripheral
 * @changelog   v1.0    mculover666     2021/5/9
 */

#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#include "stm32l4xx.h"
#include "core_delay/core_delay.h"

/* HC-SR04模块控制引脚 */
#define HC_SR04_TRIG_PORT       GPIOB
#define HC_SR04_TRIG_PIN        GPIO_PIN_8
#define HC_SR04_ECHO_PORT       GPIOB
#define HC_SR04_ECHO_PIN        GPIO_PIN_9

/* us级延时函数 */
#define HC_SR04_Delay_Us(n)     CPU_TS_Tmr_Delay_US(n)

/* us级硬件定时器 */
#define HC_SR04_TIM     htim2

void HC_SR04_Init(void);
int HC_SR04_Measure(double *distance_cm);

#endif /* _HC_SR04_H_ */
