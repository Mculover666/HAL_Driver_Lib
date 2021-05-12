/**
 * @Copyright   (c) 2021,mculover666 All rights reserved
 * @filename    HC_SR04.c
 * @breif       Drive HC_SR04 based on stm32 tim peripheral
 * @changelog   v1.0    mculover666     2021/5/9
 *              v1.1    mculover666     2021/5/12   add object design
 */

#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#include "stm32f1xx.h"
#include "core_delay.h"

typedef struct hc_sr04_device_st {
    GPIO_TypeDef      *trig_port;
    uint16_t          trig_pin;
    GPIO_TypeDef      *echo_port;
    uint16_t          echo_pin;
    TIM_HandleTypeDef *tim;         //us级硬件定时器
    
    double            distance;     //测算距离
} hc_sr04_device_t;

/* us级延时函数 */
#define HC_SR04_Delay_Us(n)     CPU_TS_Tmr_Delay_US(n)

void HC_SR04_Init(hc_sr04_device_t *hc_sr04_device);
int HC_SR04_Measure(hc_sr04_device_t *hc_sr04_device);

#endif /* _HC_SR04_H_ */
