/**
 * @Copyright   (c) 2021,mculover666 All rights reserved
 * @filename    HC_SR04.c
 * @breif       Drive HC_SR04 based on stm32 tim peripheral
 * @changelog   v1.0    mculover666     2021/5/9
 *              v1.1    mculover666     2021/5/12   add object design
 */

#include "HC_SR04.h"
#include <stdio.h>

/**
 * @brief   pend the mutex to protect tim.
 * @param   none
 * @return  none
 * @note    it will be need if you use rtos
*/
static void HC_SRO4_Mutex_Pend()
{
    //add your code here
}

/**
 * @brief   post the mutex to protect tim.
 * @param   none
 * @return  none
 * @note    it will be need if you use rtos
*/
static void HC_SRO4_Mutex_Post()
{
    //add your code here
}

/**
 * @brief   hc_sr04_device object initialization.
 * @param   none
 * @return  none
*/
void HC_SR04_Init(hc_sr04_device_t *hc_sr04_device)
{
    // the gpio and tim is initialized in main
}

/**
 * @brief   Send trig signal.
 * @param   none
 * @return  none
*/
static void HC_SR04_Start(hc_sr04_device_t *hc_sr04_device)
{
    /* output high level */
    HAL_GPIO_WritePin(hc_sr04_device->trig_port, hc_sr04_device->trig_pin, GPIO_PIN_SET);
    
    /* maintain high level at least 10us */
    CPU_TS_Tmr_Delay_US(10);
    
    /* resume low level */
    HAL_GPIO_WritePin(hc_sr04_device->trig_port, hc_sr04_device->trig_pin, GPIO_PIN_RESET);
}

/**
 * @brief   Measure the high level time of the echo signal.
 * @param   hc_sr04_device  the pointer of the hc_sr04_device_t object
 * @return  errcode
 * @retval  0 success
 * @retval -1 fail
*/
int HC_SR04_Measure(hc_sr04_device_t *hc_sr04_device)
{
    uint32_t tick_us;
    
    HC_SRO4_Mutex_Pend();
    
    HC_SR04_Start(hc_sr04_device);
    
    __HAL_TIM_SetCounter(hc_sr04_device->tim, 0);
    
    /* waitting for start of the high level through echo pin */
    while (HAL_GPIO_ReadPin(hc_sr04_device->echo_port, hc_sr04_device->echo_pin) == GPIO_PIN_RESET);

    /* start the tim and enable the interrupt */
    HAL_TIM_Base_Start(hc_sr04_device->tim);
    
    /* waitting for end of the high level through echo pin */
    while (HAL_GPIO_ReadPin(hc_sr04_device->echo_port, hc_sr04_device->echo_pin) == GPIO_PIN_SET);
    
    /* stop the tim */
    HAL_TIM_Base_Stop(hc_sr04_device->tim);
    
    /* get the time of high level */
    tick_us = __HAL_TIM_GetCounter(hc_sr04_device->tim);
    
    /* calc distance in unit cm */
    hc_sr04_device->distance = (double)(tick_us/1000000.0) * 340.0 / 2.0 *100.0;
    
    HC_SRO4_Mutex_Post();
    
     return 0;
}
