/**
 * @Copyright   (c) 2021,mculover666 All rights reserved
 * @filename    HC_SR04.c
 * @breif       Drive HC_SR04 based on stm32 tim peripheral
 * @changelog   v1.0    mculover666     2021/5/9
 */

#include "HC_SR04.h"
#include <stdio.h>

extern TIM_HandleTypeDef HC_SR04_TIM;

/**
 * @brief   GPIO and TIM initialization.
 * @param   none
 * @return  none
*/
void HC_SR04_Init(void)
{
    // it is initialized in main function
}

/**
 * @brief   Send trig signal.
 * @param   none
 * @return  none
*/
static void HC_SR04_Start(void)
{
    /* output high level */
    HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_SET);
    
    /* maintain high level at least 10us */
    CPU_TS_Tmr_Delay_US(10);
    
    /* resume low level */
    HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief   Measure the high level time of the echo signal.
 * @param   none
 * @return  errcode
 * @retval  0 success
 * @retval -1 fail
*/
int HC_SR04_Measure(double *distance_cm)
{
    uint32_t tick_us;
    
    HC_SR04_Start();
    
    __HAL_TIM_SetCounter(&HC_SR04_TIM, 0);
    
    /* waitting for start of the high level through echo pin */
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_RESET);

    /* start the tim and enable the interrupt */
    HAL_TIM_Base_Start(&HC_SR04_TIM);
    
    /* waitting for end of the high level through echo pin */
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_SET);
    
    /* stop the tim */
    HAL_TIM_Base_Stop(&HC_SR04_TIM);
    
    /* get the time of high level */
    tick_us = __HAL_TIM_GetCounter(&HC_SR04_TIM);
    
    /* calc distance in unit cm */
    *distance_cm = (double)(tick_us/1000000.0) * 340.0 / 2.0 *100.0;
    
     return 0;
}
