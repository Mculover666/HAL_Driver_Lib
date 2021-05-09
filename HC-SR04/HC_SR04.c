/**
 * @Copyright   (c) 2021,mculover666 All rights reserved
 * @filename    HC_SR04.c
 * @breif       Drive HC_SR04 based on stm32 tim peripheral
 * @changelog   v1.0    mculover666     2021/5/9
 */

#include "HC_SR04.h"
#include <stdio.h>

extern TIM_HandleTypeDef HC_SR04_TIM;
static uint32_t high_level_time_us;

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
    HC_SR04_Delay_Us(10);
    
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
    uint32_t timeout_start, timeout_cur;
    
    HC_SR04_Start();
    
    /* waitting for start of the high level through echo pin */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_RESET) {
        // timeout check code to avoid blocking
        timeout_cur = HAL_GetTick();
        if (timeout_cur - timeout_start > 100) {
            //100ms timeout
            return -1;
        }
    }

    /* start the tim and enable the interrupt */
    high_level_time_us = 0;
    HAL_TIM_Base_Start_IT(&HC_SR04_TIM);
    
    /* waitting for end of the high level through echo pin */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        // timeout check code to avoid blocking
        timeout_cur = HAL_GetTick();
        if (timeout_cur - timeout_start > 100) {
            //100ms timeout
            return -1;
        }
    }
    
    /* stop the tim */
    HAL_TIM_Base_Stop(&HC_SR04_TIM);
    
    /* calc distance in unit cm */
    *distance_cm = (double)(high_level_time_us/1000000.0) * 340.0 / 2.0 *100.0;
    
     return 0;
}

/**
 * @brief   Realize the callback function of TIM.
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance == HC_SR04_TIM.Instance)
    {
        high_level_time_us++;
    }
}
