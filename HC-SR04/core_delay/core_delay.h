#ifndef __CORE_DELAY_H
#define __CORE_DELAY_H

#include "stm32l4xx.h"

/* ��ȡ�ں�ʱ��Ƶ�� */
#define GET_CPU_ClkFreq()       HAL_RCC_GetSysClockFreq()
#define SysClockFreq            (80000000)

/* Ϊ����ʹ�ã�����ʱ�����ڲ�����CPU_TS_TmrInit������ʼ��ʱ����Ĵ�����
   ����ÿ�ε��ú��������ʼ��һ�顣
   �ѱ���ֵ����Ϊ0��Ȼ����main����������ʱ����CPU_TS_TmrInit�ɱ���ÿ�ζ���ʼ�� */  
#define CPU_TS_INIT_IN_DELAY_FUNCTION   1

/*******************************************************************************
 * ��������
 ******************************************************************************/
uint32_t CPU_TS_TmrRd(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

//ʹ�����º���ǰ�����ȵ���CPU_TS_TmrInit����ʹ�ܼ���������ʹ�ܺ�CPU_TS_INIT_IN_DELAY_FUNCTION
//�����ʱֵΪ8��
void CPU_TS_Tmr_Delay_US(uint32_t us);
#define HAL_Delay(ms)     CPU_TS_Tmr_Delay_US(ms*1000)
#define CPU_TS_Tmr_Delay_S(s)       CPU_TS_Tmr_Delay_MS(s*1000)


#endif /* __CORE_DELAY_H */

