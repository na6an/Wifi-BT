/*******************************************************************************
* File Name: pwm.h
* Version 1.0
*
* Description:
*  This file provides constants and parameter values for the pwm
*  component.
*
********************************************************************************
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(pwm_CY_TCPWM_PWM_PDL_H)
#define pwm_CY_TCPWM_PWM_PDL_H

#include "cyfitter.h"
#include "tcpwm/cy_tcpwm_pwm.h"

   
/*******************************************************************************
* Variables
*******************************************************************************/
/**
* \addtogroup group_globals
* @{
*/
extern uint8_t  pwm_initVar;
extern cy_stc_tcpwm_pwm_config_t const pwm_config;
/** @} group_globals */


/***************************************
*        Function Prototypes
****************************************/
/**
* \addtogroup group_general
* @{
*/
void pwm_Start(void);
__STATIC_INLINE cy_en_tcpwm_status_t pwm_Init(cy_stc_tcpwm_pwm_config_t const *config);
__STATIC_INLINE void pwm_DeInit(void);
__STATIC_INLINE void pwm_Enable(void);
__STATIC_INLINE void pwm_Disable(void);
__STATIC_INLINE uint32_t pwm_GetStatus(void);
__STATIC_INLINE void pwm_SetCompare0(uint32_t compare0);
__STATIC_INLINE uint32_t pwm_GetCompare0(void);
__STATIC_INLINE void pwm_SetCompare1(uint32_t compare1);
__STATIC_INLINE uint32_t pwm_GetCompare1(void);
__STATIC_INLINE void pwm_EnableCompareSwap(bool enable);
__STATIC_INLINE void pwm_SetCounter(uint32_t count);
__STATIC_INLINE uint32_t pwm_GetCounter(void);
__STATIC_INLINE void pwm_SetPeriod0(uint32_t period0);
__STATIC_INLINE uint32_t pwm_GetPeriod0(void);
__STATIC_INLINE void pwm_SetPeriod1(uint32_t period1);
__STATIC_INLINE uint32_t pwm_GetPeriod1(void);
__STATIC_INLINE void pwm_EnablePeriodSwap(bool enable);
__STATIC_INLINE void pwm_TriggerStart(void);
__STATIC_INLINE void pwm_TriggerReload(void);
__STATIC_INLINE void pwm_TriggerKill(void);
__STATIC_INLINE void pwm_TriggerSwap(void);
__STATIC_INLINE uint32_t pwm_GetInterruptStatus(void);
__STATIC_INLINE void pwm_ClearInterrupt(uint32_t source);
__STATIC_INLINE void pwm_SetInterrupt(uint32_t source);
__STATIC_INLINE void pwm_SetInterruptMask(uint32_t mask);
__STATIC_INLINE uint32_t pwm_GetInterruptMask(void);
__STATIC_INLINE uint32_t pwm_GetInterruptStatusMasked(void);
/** @} general */


/***************************************
*           API Constants
***************************************/

/**
* \addtogroup group_macros
* @{
*/
/** This is a ptr to the base address of the TCPWM instance */
#define pwm_HW                 (pwm_TCPWM__HW)

/** This is a ptr to the base address of the TCPWM CNT instance */
#define pwm_CNT_HW             (pwm_TCPWM__CNT_HW)

/** This is the counter instance number in the selected TCPWM */
#define pwm_CNT_NUM            (pwm_TCPWM__CNT_IDX)

/** This is the bit field representing the counter instance in the selected TCPWM */
#define pwm_CNT_MASK           (1UL << pwm_CNT_NUM)
/** @} group_macros */

#define pwm_INPUT_MODE_MASK    (0x3U)
#define pwm_INPUT_DISABLED     (7U)


/*******************************************************************************
* Function Name: pwm_Init
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_Init() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE cy_en_tcpwm_status_t pwm_Init(cy_stc_tcpwm_pwm_config_t const *config)
{
    return(Cy_TCPWM_PWM_Init(pwm_HW, pwm_CNT_NUM, config));
}


/*******************************************************************************
* Function Name: pwm_DeInit
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_DeInit() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_DeInit(void)                   
{
    Cy_TCPWM_PWM_DeInit(pwm_HW, pwm_CNT_NUM, &pwm_config);
}

/*******************************************************************************
* Function Name: pwm_Enable
****************************************************************************//**
*
* Invokes the Cy_TCPWM_Enable_Multiple() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_Enable(void)                   
{
    Cy_TCPWM_Enable_Multiple(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_Disable
****************************************************************************//**
*
* Invokes the Cy_TCPWM_Disable_Multiple() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_Disable(void)                  
{
    Cy_TCPWM_Disable_Multiple(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_GetStatus
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetStatus() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetStatus(void)                
{
    return(Cy_TCPWM_PWM_GetStatus(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_SetCompare0
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_SetCompare0() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetCompare0(uint32_t compare0)      
{
    Cy_TCPWM_PWM_SetCompare0(pwm_HW, pwm_CNT_NUM, compare0);
}


/*******************************************************************************
* Function Name: pwm_GetCompare0
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetCompare0() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetCompare0(void)              
{
    return(Cy_TCPWM_PWM_GetCompare0(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_SetCompare1
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_SetCompare1() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetCompare1(uint32_t compare1)      
{
    Cy_TCPWM_PWM_SetCompare1(pwm_HW, pwm_CNT_NUM, compare1);
}


/*******************************************************************************
* Function Name: pwm_GetCompare1
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetCompare1() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetCompare1(void)              
{
    return(Cy_TCPWM_PWM_GetCompare1(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_EnableCompareSwap
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_EnableCompareSwap() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_EnableCompareSwap(bool enable)  
{
    Cy_TCPWM_PWM_EnableCompareSwap(pwm_HW, pwm_CNT_NUM, enable);
}


/*******************************************************************************
* Function Name: pwm_SetCounter
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_SetCounter() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetCounter(uint32_t count)          
{
    Cy_TCPWM_PWM_SetCounter(pwm_HW, pwm_CNT_NUM, count);
}


/*******************************************************************************
* Function Name: pwm_GetCounter
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetCounter() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetCounter(void)               
{
    return(Cy_TCPWM_PWM_GetCounter(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_SetPeriod0
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_SetPeriod0() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetPeriod0(uint32_t period0)          
{
    Cy_TCPWM_PWM_SetPeriod0(pwm_HW, pwm_CNT_NUM, period0);
}


/*******************************************************************************
* Function Name: pwm_GetPeriod0
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetPeriod0() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetPeriod0(void)                
{
    return(Cy_TCPWM_PWM_GetPeriod0(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_SetPeriod1
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_SetPeriod1() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetPeriod1(uint32_t period1)
{
    Cy_TCPWM_PWM_SetPeriod1(pwm_HW, pwm_CNT_NUM, period1);
}


/*******************************************************************************
* Function Name: pwm_GetPeriod1
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_GetPeriod1() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetPeriod1(void)                
{
    return(Cy_TCPWM_PWM_GetPeriod1(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_EnablePeriodSwap
****************************************************************************//**
*
* Invokes the Cy_TCPWM_PWM_EnablePeriodSwap() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_EnablePeriodSwap(bool enable)
{
    Cy_TCPWM_PWM_EnablePeriodSwap(pwm_HW, pwm_CNT_NUM, enable);
}


/*******************************************************************************
* Function Name: pwm_TriggerStart
****************************************************************************//**
*
* Invokes the Cy_TCPWM_TriggerStart() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_TriggerStart(void)             
{
    Cy_TCPWM_TriggerStart(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_TriggerReload
****************************************************************************//**
*
* Invokes the Cy_TCPWM_TriggerReloadOrIndex() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_TriggerReload(void)     
{
    Cy_TCPWM_TriggerReloadOrIndex(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_TriggerKill
****************************************************************************//**
*
* Invokes the Cy_TCPWM_TriggerStopOrKill() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_TriggerKill(void)
{
    Cy_TCPWM_TriggerStopOrKill(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_TriggerSwap
****************************************************************************//**
*
* Invokes the Cy_TCPWM_TriggerCaptureOrSwap() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_TriggerSwap(void)     
{
    Cy_TCPWM_TriggerCaptureOrSwap(pwm_HW, pwm_CNT_MASK);
}


/*******************************************************************************
* Function Name: pwm_GetInterruptStatus
****************************************************************************//**
*
* Invokes the Cy_TCPWM_GetInterruptStatus() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetInterruptStatus(void)       
{
    return(Cy_TCPWM_GetInterruptStatus(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_ClearInterrupt
****************************************************************************//**
*
* Invokes the Cy_TCPWM_ClearInterrupt() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_ClearInterrupt(uint32_t source)     
{
    Cy_TCPWM_ClearInterrupt(pwm_HW, pwm_CNT_NUM, source);
}


/*******************************************************************************
* Function Name: pwm_SetInterrupt
****************************************************************************//**
*
* Invokes the Cy_TCPWM_SetInterrupt() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetInterrupt(uint32_t source)
{
    Cy_TCPWM_SetInterrupt(pwm_HW, pwm_CNT_NUM, source);
}


/*******************************************************************************
* Function Name: pwm_SetInterruptMask
****************************************************************************//**
*
* Invokes the Cy_TCPWM_SetInterruptMask() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE void pwm_SetInterruptMask(uint32_t mask)     
{
    Cy_TCPWM_SetInterruptMask(pwm_HW, pwm_CNT_NUM, mask);
}


/*******************************************************************************
* Function Name: pwm_GetInterruptMask
****************************************************************************//**
*
* Invokes the Cy_TCPWM_GetInterruptMask() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetInterruptMask(void)         
{
    return(Cy_TCPWM_GetInterruptMask(pwm_HW, pwm_CNT_NUM));
}


/*******************************************************************************
* Function Name: pwm_GetInterruptStatusMasked
****************************************************************************//**
*
* Invokes the Cy_TCPWM_GetInterruptStatusMasked() PDL driver function.
*
*******************************************************************************/
__STATIC_INLINE uint32_t pwm_GetInterruptStatusMasked(void)
{
    return(Cy_TCPWM_GetInterruptStatusMasked(pwm_HW, pwm_CNT_NUM));
}

#endif /* pwm_CY_TCPWM_PWM_PDL_H */


/* [] END OF FILE */
