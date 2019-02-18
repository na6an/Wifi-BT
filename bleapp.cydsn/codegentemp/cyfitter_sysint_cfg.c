/*******************************************************************************
* File Name: cyfitter_sysint_cfg.c
* 
* PSoC Creator  4.2
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2018 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "cyfitter_sysint.h"
#include "cyfitter_sysint_cfg.h"

/* ARM CM4 */
#if (((__CORTEX_M == 4) && (CY_CORE_ID == 0)))

    /* ble_bless_isr */
    const cy_stc_sysint_t ble_bless_isr_cfg = {
        .intrSrc = (IRQn_Type)ble_bless_isr__INTC_NUMBER,
        .intrPriority = ble_bless_isr__INTC_CORTEXM4_PRIORITY
    };

    /* capsense_ISR */
    const cy_stc_sysint_t capsense_ISR_cfg = {
        .intrSrc = (IRQn_Type)capsense_ISR__INTC_NUMBER,
        .intrPriority = capsense_ISR__INTC_CORTEXM4_PRIORITY
    };

#endif /* ((__CORTEX_M == 4) && (CY_CORE_ID == 0)) */

