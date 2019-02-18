/***************************************************************************//**
* \file     capsense_ModClk.h
* \version  1.0
*
* \brief
* Provides the source code to the API for the capsense_ModClk Component.
*
********************************************************************************
* \copyright
* Copyright 2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_capsense_ModClk_NAME_capsense_ModClk_H)
#define CY_capsense_ModClk_NAME_capsense_ModClk_H

/* For _DIV_TYPE and _DIV_NUM #defines */
#include "cyfitter.h"

/* makes uint32_t defined */
#include <stdint.h>

#include "sysclk/cy_sysclk.h"

/***************************************
* Preprocessor Macros
***************************************/
/**
* \addtogroup group_macros
* @{
*/
/** The peripheral clock divider number */
#define capsense_ModClk_DIV_NUM ((uint32_t)capsense_ModClk__DIV_NUM)
/** The peripheral clock divider type */
#define capsense_ModClk_DIV_TYPE ((cy_en_divider_types_t)capsense_ModClk__DIV_TYPE)
/** @} group_macros */

/***************************************
* Function Prototypes
***************************************/

/**
* \addtogroup group_general
* @{
*/

/*******************************************************************************
* Function Name: capsense_ModClk_Enable
****************************************************************************//**
*
* Enables the programmable clock divider assigned to this Component.
*
* \param None
*
* \return None
*
*******************************************************************************/
__STATIC_INLINE void capsense_ModClk_Enable(void)
{
    (void)Cy_SysClk_PeriphEnableDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM);
}

/*******************************************************************************
* Function Name: capsense_ModClk_Disable
****************************************************************************//**
*
* Disables the programmable clock divider assigned to this Component.
*
* \param None
*
* \return None
*
*******************************************************************************/
__STATIC_INLINE void capsense_ModClk_Disable(void)
{
    (void)Cy_SysClk_PeriphDisableDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM);
}

/*******************************************************************************
* Function Name: capsense_ModClk_SetDivider
****************************************************************************//**
*
* Sets the value of a programmable clock divider assigned to this Component.
* This is only used for integer dividers. Use capsense_ModClk_SetFracDivider()
* for setting factional dividers.
*
* \param dividerValue
* The divider value. The source of the divider is peri_clk which is a divided
* version of hf_clk[0]. The divider value causes integer division of
* (divider value + 1), or division by 1 to 256 (8-bit divider) or
* 1 to 65536 (16-bit divider).
*
* \return None
*
*******************************************************************************/
__STATIC_INLINE void capsense_ModClk_SetDivider(uint32_t dividerValue)
{
    (void)Cy_SysClk_PeriphSetDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM, dividerValue);
}

/*******************************************************************************
* Function Name: capsense_ModClk_GetDivider
****************************************************************************//**
*
* Returns the integer divider value for the programmable clock divider assigned
* to this Component. This is only used for integer dividers.
* Use capsense_ModClk_GetFracDivider() with a fractional divider.
*
* \param None
*
* \return
* The divider value. The source of the divider is peri_clk which is a divided
* version of hf_clk[0]. The integer division done is by (divider value + 1),
* or division by 1 to 256 (8-bit divider) or 1 to 65536 (16-bit divider).
*
*******************************************************************************/
__STATIC_INLINE uint32_t capsense_ModClk_GetDivider(void)
{
    return Cy_SysClk_PeriphGetDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM);
}

/*******************************************************************************
* Function Name: capsense_ModClk_SetFracDivider
****************************************************************************//**
*
* Sets the values of a programmable clock divider assigned to this Component.
* This is only used for fractional dividers. Use capsense_ModClk_SetDivider()
* for setting integer dividers.
*
* \param dividerIntValue
* The integer divider value. The source of the divider is peri_clk which is a
* divided version of hf_clk[0]. The divider value causes integer division of
* (divider value + 1), or division by 1 to 65536 (16-bit divider) or
* 1 to 16777216 (24-bit divider).
*
* \param dividerFracValue
* This is the fraction piece of the divider. The fractional divider can be
* 0 - 31; it divides the clock by 1/32 for each count. To divide the clock by
* 11/32nds, set this value to 11.
*
* \return None
*
*******************************************************************************/
__STATIC_INLINE void capsense_ModClk_SetFracDivider(uint32_t dividerIntValue, uint32_t dividerFracValue)
{
    (void)Cy_SysClk_PeriphSetFracDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM, dividerIntValue, dividerFracValue);
}

/*******************************************************************************
* Function Name: capsense_ModClk_GetFracDivider
****************************************************************************//**
*
* Returns the divider values for the programmable clock divider assigned to this
* Component. This is only used for fractional dividers.
* Use capsense_ModClk_GetDivider() with an integer divider.
*
* \param *dividerIntValue
* pointer to return integer divider value
*
* \param *dividerFracValue
* pointer to return fractional divider value
*
* \return None. Loads pointed-to variables.
*
*******************************************************************************/
__STATIC_INLINE void capsense_ModClk_GetFracDivider(uint32_t *dividerIntValue, uint32_t *dividerFracValue)
{
    Cy_SysClk_PeriphGetFracDivider(capsense_ModClk_DIV_TYPE, capsense_ModClk_DIV_NUM, dividerIntValue, dividerFracValue);
}

/** @} general */

#endif /* CY_CLOCK_capsense_ModClk_H */

/* [] END OF FILE */
