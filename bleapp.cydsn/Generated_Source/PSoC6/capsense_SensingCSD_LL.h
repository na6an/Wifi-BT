/***************************************************************************//**
* \file capsense_SensingCSD_LL.h
* \version 2.0
*
* \brief
*   This file provides the headers of APIs specific to CSD sensing implementation.
*
* \see capsense v2.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016-2017), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_SENSE_capsense_SENSINGCSD_LL_H)
#define CY_SENSE_capsense_SENSINGCSD_LL_H

#include "cyfitter_gpio.h"
#include "syslib/cy_syslib.h"
#include "capsense_Structure.h"
#include "capsense_Configuration.h"

/****************************************************************************
* Register and mode mask definition
****************************************************************************/

#define capsense_CSD_CSDCMP_INIT                                (capsense_CSD_CSDCMP_CSDCMP_DISABLED)

/* SW_HS_P_SEL switches state for Coarse initialization of CMOD (sense path) */
#if (capsense_ENABLE == capsense_CSD_EN)
    #if (capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION)
        #define capsense_CSD_HS_P_SEL_COARSE_CMOD               (capsense_CSD_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
    #elif (capsense_CSD__CSHIELD_PAD == capsense_CMOD_CONNECTION)
        #define capsense_CSD_HS_P_SEL_COARSE_CMOD               (capsense_CSD_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
    #else
        #define capsense_CSD_HS_P_SEL_COARSE_CMOD               (capsense_CSD_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
    #endif /* (capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION) */
#else
    #define capsense_CSD_HS_P_SEL_COARSE_CMOD                   (0x00000000uL)
#endif /* ((capsense_ENABLE == capsense_CSD_CSX_EN) && (capsense_ENABLE == capsense_CSD_EN)) */

#if ((capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
    (capsense_ENABLE == capsense_CSD_SHIELD_EN))
    /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
    #if (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION)
        #define capsense_CSD_HS_P_SEL_COARSE_TANK               (capsense_CSD_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
    #elif (capsense_CSD__CSHIELD_PAD == capsense_CTANK_CONNECTION)
        #define capsense_CSD_HS_P_SEL_COARSE_TANK               (capsense_CSD_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
    #elif (capsense_CSD__CMOD_PAD == capsense_CTANK_CONNECTION)
        #define capsense_CSD_HS_P_SEL_COARSE_TANK               (capsense_CSD_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
    #else
        #define capsense_CSD_HS_P_SEL_COARSE_TANK               (capsense_CSD_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE)
    #endif /* (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION) */
#else
    #define capsense_CSD_HS_P_SEL_COARSE_TANK                   (0x00000000uL)
#endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
           (capsense_ENABLE == capsense_CSD_SHIELD_EN)) */

#define capsense_CSD_SW_HS_P_SEL_COARSE                         (capsense_CSD_HS_P_SEL_COARSE_CMOD | capsense_CSD_HS_P_SEL_COARSE_TANK)

/* C_mod config */
#if ((capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION) || (capsense_CSD__CMOD_PAD == capsense_CTANK_CONNECTION))
    #define capsense_CSD_SW_FW_MOD_SEL_INIT             (capsense_CSD_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                 capsense_CSD_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE |\
                                                                 capsense_CSD_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE)
#else
    #define capsense_CSD_SW_FW_MOD_SEL_INIT             (0x00000000uL)
#endif /* (capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION) */

/* C_tank config */
#if ((capsense_CSD__CSH_TANK_PAD == capsense_CMOD_CONNECTION) || (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION))
    #define capsense_CSD_SW_FW_TANK_SEL_INIT            (capsense_CSD_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                 capsense_CSD_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE |\
                                                                 capsense_CSD_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE)
#else
    #define capsense_CSD_SW_FW_TANK_SEL_INIT            (0x00000000uL)
#endif /* (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION) */

#define capsense_CSD_SW_SHIELD_SEL_INIT                 (capsense_CSD_SW_SHIELD_SEL_SW_HCAV_HSCMP)

/* Defining default HW configuration according to settings in customizer. */
#define capsense_DEFAULT_CSD_CONFIG                 (capsense_CSD_CONFIG_FILTER_DELAY_12MHZ |\
                                                             capsense_IREF_SRC_CFG |\
                                                             capsense_PWR_MODE_CFG)

#if(capsense_SENSING_LEGACY == capsense_CSD_SENSING_METHOD)

    #if (capsense_ENABLE == capsense_CSD_AUTO_ZERO_EN)
        /* Enable CSDCMP */
        #define capsense_CSD_CSDCMP_SCAN                (capsense_CSD_CSDCMP_CSDCMP_EN_MSK |\
                                                                 capsense_CSD_CSDCMP_AZ_EN_MSK)
    #else
        /* Enable CSDCMP */
        #define capsense_CSD_CSDCMP_SCAN                (capsense_CSD_CSDCMP_CSDCMP_EN_MSK)
    #endif /* (capsense_ENABLE == capsense_CSD_AUTO_ZERO_EN) */

    #if ((capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
        (capsense_ENABLE == capsense_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION)
            #define capsense_CSD_HS_P_SEL_SCAN_TANK                 (capsense_CSD_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (capsense_CSD__CSHIELD_PAD == capsense_CTANK_CONNECTION)
            #define capsense_CSD_HS_P_SEL_SCAN_TANK                 (capsense_CSD_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (capsense_CSD__CMOD_PAD == capsense_CTANK_CONNECTION)
            #define capsense_CSD_HS_P_SEL_SCAN_TANK                 (capsense_CSD_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define capsense_CSD_HS_P_SEL_SCAN_TANK                 (capsense_CSD_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
        #endif /* (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION) */
        #define capsense_CSD_SW_HS_P_SEL_SCAN                       (capsense_CSD_HS_P_SEL_SCAN_TANK)
    #elif(capsense_ENABLE == capsense_CSD_SHIELD_EN)
        #define capsense_CSD_SW_HS_P_SEL_SCAN                       (capsense_CSD_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #else
        #define capsense_CSD_SW_HS_P_SEL_SCAN                       (capsense_CSD_WAVEFORM_STATIC_OPEN)
    #endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
               (capsense_ENABLE == capsense_CSD_SHIELD_EN)) */

    /* SW_FW_MOD_SEL switches state for Coarse initialization of CMOD (sense path) */
    #define capsense_CSD_SW_FW_MOD_SEL_SCAN                 (0x00000000uL)

    #if((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
        (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
        (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION))
        #define capsense_CSD_SW_FW_TANK_SEL_SCAN            (capsense_CSD_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE | \
                                                                 capsense_CSD_SW_FW_TANK_SEL_SW_F2CB_STATIC_CLOSE)
    #else
        #define capsense_CSD_SW_FW_TANK_SEL_SCAN            (0x00000000uL)
    #endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
               (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN) && \
               (capsense_CSD__CSH_TANK_PAD == capsense_CTANK_CONNECTION)) */

    /* Shield switch default config */
    #if ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
         (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN))
        #if (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG)
            #define capsense_CSD_SW_SHIELD_SEL_SCAN          (capsense_CSD_SW_SHIELD_SEL_SW_HCBG_HSCMP)
        #else
            #define capsense_CSD_SW_SHIELD_SEL_SCAN          (capsense_CSD_SW_SHIELD_SEL_SW_HCBV_HSCMP)
        #endif /* (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG) */
    #elif(capsense_ENABLE == capsense_CSD_SHIELD_EN)
        #if (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG)
            #define capsense_CSD_SW_SHIELD_SEL_SCAN          (capsense_CSD_SW_SHIELD_SEL_SW_HCBV_PHI1 | \
                                                                     capsense_CSD_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP)
        #else
            #define capsense_CSD_SW_SHIELD_SEL_SCAN          (capsense_CSD_SW_SHIELD_SEL_SW_HCBG_PHI1 | \
                                                                     capsense_CSD_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP)
        #endif /* (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG) */
    #else
        #define capsense_CSD_SW_SHIELD_SEL_SCAN              (0x00000000uL)
    #endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
               (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN)) */

    #define capsense_CSD_SW_RES_INIT                        (capsense_CSD_INIT_SWITCH_RES << CSD_SW_RES_RES_HCAV_Pos)
    #define capsense_CSD_SW_RES_SCAN                        ((capsense_CSD_SHIELD_SWITCH_RES << CSD_SW_RES_RES_HCBV_Pos) |\
                                                                     (capsense_CSD_SHIELD_SWITCH_RES << CSD_SW_RES_RES_HCBG_Pos))

    #define capsense_CSD_SHIELD_GPIO_DM                 (CY_GPIO_DM_STRONG_IN_OFF)
    #define capsense_CSD_SENSOR_HSIOM_SEL               (capsense_HSIOM_SEL_CSD_SENSE)
    #define capsense_CSD_SHIELD_HSIOM_SEL               (capsense_HSIOM_SEL_CSD_SHIELD)
    #define capsense_CSD_CMOD_HSIOM_SEL                 (capsense_HSIOM_SEL_AMUXA)

    #define capsense_DEFAULT_IDACA_BALL_MODE            (capsense_CSD_IDACA_BALL_MODE_FULL <<\
                                                                 capsense_CSD_IDACA_BALL_MODE_POS)
    #define capsense_DEFAULT_IDACB_BALL_MODE            (capsense_CSD_IDACB_BALL_MODE_FULL <<\
                                                                 capsense_CSD_IDACB_BALL_MODE_POS)

    #define capsense_DEFAULT_SENSE_DUTY_SEL             (capsense_CSD_SENSE_DUTY_SENSE_POL_PHI_HIGH)

#elif(capsense_SENSING_LOW_EMI == capsense_CSD_SENSING_METHOD)

    #if (capsense_ENABLE == capsense_CSD_AUTO_ZERO_EN)
        /* Enable CSDCMP */
        #define capsense_CSD_CSDCMP_SCAN                (capsense_CSD_CSDCMP_CSDCMP_EN_MSK  |\
                                                                 capsense_CSD_CSDCMP_CMP_PHASE_PHI2 |\
                                                                 capsense_CSD_CSDCMP_AZ_EN_MSK)
    #else
        /* Enable CSDCMP */
        #define capsense_CSD_CSDCMP_SCAN                (capsense_CSD_CSDCMP_CSDCMP_EN_MSK |\
                                                                 capsense_CSD_CSDCMP_CMP_PHASE_PHI2)
    #endif /* (capsense_ENABLE == capsense_CSD_AUTO_ZERO_EN) */

    #if(capsense_ENABLE == capsense_CSD_SHIELD_EN)
        #define capsense_CSD_SW_HS_P_SEL_SCAN               (capsense_CSD_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #else
        #define capsense_CSD_SW_HS_P_SEL_SCAN               (capsense_CSD_SW_HS_P_SEL_SW_HMMB_STATIC_OPEN)
    #endif /* (capsense_ENABLE == capsense_CSD_SHIELD_EN) */

    #if (capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION)
        #define capsense_CSD_SW_FW_MOD_SEL_SCAN             (capsense_CSD_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                     capsense_CSD_SW_FW_MOD_SEL_SW_F1CA_PHI2)
        #define capsense_CSD_SW_FW_TANK_SEL_SCAN            (0x00000000uL)
    #else
        #define capsense_CSD_SW_FW_MOD_SEL_SCAN             (0x00000000uL)
        #define capsense_CSD_SW_FW_TANK_SEL_SCAN            (capsense_CSD_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                     capsense_CSD_SW_FW_TANK_SEL_SW_F2CA_PHI2)
    #endif /* (capsense_CSD__CMOD_PAD == capsense_CMOD_CONNECTION) */

    #if(capsense_ENABLE == capsense_CSD_SHIELD_EN)
        #if(capsense_IDAC_SINKING != capsense_CSD_IDAC_CONFIG)
            #define capsense_CSD_SW_SHIELD_SEL_SCAN             (capsense_CSD_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP | capsense_CSD_SW_SHIELD_SEL_SW_HCBG_PHI1 | capsense_CSD_SW_SHIELD_SEL_SW_HCAG_PHI1)
        #else
            #define capsense_CSD_SW_SHIELD_SEL_SCAN             (capsense_CSD_SW_SHIELD_SEL_SW_HCBV_PHI1 | capsense_CSD_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP | capsense_CSD_SW_SHIELD_SEL_SW_HCAV_PHI1)
        #endif /* (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG) */
    #else
        #if(capsense_IDAC_SINKING != capsense_CSD_IDAC_CONFIG)
            #define capsense_CSD_SW_SHIELD_SEL_SCAN             (capsense_CSD_SW_SHIELD_SEL_SW_HCAG_PHI1)
        #else
            #define capsense_CSD_SW_SHIELD_SEL_SCAN             (capsense_CSD_SW_SHIELD_SEL_SW_HCAV_PHI1)
        #endif /* (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG) */
    #endif /* (capsense_ENABLE == capsense_CSD_SHIELD_EN) */

    #define capsense_CSD_SW_RES_INIT                    ((capsense_CSD_F1PM_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_F1PM_Pos) |\
                                                                 (capsense_CSD_F2PT_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_F2PT_Pos) |\
                                                                 (capsense_CSD_HCAG_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_HCAG_Pos) |\
                                                                 (capsense_CSD_HCAV_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_HCAV_Pos) |\
                                                                 (capsense_CSD_HCBG_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_HCBG_Pos) |\
                                                                 (capsense_CSD_HCBV_SWITCH_RES_LE_INIT << CSD_SW_RES_RES_HCBV_Pos))

    #define capsense_CSD_SW_RES_SCAN                    ((capsense_CSD_F1PM_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_F1PM_Pos) |\
                                                                 (capsense_CSD_F2PT_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_F2PT_Pos) |\
                                                                 (capsense_CSD_HCAG_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_HCAG_Pos) |\
                                                                 (capsense_CSD_HCAV_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_HCAV_Pos) |\
                                                                 (capsense_CSD_HCBG_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_HCBG_Pos) |\
                                                                 (capsense_CSD_HCBV_SWITCH_RES_LE_SCAN << CSD_SW_RES_RES_HCBV_Pos))

    #define capsense_CSD_SHIELD_GPIO_DM                 (CY_GPIO_DM_ANALOG)
    #define capsense_CSD_SENSOR_HSIOM_SEL               (capsense_HSIOM_SEL_AMUXA)
    #define capsense_CSD_SHIELD_HSIOM_SEL               (capsense_HSIOM_SEL_AMUXB)
    #define capsense_CSD_CMOD_HSIOM_SEL                 (capsense_HSIOM_SEL_GPIO)

    #define capsense_DEFAULT_IDACA_BALL_MODE            (capsense_CSD_IDACA_BALL_MODE_PHI2 <<\
                                                                 capsense_CSD_IDACA_BALL_MODE_POS)
    #define capsense_DEFAULT_IDACB_BALL_MODE            (capsense_CSD_IDACB_BALL_MODE_FULL <<\
                                                                 capsense_CSD_IDACB_BALL_MODE_POS)

    #if(capsense_IDAC_SINKING != capsense_CSD_IDAC_CONFIG)
        #define capsense_DEFAULT_SENSE_DUTY_SEL             (0x00000000uL)
    #else
        #define capsense_DEFAULT_SENSE_DUTY_SEL             (capsense_CSD_SENSE_DUTY_SENSE_POL_MSK)
    #endif /* (capsense_IDAC_SINKING != capsense_CSD_IDAC_CONFIG) */

#elif(capsense_SENSING_FULL_WAVE == capsense_CSD_SENSING_METHOD)
#else
    #error "Not supported sensing method."
#endif /* (capsense_SENSING_LEGACY == capsense_CSD_SENSING_METHOD) */


/***************************************
* API Constants
***************************************/
/* Definition of time interval that is enough for charging 100nF capacitor */
#define capsense_CSD_AVG_CYCLES_PER_LOOP                    (5u)
#define capsense_CSD_MAX_CHARGE_TIME_US                     (100u)
#define capsense_CSD_PRECHARGE_WATCHDOG_CYCLES_NUM          (((capsense_CPU_CLOCK_FREQ_MHZ) * (capsense_CSD_MAX_CHARGE_TIME_US)) /\
                                                                        (capsense_CSD_AVG_CYCLES_PER_LOOP))

#define capsense_CSD_IDAC_MOD_BITS_USED                     (7u)
#define capsense_CAL_MIDDLE_BIT                             (1uL << (capsense_CSD_IDAC_MOD_BITS_USED - 1u))
/* Increased scan time duration to cover coarse and fine init cycles */
#define capsense_MAX_17_BITS_VALUE                          (0x0001FFFFLu)
#define capsense_MAX_SCAN_TIME                              ((capsense_MAX_17_BITS_VALUE * capsense_CSD_SCANSPEED_DIVIDER) /\
                                                                        (CYDEV_CLK_PERICLK__MHZ))
#define capsense_CALIBR_WATCHDOG_CYCLES_NUM                 (((capsense_CPU_CLOCK_FREQ_MHZ) * (capsense_MAX_SCAN_TIME)) /\
                                                                        (capsense_CSD_AVG_CYCLES_PER_LOOP))

#define capsense_DELAY_EXTRACYCLES_NUM                      (9u)

/* Calibration constants */
#define capsense_IDAC_MOD_MAX_CALIB_ERROR                   (10u)

#if (capsense_IDAC_GAIN_HIGH == capsense_CSD_IDAC_GAIN)
    #define capsense_CSD_IDAC_GAIN_VALUE_NA                 (2400u)
#elif (capsense_IDAC_GAIN_MEDIUM == capsense_CSD_IDAC_GAIN)
    #define capsense_CSD_IDAC_GAIN_VALUE_NA                 (300u)
#else
    #define capsense_CSD_IDAC_GAIN_VALUE_NA                 (37u)
#endif /* (capsense_IDAC_GAIN_HIGH == capsense_CSD_IDAC_GAIN) */

/* Defining the drive mode of pins depending on the Inactive sensor connection setting in the Component customizer. */
#if(capsense_SNS_CONNECTION_GROUND == capsense_CSD_INACTIVE_SNS_CONNECTION)
    #define capsense_CSD_INACTIVE_SNS_GPIO_DM               (CY_GPIO_DM_STRONG)
#elif(capsense_SNS_CONNECTION_HIGHZ == capsense_CSD_INACTIVE_SNS_CONNECTION)
    #define capsense_CSD_INACTIVE_SNS_GPIO_DM               (CY_GPIO_DM_ANALOG)
#elif(capsense_SNS_CONNECTION_SHIELD == capsense_CSD_INACTIVE_SNS_CONNECTION)
    #define capsense_CSD_INACTIVE_SNS_GPIO_DM               (CY_GPIO_DM_STRONG_IN_OFF)
#else
    #error "Unsupported inactive connection for the inactive sensors."
#endif /* (capsense_SNS_CONNECTION_GROUND == capsense_CSD_INACTIVE_SNS_CONNECTION) */


/* Clock Source Mode */
#if (capsense_CLK_SOURCE_DIRECT == capsense_CSD_SNS_CLK_SOURCE)
    #define capsense_DEFAULT_MODULATION_MODE                (capsense_CLK_SOURCE_DIRECT)
#elif (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)
    #define capsense_DEFAULT_MODULATION_MODE                (capsense_CLK_SOURCE_SSC2)
#elif ((capsense_CLK_SOURCE_PRS8) == capsense_CSD_SNS_CLK_SOURCE)
    #define capsense_DEFAULT_MODULATION_MODE                (capsense_CSD_SNS_CLK_SOURCE)
#elif ((capsense_CLK_SOURCE_PRS12) == capsense_CSD_SNS_CLK_SOURCE)
    #define capsense_DEFAULT_MODULATION_MODE                (capsense_CSD_SNS_CLK_SOURCE)
#else
    #define capsense_DEFAULT_MODULATION_MODE                (capsense_CSD_SNS_CLK_SOURCE)
#endif /* (capsense_CLK_SOURCE_DIRECT != capsense_CSD_SNS_CLK_SOURCE) */

/* IDACs Ranges */
#if (capsense_IDAC_GAIN_LOW == capsense_CSD_IDAC_GAIN)
    #define capsense_DEFAULT_IDACA_RANGE                    (capsense_CSD_IDACA_RANGE_IDAC_LO << capsense_CSD_IDACA_RANGE_POS)
    #define capsense_DEFAULT_IDACB_RANGE                    (capsense_CSD_IDACB_RANGE_IDAC_LO << capsense_CSD_IDACB_RANGE_POS)
#elif (capsense_IDAC_GAIN_MEDIUM == capsense_CSD_IDAC_GAIN)
    #define capsense_DEFAULT_IDACA_RANGE                    (capsense_CSD_IDACA_RANGE_IDAC_MED << capsense_CSD_IDACA_RANGE_POS)
    #define capsense_DEFAULT_IDACB_RANGE                    (capsense_CSD_IDACB_RANGE_IDAC_MED << capsense_CSD_IDACB_RANGE_POS)
#else
    #define capsense_DEFAULT_IDACA_RANGE                    (capsense_CSD_IDACA_RANGE_IDAC_HI << capsense_CSD_IDACA_RANGE_POS)
    #define capsense_DEFAULT_IDACB_RANGE                    (capsense_CSD_IDACB_RANGE_IDAC_HI << capsense_CSD_IDACB_RANGE_POS)
#endif

/* IDACs Polarities */
#if (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG)
    #define capsense_DEFAULT_IDACA_POLARITY                 (capsense_CSD_IDACA_POLARITY_VDDA_SNK << capsense_CSD_IDACA_POLARITY_POS)
    #define capsense_DEFAULT_IDACB_POLARITY                 (capsense_CSD_IDACB_POLARITY_VDDA_SNK << capsense_CSD_IDACB_POLARITY_POS)
#else
    #define capsense_DEFAULT_IDACA_POLARITY                 (capsense_CSD_IDACA_POLARITY_VSSA_SRC << capsense_CSD_IDACA_POLARITY_POS)
    #define capsense_DEFAULT_IDACB_POLARITY                 (capsense_CSD_IDACB_POLARITY_VSSA_SRC << capsense_CSD_IDACB_POLARITY_POS)
#endif /* (capsense_IDAC_SINKING == capsense_CSD_IDAC_CONFIG) */


#if(capsense_VREF_SRSS != capsense_CSD_VREF_SOURCE)
    #define capsense_CSD_SW_REFGEN_VREF_SRC                 (capsense_CSD_SW_REFGEN_SEL_SW_SGRP_MSK)
#else
    #define capsense_CSD_SW_REFGEN_VREF_SRC                 (capsense_CSD_SW_REFGEN_SEL_SW_SGR_MSK)
#endif /* (capsense_VREF_SRSS != capsense_CSD_VREF_SOURCE) */


/* IDAC legs configuration */
#if ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
     (capsense_ENABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN))
        #define capsense_DEFAULT_SW_REFGEN_SEL              (capsense_CSD_SW_REFGEN_VREF_SRC | capsense_CSD_SW_REFGEN_SEL_SW_IAIB_MSK)
        #define capsense_DEFAULT_IDACB_LEG1_MODE            (capsense_CSD_IDACB_LEG1_EN_MSK |\
                                                                    (capsense_CSD_IDACB_LEG1_MODE_CSD_STATIC << capsense_CSD_IDACB_LEG1_MODE_POS))
#else
        #define capsense_DEFAULT_SW_REFGEN_SEL              (capsense_CSD_SW_REFGEN_VREF_SRC)
        #define capsense_DEFAULT_IDACB_LEG1_MODE            (capsense_CSD_IDACB_LEG1_MODE_GP_STATIC << capsense_CSD_IDACB_LEG1_MODE_POS)
#endif /* ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
           (capsense_ENABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN)) */


#if ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
     (capsense_DISABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN))
        #define capsense_DEFAULT_IDACB_LEG2_MODE            (capsense_CSD_IDACA_LEG2_EN_MSK |\
                                                                    (capsense_CSD_IDACA_LEG2_MODE_CSD_STATIC << capsense_CSD_IDACA_LEG2_MODE_POS))
#else
        #define capsense_DEFAULT_IDACB_LEG2_MODE            (capsense_CSD_IDACA_LEG2_MODE_GP_STATIC << capsense_CSD_IDACA_LEG2_MODE_POS)
#endif /* ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
        (capsense_DISABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN)) */

/* IDACs register configuration is based on the Component configuration */
#define capsense_IDAC_MOD_DEFAULT_CFG                       (capsense_DEFAULT_IDACA_RANGE | \
                                                                     capsense_DEFAULT_IDACA_POLARITY | \
                                                                     capsense_DEFAULT_IDACA_BALL_MODE | \
                                                                    (capsense_CSD_IDACA_LEG1_MODE_CSD << capsense_CSD_IDACA_LEG1_MODE_POS) | \
                                                                     capsense_CSD_IDACA_LEG1_EN_MSK | \
                                                                     capsense_DEFAULT_IDACB_LEG2_MODE)

#define capsense_IDAC_COMP_DEFAULT_CFG                      (capsense_DEFAULT_IDACB_RANGE | \
                                                                     capsense_DEFAULT_IDACB_POLARITY | \
                                                                     capsense_DEFAULT_IDACB_BALL_MODE | \
                                                                     capsense_DEFAULT_IDACB_LEG1_MODE | \
                                                                    (capsense_CSD_IDACB_LEG2_MODE_GP_STATIC << capsense_CSD_IDACB_LEG2_MODE_POS))

#define capsense_IDAC_MOD_CALIBRATION_CFG                   (capsense_DEFAULT_IDACA_RANGE | \
                                                                     capsense_DEFAULT_IDACA_POLARITY | \
                                                                     capsense_DEFAULT_IDACA_BALL_MODE | \
                                                                    (capsense_CSD_IDACA_LEG1_MODE_CSD << capsense_CSD_IDACA_LEG1_MODE_POS) | \
                                                                     capsense_CSD_IDACA_LEG1_EN_MSK | \
                                                                    (capsense_CSD_IDACA_LEG2_MODE_GP_STATIC << capsense_CSD_IDACA_LEG2_MODE_POS))

#define capsense_IDAC_COMP_CALIBRATION_CFG                  (capsense_DEFAULT_IDACB_RANGE | \
                                                                     capsense_DEFAULT_IDACB_POLARITY | \
                                                                     capsense_DEFAULT_IDACB_BALL_MODE | \
                                                                    (capsense_CSD_IDACB_LEG1_MODE_GP_STATIC << capsense_CSD_IDACB_LEG1_MODE_POS) | \
                                                                    (capsense_CSD_IDACB_LEG2_MODE_GP_STATIC << capsense_CSD_IDACB_LEG2_MODE_POS))

/***************************************
* Global software variables
***************************************/

extern uint32 capsense_configCsd;

#if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
    extern uint8 capsense_badConversionsNum;
#endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

#if (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE)
    /* Stores IDAC and raw count that corresponds to a sensor with maximum Cp within a widget */
    extern uint8 capsense_calibratedIdac;
    extern uint16 capsense_calibratedRawcount;
    #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
        extern uint8 capsense_calibratedIdacRow;
        extern uint16 capsense_calibratedRawcountRow;
    #endif /*(capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */
#endif /* (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) */

/***************************************
* Function Prototypes
**************************************/

/* Interrupt handler */
extern void capsense_CSDPostSingleScan(void);
extern void capsense_CSDPostMultiScan(void);

#if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
    extern void capsense_CSDPostMultiScanGanged(void);
#endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */


/**
* \cond SECTION_CYSENSE_LOW_LEVEL
* \addtogroup group_cysense_low_level
* \{
*/

void capsense_CSDSetupWidget(uint32 widgetId);
void capsense_CSDSetupWidgetExt(uint32 widgetId, uint32 sensorId);
void capsense_CSDScan(void);
void capsense_CSDScanExt(void);
#if ((capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) || \
     (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN))
    cy_status capsense_CSDCalibrateWidget(uint32 widgetId, uint32 target);
#endif /* ((capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) || \
           (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN)) */
void capsense_CSDConnectSns(capsense_FLASH_IO_STRUCT const *snsAddrPtr);
void capsense_CSDDisconnectSns(capsense_FLASH_IO_STRUCT const *snsAddrPtr);

/** \}
* \endcond */

/*****************************************************
* Function Prototypes - internal Low Level functions
*****************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/

void capsense_SsCSDInitialize(void);
void capsense_SsCSDStartSample(void);
void capsense_SsCSDSetUpIdacs(capsense_RAM_WD_BASE_STRUCT const *ptrWdgt);
void capsense_SsCSDConfigClock(void);
void capsense_SsCSDElectrodeCheck(void);
#if ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
     (0u != capsense_CSD_TOTAL_SHIELD_COUNT))
    void capsense_SsCSDDisableShieldElectrodes(void);
#endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
           (0u != capsense_CSD_TOTAL_SHIELD_COUNT)) */
uint32 capsense_SsCSDGetNumberOfConversions(uint32 snsClkDivider, uint32 resolution, uint32 snsClkSrc);
void capsense_SsCSDCalculateScanDuration(capsense_RAM_WD_BASE_STRUCT const *ptrWdgt);
void capsense_SsCSDConnectSensorExt(uint32 widgetId, uint32 sensorId);
void capsense_SsCSDDisconnectSnsExt(uint32 widgetId, uint32 sensorId);

#if ((capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) || \
     (capsense_ENABLE == capsense_SELF_TEST_EN) || \
     (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN))
#endif /* ((capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) || \
           (capsense_ENABLE == capsense_SELF_TEST_EN) || \
           (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN)) */

/** \}
* \endcond */

/***************************************
* Global software variables
***************************************/
extern uint32 capsense_configCsd;
/* Interrupt handler */
extern void capsense_CSDPostSingleScan(void);
extern void capsense_CSDPostMultiScan(void);
#if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
extern void capsense_CSDPostMultiScanGanged(void);
#endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */
#if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
    extern uint8 capsense_badConversionsNum;
#endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

#endif /* End CY_SENSE_capsense_SENSINGCSD_LL_H */


/* [] END OF FILE */
