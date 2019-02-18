/***************************************************************************//**
* \file capsense_INT.c
* \version 2.0
*
* \brief
*   This file contains the source code for implementation of the capsense
*   Component Interrupt Service Routine (ISR).
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
#include "syslib/cy_syslib.h"
#include "cyfitter.h"
#include "capsense_Configuration.h"
#include "capsense_Structure.h"
#include "capsense_Sensing.h"
#if (capsense_ENABLE == capsense_CSD_EN)
    #include "capsense_SensingCSD_LL.h"
#endif /* (capsense_ENABLE == capsense_CSD_EN) */
#include "cyapicallbacks.h"

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/

#if (((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN)) && \
     (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN))
    static void capsense_SsNextFrequencyScan(void);
#endif /* (((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN)) && \
            (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)) */

#if ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN))
    static void capsense_SsCSDPostScan(void);
    static void capsense_SsCSDInitNextScan(void);
#endif /* ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN)) */
/** \}
* \endcond */


/**
* \cond SECTION_CYSENSE_INTERRUPT
* \addtogroup group_cysense_interrupt
* \{
*/


#if ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN))


    /*******************************************************************************
    * Function Name: capsense_CSDPostSingleScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the single-sensor scanning implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  capsense_CSDScanExt() function.
    *
    *  The following tasks are performed:
    *    1. Check if the raw data is not noisy.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Configure and start the scan for the next frequency if the
    *      multi-frequency is enabled.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The ISR handler changes the scanning for the next frequency
    *  channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user code
    *  from macros specified in Component's generated code. Refer to the
    *  \ref group_cysense_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    void capsense_CSDPostSingleScan(void)
    {
        #ifdef capsense_ENTRY_CALLBACK
            capsense_EntryCallback();
        #endif /* capsense_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(capsense_CSD_INTR_PTR, capsense_CSD_INTR_ALL_MSK);
        (void)CY_GET_REG32(capsense_CSD_INTR_PTR);

        #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
            if ((capsense_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(capsense_CSD_RESULT_VAL1_PTR) &
                                                        capsense_CSD_RESULT_VAL1_BAD_CONVS_MSK) >>
                                                        capsense_CSD_RESULT_VAL1_BAD_CONVS_POS)) &&
                                                        (0u < capsense_badConversionsNum))
            {
                /* Decrement bad conversions number */
                capsense_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(capsense_CSD_SEQ_START_PTR, capsense_CSD_SEQ_START_AZ0_SKIP_MSK |
                                                             capsense_CSD_SEQ_START_AZ1_SKIP_MSK |
                                                             capsense_CSD_SEQ_START_START_MSK);
            }
            else
            {
        #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

            capsense_SsCSDPostScan();

            #if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
                if (capsense_FREQ_CHANNEL_2 > capsense_scanFreqIndex)
                {
                    /* Scan the next channel */
                    capsense_SsNextFrequencyScan();
                }
                else
                {
                    /* All channels are scanned. Reset the frequency scan channel */
                    capsense_scanFreqIndex = capsense_FREQ_CHANNEL_0;
                    capsense_SsChangeClkFreq(capsense_FREQ_CHANNEL_0);

                    #if (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable HW IP block */
                        CY_SET_REG32(capsense_CSD_CONFIG_PTR, capsense_configCsd);
                    #endif /* (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    capsense_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    capsense_dsRam.status &= ~(capsense_SW_STS_BUSY | capsense_WDGT_SW_STS_BUSY);
                }
            #else
                {
                    #if (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable HW IP block */
                        CY_SET_REG32(capsense_CSD_CONFIG_PTR, capsense_configCsd);
                    #endif /* (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    capsense_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    capsense_dsRam.status &= ~(capsense_SW_STS_BUSY | capsense_WDGT_SW_STS_BUSY);
                }
            #endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */
    #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
        }
    #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

        #ifdef capsense_EXIT_CALLBACK
            capsense_ExitCallback();
        #endif /* capsense_EXIT_CALLBACK */
    }


    /*******************************************************************************
    * Function Name: capsense_CSDPostMultiScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  capsense_Scan() or capsense_ScanAllWidgets() APIs.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  capsense_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_cysense_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    void capsense_CSDPostMultiScan(void)
    {
        /* Declare and initialize ptr to sensor IO structure to appropriate address */
        capsense_FLASH_IO_STRUCT const *curSnsIOPtr = (capsense_FLASH_IO_STRUCT const *)
                                                          capsense_dsFlash.wdgtArray[capsense_widgetIndex].ptr2SnsFlash
                                                          + capsense_sensorIndex;

        #ifdef capsense_ENTRY_CALLBACK
            capsense_EntryCallback();
        #endif /* capsense_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(capsense_CSD_INTR_PTR, capsense_CSD_INTR_ALL_MSK);
        (void)CY_GET_REG32(capsense_CSD_INTR_PTR);

        #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
            if ((capsense_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(capsense_CSD_RESULT_VAL1_PTR) &
                                                      capsense_CSD_RESULT_VAL1_BAD_CONVS_MSK) >>
                                                      capsense_CSD_RESULT_VAL1_BAD_CONVS_POS)) &&
                                                      (0u < capsense_badConversionsNum))
            {
                /* Decrement bad conversions number */
                capsense_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(capsense_CSD_SEQ_START_PTR, capsense_CSD_SEQ_START_AZ0_SKIP_MSK |
                                                             capsense_CSD_SEQ_START_AZ1_SKIP_MSK |
                                                             capsense_CSD_SEQ_START_START_MSK);
            }
            else
            {
        #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

        capsense_SsCSDPostScan();

        /* Disable sensor when all frequency channels are scanned */
        #if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
            if (capsense_FREQ_CHANNEL_2 == capsense_scanFreqIndex)
        #endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */
        {
            /* Disable sensor */
            capsense_CSDDisconnectSns(curSnsIOPtr);
        }

        #if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
            if (capsense_FREQ_CHANNEL_2 > capsense_scanFreqIndex)
            {
                 /* Scan the next channel */
                capsense_SsNextFrequencyScan();
            }
            else
            {
                /* All channels are scanned. Reset the frequency scan channel */
                capsense_scanFreqIndex = capsense_FREQ_CHANNEL_0;
                capsense_SsChangeClkFreq(capsense_FREQ_CHANNEL_0);

                 /* Scan the next sensor */
                capsense_SsCSDInitNextScan();
            }
        #else
            /* Scan the next sensor */
            capsense_SsCSDInitNextScan();
        #endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */
    #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
        }
    #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

    #ifdef capsense_EXIT_CALLBACK
        capsense_ExitCallback();
    #endif /* capsense_EXIT_CALLBACK */
}


#if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
/*******************************************************************************
* Function Name: capsense_CSDPostMultiScanGanged
****************************************************************************//**
*
* \brief
*  This is an internal ISR function for the multiple-sensor scanning
*  implementation for ganged sensors.
*
* \details
*  This ISR handler is triggered when the user calls the
*  capsense_Scan() API for a ganged sensor or the
*  capsense_ScanAllWidgets() API in the project with ganged sensors.
*
*  The following tasks are performed:
*    1. Disable the CSD interrupt.
*    2. Read the Counter register and update the data structure with raw data.
*    3. Connect the Vref buffer to the AMUX bus.
*    4. Disable the CSD block (after the widget has been scanned).
*    5. Update the Scan Counter.
*    6. Reset the BUSY flag.
*    7. Enable the CSD interrupt.
*
*  The ISR handler initializes scanning for the previous sensor when the
*  widget has more than one sensor.
*  The ISR handler initializes scanning for the next widget when the
*  capsense_ScanAllWidgets() APIs are called and the project has
*  more than one widget.
*  The ISR handler changes the IMO and initializes scanning for the next
*  frequency channels when multi-frequency scanning is enabled.
*
*  This function has two Macro Callbacks that allow calling the user
*  code from macros specified in Component's generated code. Refer to the
*  \ref group_cysense_macrocallbacks section of the PSoC Creator User Guide
*  for details.
*
*******************************************************************************/
void capsense_CSDPostMultiScanGanged(void)
{
    #ifdef capsense_ENTRY_CALLBACK
        capsense_EntryCallback();
    #endif /* capsense_ENTRY_CALLBACK */

        /* Clear pending interrupt */
    CY_SET_REG32(capsense_CSD_INTR_PTR, capsense_CSD_INTR_ALL_MSK);
    (void)CY_GET_REG32(capsense_CSD_INTR_PTR);

    #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
        if ((capsense_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(capsense_CSD_RESULT_VAL1_PTR) &
                                                  capsense_CSD_RESULT_VAL1_BAD_CONVS_MSK) >>
                                                  capsense_CSD_RESULT_VAL1_BAD_CONVS_POS)) &&
                                                  (0u < capsense_badConversionsNum))
        {
            /* Decrement bad conversions number */
            capsense_badConversionsNum--;

            /* Start the re-scan */
            CY_SET_REG32(capsense_CSD_SEQ_START_PTR, capsense_CSD_SEQ_START_AZ0_SKIP_MSK |
                                                         capsense_CSD_SEQ_START_AZ1_SKIP_MSK |
                                                         capsense_CSD_SEQ_START_START_MSK);
        }
        else
        {
    #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

        capsense_SsCSDPostScan();

        #if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
            if (capsense_FREQ_CHANNEL_2 == capsense_scanFreqIndex)
        #endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */
        {
            capsense_SsCSDDisconnectSnsExt((uint32)capsense_widgetIndex, (uint32)capsense_sensorIndex);
        }

        #if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
            if (capsense_FREQ_CHANNEL_2 > capsense_scanFreqIndex)
            {
                 /* Scan the next channel */
                capsense_SsNextFrequencyScan();
            }
            else
            {
                /* All channels are scanned. Reset the frequency scan channel */
                capsense_scanFreqIndex = capsense_FREQ_CHANNEL_0;
                capsense_SsChangeClkFreq(capsense_FREQ_CHANNEL_0);

                 /* Scan the next sensor */
                capsense_SsCSDInitNextScan();
            }
        #else
             /* Scan the next sensor */
            capsense_SsCSDInitNextScan();
        #endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */
    #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
        }
    #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

    #ifdef capsense_EXIT_CALLBACK
        capsense_ExitCallback();
    #endif /* capsense_EXIT_CALLBACK */

    (void)CY_GET_REG32(capsense_CSD_INTR_PTR);
}
#endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */


#endif /* ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN)) */

/** \}
 * \endcond */


#if ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN))

/*******************************************************************************
* Function Name: capsense_SsCSDPostScan
****************************************************************************//**
*
* \brief
*   This function reads rawdata and releases required HW resources after scan.
*
* \details
*   This function performs following tasks after scan:
*   - Reads SlotResult from Raw Counter;
*   - Inits bad Conversions number;
*   - Disconnects Vrefhi from AMUBUF positive input;
*   - Disconnects AMUBUF output from CSDBUSB with sych PHI2+HSCMP;
*   - Opens HCBV and HCBG switches.
*
*******************************************************************************/
static void capsense_SsCSDPostScan(void)
{
    uint32 tmpRawData;
    uint32 tmpMaxCount;
    capsense_RAM_WD_BASE_STRUCT const *ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                                        capsense_dsFlash.wdgtArray[capsense_widgetIndex].ptr2WdgtRam;

    /* Read SlotResult from Raw Counter */
    tmpRawData = capsense_CSD_RESULT_VAL1_REG & capsense_CSD_RESULT_VAL1_VALUE_MSK;

    tmpMaxCount = ((1uL << ptrWdgt->resolution) - 1uL);
    if(tmpRawData < tmpMaxCount)
    {
        capsense_curRamSnsPtr->raw[capsense_scanFreqIndex] = CY_LO16(tmpRawData);
    }
    else
    {
        capsense_curRamSnsPtr->raw[capsense_scanFreqIndex] = CY_LO16(tmpMaxCount);
    }

    #if (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN)
        /* Init bad Conversions number */
        capsense_badConversionsNum = capsense_BAD_CONVERSIONS_NUM;
    #endif /* (capsense_ENABLE == capsense_CSD_NOISE_METRIC_EN) */

    #if (capsense_ENABLE == capsense_CSD_SHIELD_EN)
        /* Open HCBV and HCBG switches */
        CY_SET_REG32(capsense_CSD_SW_SHIELD_SEL_PTR, capsense_CSD_SW_SHIELD_SEL_SW_HCBV_STATIC_OPEN |
                                                         capsense_CSD_SW_SHIELD_SEL_SW_HCBG_STATIC_OPEN);
    #endif /* (capsense_ENABLE == capsense_CSD_SHIELD_EN) */
}


/*******************************************************************************
* Function Name: capsense_SsCSDInitNextScan
****************************************************************************//**
*
* \brief
*   This function initializes the next sensor scan.
*
* \details
*   The function increments the sensor index, updates sense clock for matrix
*   or touchpad widgets only, sets up Compensation IDAC, enables the sensor and
*   scans it. When all the sensors are scanned it continues to set up the next widget
*   until all the widgets are scanned. The CSD block is disabled when all the widgets are
*   scanned.
*
*******************************************************************************/
static void capsense_SsCSDInitNextScan(void)
{
    /* Declare and initialize ptr to widget and sensor structures to appropriate address */
    #if (((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) || \
            (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)) || \
            (((capsense_DISABLE == capsense_CSD_COMMON_SNS_CLK_EN) && \
            (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN))))
        capsense_RAM_WD_BASE_STRUCT *ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
            capsense_dsFlash.wdgtArray[capsense_widgetIndex].ptr2WdgtRam;
    #endif

    /* Check if all the sensors are scanned in widget */
    if (((uint8)capsense_dsFlash.wdgtArray[(capsense_widgetIndex)].totalNumSns - 1u) > capsense_sensorIndex)
    {
        /* Increment sensor index to configure next sensor in widget */
        capsense_sensorIndex++;

        /* Update global pointer to capsense_RAM_SNS_STRUCT to current sensor */
        capsense_curRamSnsPtr = (capsense_RAM_SNS_STRUCT *)
                                                  capsense_dsFlash.wdgtArray[capsense_widgetIndex].ptr2SnsRam
                                                  + capsense_sensorIndex;

        /* Configure clock divider to row or column */
        #if ((capsense_DISABLE == capsense_CSD_COMMON_SNS_CLK_EN) && \
             (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN))
            if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[(capsense_widgetIndex)].wdgtType) ||
                (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[(capsense_widgetIndex)].wdgtType))
            {
                capsense_SsCSDConfigClock();

                /* Set up scanning resolution */
                capsense_SsCSDCalculateScanDuration(ptrWdgt);
            }
        #endif /* ((capsense_DISABLE == capsense_CSD_COMMON_SNS_CLK_EN) && \
                   (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN))) */

        /* Setup Compensation IDAC for next sensor in widget */
        #if ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) || \
             (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN))
            capsense_SsCSDSetUpIdacs(ptrWdgt);
        #endif /* ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) || \
             (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)) */

        /* Enable sensor */
        capsense_SsCSDConnectSensorExt((uint32)capsense_widgetIndex, (uint32)capsense_sensorIndex);

        /* Proceed scanning */
        capsense_SsCSDStartSample();
    }
    /* Call scan next widget API if requested, if not, complete the scan */
    else
    {
        capsense_sensorIndex = 0u;

        /* Current widget is totally scanned. Reset WIDGET BUSY flag */
        capsense_dsRam.status &= ~capsense_WDGT_SW_STS_BUSY;

        /* Check if all the widgets have been scanned */
        if (capsense_ENABLE == capsense_requestScanAllWidget)
        {
            /* Configure and begin scanning next widget */
            capsense_SsPostAllWidgetsScan();
        }
        else
        {
            #if (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN)
                /* Disable the CSD block */
                CY_SET_REG32(capsense_CSD_CONFIG_PTR, capsense_configCsd);
            #endif /* (capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN) */

            /* all the widgets are totally scanned. Reset BUSY flag */
            capsense_dsRam.status &= ~capsense_SW_STS_BUSY;

            /* Update scan Counter */
            capsense_dsRam.scanCounter++;
        }
    }
}

#if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: capsense_SsNextFrequencyScan
    ****************************************************************************//**
    *
    * \brief
    *   This function scans the sensor on the next frequency channel.
    *
    * \details
    *   The function increments the frequency channel, changes scan frequency and 
    *   initializes the scanning process of the same sensor.
    *
    *******************************************************************************/
    static void capsense_SsNextFrequencyScan(void)
    {
        capsense_RAM_WD_BASE_STRUCT const *ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                                                        capsense_dsFlash.wdgtArray[capsense_widgetIndex].ptr2WdgtRam;

        capsense_scanFreqIndex++;

        /* Set Immunity */
        capsense_SsChangeClkFreq((uint32)capsense_scanFreqIndex);

        /* Update IDAC registers */
        capsense_SsCSDSetUpIdacs(ptrWdgt);

        /* Proceed scanning */
        capsense_SsCSDStartSample();
    }
#endif /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */

#endif /* ((capsense_ENABLE == capsense_CSD_EN) || (capsense_ENABLE == capsense_CSD_CSX_EN)) */


/* [] END OF FILE */
