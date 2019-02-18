/***************************************************************************//**
* \file capsense_Sensing.c
* \version 2.0
*
* \brief
*   This file contains the source of functions common for
*   different sensing methods.
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

#include <stdlib.h>
#include "cyfitter.h"
#include "gpio/cy_gpio.h"
#include "cyfitter_sysint_cfg.h"
#include "capsense_ModClk.h"
#include "capsense_Configuration.h"
#include "capsense_Structure.h"
#include "capsense_Sensing.h"
#if (capsense_ENABLE == capsense_CSX_EN)
    #include "capsense_SensingCSX_LL.h"
#endif /* (capsense_ENABLE == capsense_CSX_EN) */
#if (capsense_ENABLE == capsense_CSD_EN)
    #include "capsense_SensingCSD_LL.h"
#endif /* (capsense_ENABLE == capsense_CSD_EN) */
#if (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE)
    #include "capsense_SmartSense_LL.h"
#endif  /* (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE) */
#if (capsense_ENABLE == capsense_ADC_EN)
    #include "capsense_Adc.h"
#endif /* (capsense_ENABLE == capsense_ADC_EN) */
#if (capsense_ENABLE == capsense_SELF_TEST_EN)
    #include "capsense_SelfTest.h"
#endif

/***************************************
* API Constants
***************************************/

#define capsense_CALIBRATION_RESOLUTION                 (12u)
#define capsense_CALIBRATION_FREQ_KHZ                   (1500u)
#define capsense_CSD_AUTOTUNE_CAL_LEVEL                 (capsense_CSD_RAWCOUNT_CAL_LEVEL)
#define capsense_CSD_AUTOTUNE_CAL_UNITS                 (1000u)
#define capsense_CP_MIN                                 (0u)
#define capsense_CP_MAX                                 (65000Lu)
#define capsense_CP_ERROR                               (4000Lu)
#define capsense_CLK_SOURCE_LFSR_SCALE_OFFSET           (4u)
#define capsense_CSD_SNS_FREQ_KHZ_MAX                   (6000u)

#define capsense_MOD_CSD_CLK_12000KHZ                   (12000uL)
#define capsense_MOD_CSD_CLK_24000KHZ                   (24000uL)
#define capsense_MOD_CSD_CLK_48000KHZ                   (48000uL)

#define capsense_EXT_CAP_DISCHARGE_TIME                 (1u)

/*****************************************************************************/
/* Enumeration types definition                                              */
/*****************************************************************************/

typedef enum
{
    capsense_RES_PULLUP_E   = 0x02u,
    capsense_RES_PULLDOWN_E = 0x03u
} capsense_PORT_TEST_DM;

typedef enum
{
    capsense_STS_RESET      = 0x01u,
    capsense_STS_NO_RESET   = 0x02u
} capsense_TEST_TYPE;


/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/

#if ((capsense_ENABLE == capsense_CSD_CSX_EN) || \
     (capsense_ENABLE == capsense_SELF_TEST_EN) || \
     (capsense_ENABLE == capsense_ADC_EN))
    #if (capsense_ENABLE == capsense_CSD_EN)
        static void capsense_SsCSDDisableMode(void);
    #endif /* (capsense_ENABLE == capsense_CSD_EN) */
    #if (capsense_ENABLE == capsense_CSX_EN)
        static void capsense_SsDisableCSXMode(void);
    #endif /* (capsense_ENABLE == capsense_CSX_EN) */
#endif

#if(((capsense_ENABLE == capsense_CSX_EN) && \
     (capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE) && \
     (capsense_DISABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)) ||\
    ((capsense_ENABLE == capsense_CSD_EN) && \
     (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)))
    static uint8 capsense_SsCalcLfsrSize(uint32 snsClkDivider, uint32 conversionsNum);
    static uint8 capsense_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize);
#endif

#if (capsense_ENABLE == capsense_CSD_EN)
    static void capsense_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, capsense_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif

#if (capsense_ENABLE == capsense_CSX_EN)
    __STATIC_INLINE void capsense_SsSetWidgetTxClkSrc(uint32 wdgtIndex, capsense_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif

/** \}
* \endcond */

/*******************************************************************************
* Defines module variables
*******************************************************************************/

#if ((capsense_ENABLE == capsense_CSD_CSX_EN) || \
     (capsense_ENABLE == capsense_SELF_TEST_EN) || \
     (capsense_ENABLE == capsense_ADC_EN))
    capsense_SENSE_METHOD_ENUM capsense_currentSenseMethod = capsense_UNDEFINED_E;
#endif

#if(capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
    /* Module variable keep track of multi-frequency scan channel index */
    uint8 capsense_scanFreqIndex = 0u;
#else
    /* const allows C-compiler to do optimization */
    const uint8 capsense_scanFreqIndex = 0u;
#endif

/* Global software variables */
volatile uint8 capsense_widgetIndex = 0u;    /* Index of the scanning widget */
volatile uint8 capsense_sensorIndex = 0u;    /* Index of the scanning sensor */
uint8 capsense_requestScanAllWidget = 0u;

/* Pointer to RAM_SNS_STRUCT structure */
capsense_RAM_SNS_STRUCT *capsense_curRamSnsPtr;

#if ((capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) || \
     (capsense_ENABLE == capsense_CSX_EN))
    /* Pointer to Flash structure holding configuration of widget to be scanned */
    capsense_FLASH_WD_STRUCT const *capsense_curFlashWdgtPtr = 0u;
#endif

#if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
    /* Pointer to Flash structure holding info of sensor to be scanned */
    capsense_FLASH_SNS_STRUCT const *capsense_curFlashSnsPtr = 0u;
#endif

/* Pointer to Flash structure to hold Sns electrode that was connected previously */
capsense_FLASH_IO_STRUCT const *capsense_curSnsIOPtr;


/*******************************************************************************
* Function Name: capsense_IsBusy
****************************************************************************//**
*
* \brief
*  Returns the current status of the Component (Scan is completed or Scan is in
*  progress).
*
* \details
*  This function returns a status of the hardware block whether a scan is
*  currently in progress or not. If the Component is busy, no new scan or setup
*  widgets is made. The critical section (i.e. disable global interrupt)
*  is recommended for the application when the device transitions from
*  the active mode to sleep or deep sleep modes.
*
* \return
*  Returns the current status of the Component:
*    - capsense_NOT_BUSY - No scan is in progress and a next scan
*      can be initiated.
*    - capsense_SW_STS_BUSY - The previous scanning is not completed
*      and the hardware block is busy.
*
*******************************************************************************/
uint32 capsense_IsBusy(void)
{
    return (capsense_dsRam.status & capsense_SW_STS_BUSY);
}

/*******************************************************************************
* Function Name: capsense_SetupWidget
****************************************************************************//**
*
* \brief
*  Performs the initialization required to scan the specified widget.
*
* \details
*  This function prepares the Component to scan all the sensors in the specified
*  widget by executing the following tasks:
*    1. Re-initialize the hardware if it is not configured to perform the
*       sensing method used by the specified widget, this happens only if the
*       CSD and CSX methods are used in the Component.
*    2. Initialize the hardware with specific sensing configuration (e.g.
*       sensor clock, scan resolution) used by the widget.
*    3. Disconnect all previously connected electrodes, if the electrodes
*       connected by the capsense_CSDSetupWidgetExt(),
*       capsense_CSXSetupWidgetExt() or capsense_CSDConnectSns()
*       functions and not disconnected.
*
*  This function does not start sensor scanning, the capsense_Scan()
*  function must be called to start the scan sensors in the widget. If this
*  function is called more than once, it does not break the Component operation,
*  but only the last initialized widget is in effect.
*
* \param widgetId
*  Specifies the ID number of the widget to be initialized for scanning.
*  A macro for the widget ID can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_WDGT_ID.
*
* \return
*  Returns the status of the widget setting up operation:
*    - CY_RET_SUCCESS - The operation is successfully completed.
*    - CY_RET_BAD_PARAM - The widget is invalid or if the specified widget is
*      disabled
*    - CY_RET_INVALID_STATE - The previous scanning is not completed and the
*      hardware block is busy.
*    - CY_RET_UNKNOWN - An unknown sensing method is used by the widget or any
*      other spurious error occurred.
*
**********************************************************************************/
cy_status capsense_SetupWidget(uint32 widgetId)
{
    cy_status widgetStatus;

    if (capsense_WDGT_SW_STS_BUSY == (capsense_dsRam.status & capsense_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        widgetStatus = CY_RET_INVALID_STATE;
    }
    /*
     *  Check if widget id is valid, specified widget is enabled and widget did not
     *  detect any fault conditions if BIST is enabled. If all conditions are met,
     *  set widgetStatus as good, if not, set widgetStatus as bad.
     */
    else if ((capsense_TOTAL_WIDGETS > widgetId) &&
        (0uL != capsense_GET_WIDGET_EN_STATUS(widgetId)))

    {
        widgetStatus = CY_RET_SUCCESS;
    }
    else
    {
        widgetStatus = CY_RET_BAD_PARAM;
    }

    /*
     * Check widgetStatus flag that is set earlier, if flag is good, then set up only
     * widget
     */
    if (CY_RET_SUCCESS == widgetStatus)
    {
        #if (capsense_ENABLE == capsense_CSD_CSX_EN)
            /* Check widget sensing method is CSX and call CSX APIs */
            if (capsense_SENSE_METHOD_CSX_E ==
                capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[widgetId]))
            {
                /* Set up widget for CSX scan */
                capsense_CSXSetupWidget(widgetId);
            }
            /* Check widget sensing method is CSD and call appropriate API */
            else if (capsense_SENSE_METHOD_CSD_E ==
                     capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[widgetId]))
            {
                /* Set up widget for CSD scan */
                capsense_CSDSetupWidget(widgetId);
            }
            else
            {
                /* Sensing method is invalid, return error to caller */
                widgetStatus = CY_RET_UNKNOWN;
            }
        #elif (capsense_ENABLE == capsense_CSD_EN)
            /* Set up widget for scan */
            capsense_CSDSetupWidget(widgetId);
        #elif (capsense_ENABLE == capsense_CSX_EN)
            /* Set up widgets for scan */
            capsense_CSXSetupWidget(widgetId);
        #else
            widgetStatus = CY_RET_UNKNOWN;
            #error "No sensing method enabled, Component cannot work in this mode"
        #endif
    }

    return (widgetStatus);
}


/*******************************************************************************
* Function Name: capsense_Scan
****************************************************************************//**
*
* \brief
*  Initiates scanning of all the sensors in the widget initialized by
*  capsense_SetupWidget(), if no scan is in progress.
*
* \details
*  This function is called only after the capsense_SetupWidget()
*  function is called to start the scanning of the sensors in the widget. The
*  status of a sensor scan must be checked using the capsense_IsBusy()
*  API prior to starting a next scan or setting up another widget.
*
* \return
*  Returns the status of the scan initiation operation:
*    - CY_RET_SUCCESS - Scanning is successfully started.
*    - CY_RET_INVALID_STATE - The previous scanning is not completed and the
*      hardware block is busy.
*    - CY_RET_UNKNOWN - An unknown sensing method is used by the widget.
*
********************************************************************************/
cy_status capsense_Scan(void)
{
    cy_status scanStatus = CY_RET_SUCCESS;

    if (capsense_WDGT_SW_STS_BUSY == (capsense_dsRam.status & capsense_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        scanStatus = CY_RET_INVALID_STATE;
    }
    else
    {
        /* If both CSD and CSX are enabled, call scan API based on widget sensing method */
        #if (capsense_ENABLE == capsense_CSD_CSX_EN)
            /* Check widget sensing method and call appropriate APIs */
            switch (capsense_currentSenseMethod)
            {
                case capsense_SENSE_METHOD_CSX_E:
                    capsense_CSXScan();
                    break;

                case capsense_SENSE_METHOD_CSD_E:
                     capsense_CSDScan();
                    break;

                default:
                    scanStatus = CY_RET_UNKNOWN;
                    break;
            }

        /* If only CSD is enabled, call CSD scan */
        #elif (capsense_ENABLE == capsense_CSD_EN)
            capsense_CSDScan();

        /* If only CSX is enabled, call CSX scan */
        #elif (capsense_ENABLE == capsense_CSX_EN)
            capsense_CSXScan();

        #else
            scanStatus = CY_RET_UNKNOWN;
            #error "No sensing method enabled, Component cannot work in this mode"
        #endif /* (capsense_ENABLE == capsense_CSD_CSX_EN) */
    }

    return (scanStatus);
}


/*******************************************************************************
* Function Name: capsense_ScanAllWidgets
****************************************************************************//**
*
* \brief
*  Initializes the first enabled widget and scanning of all the sensors in the
*  widget, then the same process is repeated for all the widgets in the Component,
*  i.e. scanning of all the widgets in the Component.
*
* \details
*  This function initializes a widget and scans all the sensors in the widget,
*  and then repeats the same for all the widgets in the Component. The tasks of
*  the capsense_SetupWidget() and capsense_Scan() functions are
*  executed by these functions. The status of a sensor scan must be checked
*  using the capsense_IsBusy() API prior to starting a next scan
*  or setting up another widget.
*
* \return
*  Returns the status of the operation:
*    - CY_RET_SUCCESS - Scanning is successfully started.
*    - CY_RET_BAD_PARAM - All the widgets are disabled.
*    - CY_RET_INVALID_STATE - The previous scanning is not completed and the HW block is busy.
*    - CY_RET_UNKNOWN - There are unknown errors.
*
*******************************************************************************/
cy_status capsense_ScanAllWidgets(void)
{
    cy_status scanStatus = CY_RET_UNKNOWN;

    uint32 wdgtIndex;

    if (capsense_SW_STS_BUSY == (capsense_dsRam.status & capsense_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        scanStatus = CY_RET_INVALID_STATE;
    }
    else
    {
        /*
         *  Set up widget first widget.
         *  If widget returned error, set up next, continue same until widget does not return error.
         */
        for (wdgtIndex = 0u;
             wdgtIndex < capsense_TOTAL_WIDGETS;
             wdgtIndex++)
        {

            scanStatus = capsense_SetupWidget(wdgtIndex);

            if (CY_RET_SUCCESS == scanStatus)
            {
                #if (0u != (capsense_TOTAL_WIDGETS - 1u))
                    /* If there are more than one widget to be scanned, request callback to scan next widget */
                    if ((capsense_TOTAL_WIDGETS - 1u) > wdgtIndex)
                    {
                         /* Request callback to scan next widget in ISR */
                        capsense_requestScanAllWidget = capsense_ENABLE;
                    }
                    else
                    {
                        /* Request to exit in ISR (Do not scan the next widgets) */
                        capsense_requestScanAllWidget = capsense_DISABLE;
                    }
                #else
                    {
                        /* Request to exit in ISR (We have only one widget) */
                        capsense_requestScanAllWidget = capsense_DISABLE;
                    }
                #endif  /* (0u != (capsense_TOTAL_WIDGETS - 1u)) */

                /* Initiate scan and quit loop */
                scanStatus = capsense_Scan();

                break;
            }
        }
    }

    return (scanStatus);
}


/*******************************************************************************
* Function Name: capsense_SsInitialize
****************************************************************************//**
*
* \brief
*  Performs hardware and firmware initialization required for proper operation
*  of the capsense Component. This function is called from
*  the capsense_Start() API prior to calling any other APIs of the Component.
*
* \details
*  Performs hardware and firmware initialization required for proper operation
*  of the capsense Component. This function is called from
*  the capsense_Start() API prior to calling any other APIs of the Component.
*  1. Depending on the configuration, the function initializes the CSD block
*     for the corresponding sensing mode.
*  2. The function updates the dsRam.wdgtWorking variable with 1 when Self-test
*     is enabled.
*
*  Calling the capsense_Start API is the recommended method to initialize
*  the capsense Component at power-up. The capsense_SsInitialize()
*  API should not be used for initialization, resume, or wake-up operations.
*  The dsRam.wdgtWorking variable is updated.
*
* \return status
*  Returns status of operation:
*  - Zero        - Indicates successful initialization.
*  - Non-zero    - One or more errors occurred in the initialization process.
*
*******************************************************************************/
cy_status capsense_SsInitialize(void)
{
    cy_status initStatus = CY_RET_SUCCESS;

    #if((capsense_ENABLE == capsense_CSD_EN) ||\
        (capsense_ENABLE == capsense_CSX_EN))
        capsense_SsInitializeSourceSenseClk();
    #endif /* ((capsense_ENABLE == capsense_CSD_EN) ||\
               (capsense_ENABLE == capsense_CSX_EN)) */

    /* Set all IO states to default state */
    capsense_SsSetIOsInDefaultState();

    #if ((capsense_ENABLE == capsense_CSD_CSX_EN) || \
         (capsense_ENABLE == capsense_SELF_TEST_EN) || \
         (capsense_ENABLE == capsense_ADC_EN))
        /*
         * CSD hardware block is initialized in the Setup Widget based
         * on widget sensing method. Release previously captured HW resources
         * if it is second call of capsense_Start() function.
         */
        capsense_SsSwitchSensingMode(capsense_UNDEFINED_E);
    #elif (capsense_ENABLE == capsense_CSD_EN)
        /* Initialize CSD block for CSD scanning */
        capsense_SsCSDInitialize();

    #elif (capsense_ENABLE == capsense_CSX_EN)
        /* Initialize CSD block for CSX scanning */
        capsense_CSXInitialize();

    #else
        #error "No sensing method enabled, Component cannot work in this mode"
        initStatus = CY_RET_UNKNOWN;
    #endif /* (capsense_ENABLE == capsense_CSD_CSX_EN) */

    return (initStatus);
}


/*******************************************************************************
* Function Name: capsense_SetPinState
****************************************************************************//**
*
* \brief
*  Sets the state (drive mode and output state) of the port pin used by a sensor.
*  The possible states are GND, Shield, High-Z, Tx or Rx, Sensor. If the sensor
*  specified in the input parameter is a ganged sensor, then the state of all pins
*  associated with the ganged sensor is updated.
*
* \details
*  This function sets a specified state for a specified sensor element. For the
*  CSD widgets, sensor element is a sensor ID, for the CSX widgets, it is either
*  an Rx or Tx electrode ID. If the specified sensor is a ganged sensor, then
*  the specified state is set for all the electrodes belong to the sensor.
*  This function must not be called while the Component is in the busy state.
*
*  This function accepts the capsense_SHIELD and
*  capsense_SENSOR states as an input only if there is at least
*  one CSD widget. Similarly, this function accepts the capsense_TX_PIN
*  and capsense_RX_PIN states as an input only if there is at least
*  one CSX widget in the project.

*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the custom-specific
*  use cases. Functions that perform a setup and scan of a sensor/widget
*  automatically set the required pin states. They ignore changes
*  in the design made by the capsense_SetPinState() function.
*  This function neither check wdgtIndex nor sensorElement for the correctness.
*
* \param widgetId
*  Specifies the ID of the widget to change the pin state of the specified
*  sensor.
*  A macro for the widget ID can be found in the capsense Configuration
*  header file defined as capsense_<WidgetName>_WDGT_ID.
*
* \param sensorElement
*  Specifies the ID of the sensor element within the widget to change
*  its pin state.
*  For the CSD widgets, sensorElement is the sensor ID and can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_SNS<SensorNumber>_ID.
*  For the CSX widgets, sensorElement is defined either as Rx ID or Tx ID.
*  The first Rx in a widget corresponds to sensorElement = 0, the second
*  Rx in a widget corresponds to sensorElement = 1, and so on.
*  The last Tx in a widget corresponds to sensorElement = (RxNum + TxNum).
*  Macros for Rx and Tx IDs can be found in the
*  capsense Configuration header file defined as:
*  - capsense_<WidgetName>_RX<RXNumber>_ID
*  - capsense_<WidgetName>_TX<TXNumber>_ID.
*
* \param state
*  Specifies the state of the sensor to be set:
*     1. capsense_GROUND - The pin is connected to the ground.
*     2. capsense_HIGHZ - The drive mode of the pin is set to High-Z
*        Analog.
*     3. capsense_SHIELD - The shield signal is routed to the pin
*        (available only if CSD sensing method with shield electrode is enabled).
*     4. capsense_SENSOR - The pin is connected to the scanning bus
*        (available only if CSD sensing method is enabled).
*     5. capsense_TX_PIN - The Tx signal is routed to the sensor
*        (available only if CSX sensing method is enabled).
*     6. capsense_RX_PIN - The pin is connected to the scanning bus
*        (available only if CSX sensing method is enabled).
*
*******************************************************************************/
void capsense_SetPinState(uint32 widgetId, uint32 sensorElement, uint32 state)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    uint32 interruptState;
    capsense_FLASH_IO_STRUCT const *ioPtr;
    #if (capsense_ENABLE == capsense_GANGED_SNS_EN)
        capsense_FLASH_SNS_STRUCT const *curFlashSnsPtr;
    #endif

    /* Getting sensor element pointer and number of electrodes */
    #if (capsense_ENABLE == capsense_GANGED_SNS_EN)
        /* Check the ganged sns flag */
        if (capsense_GANGED_SNS_MASK == (capsense_dsFlash.wdgtArray[widgetId].staticConfig & capsense_GANGED_SNS_MASK))
        {
            curFlashSnsPtr = capsense_dsFlash.wdgtArray[widgetId].ptr2SnsFlash;
            curFlashSnsPtr += sensorElement;
            ioPtr = &capsense_ioList[curFlashSnsPtr->firstPinId];
            eltdNum = curFlashSnsPtr->numPins;
        }
        else
        {
            ioPtr = (capsense_FLASH_IO_STRUCT const *)capsense_dsFlash.wdgtArray[widgetId].ptr2SnsFlash + sensorElement;
            eltdNum = 1u;
        }
    #else
        ioPtr = (capsense_FLASH_IO_STRUCT const *)capsense_dsFlash.wdgtArray[widgetId].ptr2SnsFlash + sensorElement;
        eltdNum = 1u;
    #endif

    /* Loop through all electrodes of the specified sensor element */
    for (eltdIndex = 0u; eltdIndex < eltdNum; eltdIndex++)
    {
        /* Reset HSIOM and PC registers */
        interruptState = Cy_SysLib_EnterCriticalSection();
        Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, (en_hsiom_sel_t)capsense_HSIOM_SEL_GPIO);
        Cy_SysLib_ExitCriticalSection(interruptState);

        switch (state)
        {
        case capsense_GROUND:
            interruptState = Cy_SysLib_EnterCriticalSection();
            Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_STRONG_IN_OFF);
            Cy_GPIO_Clr((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber);
            Cy_SysLib_ExitCriticalSection(interruptState);
            break;

        case capsense_HIGHZ:
            interruptState = Cy_SysLib_EnterCriticalSection();
            Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_ANALOG);
            Cy_GPIO_Clr((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber);
            Cy_SysLib_ExitCriticalSection(interruptState);
            break;

        #if (capsense_ENABLE == capsense_CSD_EN)
            case capsense_SENSOR:
                /* Enable sensor */
                capsense_CSDConnectSns(ioPtr);
                break;

            #if (capsense_ENABLE == capsense_CSD_SHIELD_EN)
                case capsense_SHIELD:
                    interruptState = Cy_SysLib_EnterCriticalSection();
                    Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_STRONG_IN_OFF);
                    Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, (en_hsiom_sel_t)capsense_HSIOM_SEL_CSD_SHIELD);
                    Cy_SysLib_ExitCriticalSection(interruptState);
                    break;
            #endif /* (capsense_ENABLE == capsense_CSD_SHIELD_EN) */
        #endif /* (capsense_ENABLE == capsense_CSD_EN) */

        #if (capsense_ENABLE == capsense_CSX_EN)
            case capsense_TX_PIN:
                interruptState = Cy_SysLib_EnterCriticalSection();
                Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, (en_hsiom_sel_t)capsense_HSIOM_SEL_CSD_SHIELD);
                Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_STRONG_IN_OFF);
                Cy_SysLib_ExitCriticalSection(interruptState);
                break;

            case capsense_RX_PIN:
                interruptState = Cy_SysLib_EnterCriticalSection();
                Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, (en_hsiom_sel_t)capsense_HSIOM_SEL_AMUXA);
                Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_ANALOG);
                Cy_SysLib_ExitCriticalSection(interruptState);
                break;
        #endif /* (capsense_ENABLE == capsense_CSX_EN) */

        default:
            /* Wrong state */
            break;
        }

        ioPtr++;
    }
}


#if ((capsense_ENABLE == capsense_CSD_CSX_EN) || \
     (capsense_ENABLE == capsense_SELF_TEST_EN) || \
     (capsense_ENABLE == capsense_ADC_EN))

    #if (capsense_ENABLE == capsense_CSD_EN)
        /*******************************************************************************
        * Function Name: capsense_SsCSDDisableMode
        ****************************************************************************//**
        *
        * \brief
        *  This function disables CSD mode.
        *
        * \details
        *  To disable CSD mode the following tasks are performed:
        *  1. Disconnect Cmod from AMUXBUS-A.
        *  2. Disconnect previous CSX electrode if it has been connected.
        *  3. Disconnect Csh from AMUXBUS-B.
        *  4. Disable Shield Electrodes.
        *
        *******************************************************************************/
        static void capsense_SsCSDDisableMode(void)
        {
            uint32 newRegValue = 0uL;

            /* To remove unreferenced local variable warning */
            (void)newRegValue;
            Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)capsense_CSD_CMOD_PORT_PTR, capsense_CSD_CMOD_PIN,
                                                                             (en_hsiom_sel_t)capsense_HSIOM_SEL_GPIO);

            #if ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
                 (capsense_ENABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN))
                /* Disconnect IDACA and IDACB */
                newRegValue = CY_GET_REG32(capsense_CSD_SW_REFGEN_SEL_PTR);
                newRegValue &= (uint32)(~capsense_CSD_SW_REFGEN_SEL_SW_IAIB_MSK);
                CY_SET_REG32(capsense_CSD_SW_REFGEN_SEL_PTR, newRegValue);
            #endif /* ((capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) && \
                       (capsense_ENABLE == capsense_CSD_DEDICATED_IDAC_COMP_EN)) */

            /* Disconnect previous CSD electrode if it has been connected */
            capsense_SsCSDElectrodeCheck();

            /* Disconnect Csh from AMUXBUS-B */
            #if ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
                 (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN))

                Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)capsense_CSD_CTANK_PORT_PTR, capsense_CSD_CTANK_PIN,
                                                           (en_hsiom_sel_t)capsense_HSIOM_SEL_GPIO);

            #endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
                       (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN)) */

            #if ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
                 (0u != capsense_CSD_TOTAL_SHIELD_COUNT))
                capsense_SsCSDDisableShieldElectrodes();
            #endif /* ((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
                       (0u != capsense_CSD_TOTAL_SHIELD_COUNT)) */
        }
    #endif /* (capsense_ENABLE == capsense_CSD_EN) */


    #if (capsense_ENABLE == capsense_CSX_EN)
        /*******************************************************************************
        * Function Name: capsense_SsDisableCSXMode
        ****************************************************************************//**
        *
        * \brief
        *  This function disables CSX mode.
        *
        * \details
        *  To disable CSX mode the following tasks are performed:
        *  1. Disconnect CintA and CintB from AMUXBUS-A.
        *  2. Disconnect previous CSX electrode if it has been connected.
        *
        *******************************************************************************/
        static void capsense_SsDisableCSXMode(void)
        {
            /* Disconnect previous CSX electrode if it has been connected */
            capsense_CSXElectrodeCheck();
        }
    #endif /* (capsense_ENABLE == capsense_CSX_EN) */


    /*******************************************************************************
    * Function Name: capsense_SsSwitchSensingMode
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the mode for case when both CSD and CSX widgets are
    *  scanned.
    *
    * \details
    *  To switch to the CSD mode the following tasks are performed:
    *  1. Disconnect CintA and CintB from AMUXBUS-A.
    *  2. Disconnect previous CSD electrode if it has been connected.
    *  3. Initialize CSD mode.
    *
    *  To switch to the CSX mode the following tasks are performed:
    *  1. Disconnect Cmod from AMUXBUS-A.
    *  2. Disconnect previous CSX electrode if it has been connected.
    *  3. Initialize CSX mode.
    *
    * \param mode
    *  Specifies the scan mode:
    *  - capsense_SENSE_METHOD_CSD_E
    *  - capsense_SENSE_METHOD_CSX_E
    *  - capsense_SENSE_METHOD_BIST_E
    *  - capsense_UNDEFINED_E
    *
    *******************************************************************************/
    void capsense_SsSwitchSensingMode(capsense_SENSE_METHOD_ENUM mode)
    {
        if (capsense_currentSenseMethod != mode)
        {
            /* The requested mode differes to the current one. Disable the current mode */
            if (capsense_SENSE_METHOD_CSD_E == capsense_currentSenseMethod)
            {
                #if (capsense_ENABLE == capsense_CSD_EN)
                    capsense_SsCSDDisableMode();
                #endif /* (capsense_ENABLE == capsense_CSD_EN) */
            }
            else if (capsense_SENSE_METHOD_CSX_E == capsense_currentSenseMethod)
            {
                #if (capsense_ENABLE == capsense_CSX_EN)
                    capsense_SsDisableCSXMode();
                #endif /* (capsense_ENABLE == capsense_CSX_EN) */
            }
            else if (capsense_SENSE_METHOD_BIST_E == capsense_currentSenseMethod)
            {
                #if (capsense_ENABLE == capsense_SELF_TEST_EN)
                    capsense_BistDisableMode();
                #endif /* (capsense_ENABLE == capsense_SELF_TEST_EN) */
            }
            else
            {
                #if (capsense_ENABLE == capsense_ADC_EN)
                    /* Release ADC resources */
                    (void)capsense_AdcReleaseResources();
                #endif /* (capsense_ENABLE == capsense_ADC_EN) */
            }

            /* Enable the specified mode */
            if (capsense_SENSE_METHOD_CSD_E == mode)
            {
                #if (capsense_ENABLE == capsense_CSD_EN)
                    /* Initialize CSD mode to guarantee configured CSD mode */
                    capsense_SsCSDInitialize();
                    capsense_currentSenseMethod = capsense_SENSE_METHOD_CSD_E;
                #endif /* (capsense_ENABLE == capsense_CSD_EN) */
            }
            else if (capsense_SENSE_METHOD_CSX_E == mode)
            {
                #if (capsense_ENABLE == capsense_CSX_EN)
                    /* Initialize CSX mode to guarantee configured CSX mode */
                    capsense_CSXInitialize();
                    capsense_currentSenseMethod = capsense_SENSE_METHOD_CSX_E;
                #endif /* (capsense_ENABLE == capsense_CSX_EN) */
            }
            else if (capsense_SENSE_METHOD_BIST_E == mode)
            {
                #if (capsense_ENABLE == capsense_SELF_TEST_EN)
                    capsense_BistInitialize();
                    capsense_currentSenseMethod = capsense_SENSE_METHOD_BIST_E;
                #endif /* (capsense_ENABLE == capsense_SELF_TEST_EN) */
            }
            else
            {
                capsense_currentSenseMethod = capsense_UNDEFINED_E;
            }
        }
    }
#endif  /* ((capsense_ENABLE == capsense_CSD_CSX_EN) || \
            (capsense_ENABLE == capsense_ADC_EN)) */


/*******************************************************************************
* Function Name: capsense_SsSetIOsInDefaultState
****************************************************************************//**
*
* \brief
*  Sets all electrodes into a default state.
*
* \details
*  Sets all the CSD/CSX IOs into a default state:
*  - HSIOM   - Disconnected, the GPIO mode.
*  - DM      - Strong drive.
*  - State   - Zero.
*
*  Sets all the ADC channels into a default state:
*  - HSIOM   - Disconnected, the GPIO mode.
*  - DM      - HiZ-Analog.
*  - State   - Zero.
*
*  It is not recommended to call this function directly from the application
*  layer.
*
*******************************************************************************/
void capsense_SsSetIOsInDefaultState(void)
{
    capsense_FLASH_IO_STRUCT const *ioPtr = &capsense_ioList[0u];
    uint32 loopIndex;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < capsense_TOTAL_ELECTRODES; loopIndex++)
    {
        /*
        *   1. Disconnect HSIOM
        *   2. Set strong DM
        *   3. Set pin state to logic 0
        */
        Cy_GPIO_Pin_FastInit((GPIO_PRT_Type*)ioPtr->pcPtr, (uint32)ioPtr->pinNumber, CY_GPIO_DM_STRONG, 0u,
                                                        (en_hsiom_sel_t)capsense_HSIOM_SEL_GPIO);

        /* Get next electrode */
        ioPtr++;
    }

    #if(capsense_ENABLE == capsense_ADC_EN)
        capsense_ClearAdcChannels();
    #endif /* (capsense_ENABLE == capsense_ADC_EN) */
}

#if (capsense_ENABLE == capsense_ADC_EN)
/*******************************************************************************
* Function Name: capsense_SsReleaseResources()
****************************************************************************//**
*
* \brief
*  This function sets the resources that do not belong to the sensing HW block to
*  default state.
*
* \details
*  The function performs following tasks:
*  1. Checks if CSD block busy and returns error if it is busy
*  2. Disconnects integration capacitors (CintA, CintB for CSX mode and
*     Cmod for CSD mode)
*  3. Disconnect electroded if they have been connected.
*
* \return
*  Returns the status of the operation:
*  - Zero        - Resources released successfully.
*  - Non-zero    - One or more errors occurred in releasing process.
*
*******************************************************************************/
cy_status capsense_SsReleaseResources(void)
{
    cy_status busyStatus = CY_RET_SUCCESS;

    if (capsense_SW_STS_BUSY == (capsense_dsRam.status & capsense_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        busyStatus = CY_RET_INVALID_STATE;
    }
    else
    {
        #if (capsense_ENABLE == capsense_CSX_EN)
            capsense_SsDisableCSXMode();
        #endif /* (capsense_ENABLE == capsense_CSX_EN) */

        #if (capsense_ENABLE == capsense_CSD_EN)
            capsense_SsCSDDisableMode();
        #endif /* (capsense_ENABLE == capsense_CSD_EN) */

        #if (capsense_ENABLE == capsense_SELF_TEST_EN)
            capsense_BistDisableMode();
        #endif /* (capsense_ENABLE == capsense_SELF_TEST_EN) */

        #if ((capsense_ENABLE == capsense_CSD_EN) && \
             (capsense_ENABLE == capsense_CSD_SHIELD_EN) &&  \
             (capsense_SNS_CONNECTION_SHIELD == capsense_CSD_INACTIVE_SNS_CONNECTION))
            capsense_SsSetIOsInDefaultState();
        #endif /* ((capsense_ENABLE == capsense_CSD_EN) && \
             (capsense_DISABLE != capsense_CSD_SHIELD_EN) &&  \
             (capsense_SNS_CONNECTION_SHIELD == capsense_CSD_INACTIVE_SNS_CONNECTION)) */

        /*
         * Reset of the currentSenseMethod variable to make sure that the next
         * call of SetupWidget() API setups the correct widget mode
         */
        capsense_currentSenseMethod = capsense_UNDEFINED_E;
    }

    return busyStatus;
}
#endif /* (capsense_ENABLE == capsense_ADC_EN) */


/*******************************************************************************
* Function Name: capsense_SsPostAllWidgetsScan
****************************************************************************//**
*
* \brief
*  The ISR function for multiple widget scanning implementation.
*
* \details
*  This is the function used by the capsense ISR to implement multiple widget
*  scanning.
*  Should not be used by the application layer.
*
*******************************************************************************/
void capsense_SsPostAllWidgetsScan(void)
{
    /*
    *   1. Increment widget index
    *   2. Check if all the widgets are scanned
    *   3. If all the widgets are not scanned, set up and scan next widget
    */
    #if (1u != capsense_TOTAL_WIDGETS)
        cy_status postScanStatus;

        do
        {
            capsense_widgetIndex++;

            postScanStatus = capsense_SetupWidget((uint32)capsense_widgetIndex);

            if (CY_RET_SUCCESS == postScanStatus)
            {
                if((capsense_TOTAL_WIDGETS - 1u) == capsense_widgetIndex)
                {
                    /* The last widget will be scanned. Reset flag to skip configuring the next widget setup in ISR. */
                    capsense_requestScanAllWidget = capsense_DISABLE;
                }
                (void)capsense_Scan();
            }
            else if((capsense_TOTAL_WIDGETS - 1u) == capsense_widgetIndex)
            {
                #if ((capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN) && \
                     (capsense_ENABLE == capsense_CSD_EN))
                    if (capsense_SENSE_METHOD_CSD_E ==
                        capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[capsense_widgetIndex]))
                    {
                        /* Disable the CSD block */
                        CY_SET_REG32(capsense_CSD_CONFIG_PTR, capsense_configCsd);
                    }
                #endif /* ((capsense_ENABLE == capsense_BLOCK_OFF_AFTER_SCAN_EN) && \
                           (capsense_ENABLE == capsense_CSD_EN)) */

                /* Update scan Counter */
                capsense_dsRam.scanCounter++;
                /* all the widgets are totally processed. Reset BUSY flag */
                capsense_dsRam.status &= ~capsense_SW_STS_BUSY;

                /* Update status with with the failure */
                capsense_dsRam.status &= ~capsense_STATUS_ERR_MASK;
                capsense_dsRam.status |= ((postScanStatus & capsense_STATUS_ERR_SIZE) << capsense_STATUS_ERR_SHIFT);

                /* Set postScanStatus to exit the while loop */
                postScanStatus = CY_RET_SUCCESS;
            }
            else
            {
                /* Update status with with the failure. Configure the next widget in while() loop */
                capsense_dsRam.status &= ~capsense_STATUS_ERR_MASK;
                capsense_dsRam.status |= ((postScanStatus & capsense_STATUS_ERR_SIZE) << capsense_STATUS_ERR_SHIFT);
            }
        } while (CY_RET_SUCCESS != postScanStatus);
    #endif /* (1u != capsense_TOTAL_WIDGETS) */
}


/*******************************************************************************
* Function Name: capsense_SsIsrInitialize
****************************************************************************//**
*
* \brief
*  Enables and initializes for the function pointer for a callback for the ISR.
*
* \details
*  The "address" is a special type cy_israddress defined by syslib. This function
*  is used by Component APIs and should not be used by an application program for
*  proper working of the Component.
*
* \param address
*  The address of the function to be called when interrupt is fired.
*
*******************************************************************************/
void capsense_SsIsrInitialize(cy_israddress address)
{
    /* Disable interrupt */
    #if defined(capsense_ISR__INTC_ASSIGNED)
        NVIC_DisableIRQ(capsense_ISR_cfg.intrSrc);
    #endif

    /* Configure interrupt with priority and vector */
    #if defined(capsense_ISR__INTC_ASSIGNED)
        (void)Cy_SysInt_Init(&capsense_ISR_cfg, address);
    #endif

    /* Enable interrupt */
    #if defined(capsense_ISR__INTC_ASSIGNED)
        NVIC_EnableIRQ(capsense_ISR_cfg.intrSrc);
    #endif
}


/*******************************************************************************
* Function Name: capsense_SsSetSnsFirstPhaseWidth
****************************************************************************//**
*
* \brief
*  Defines the length of the first phase of the sense clock in clk_csd cycles.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*  At all times it must be assured that the phases are at least 2 clk_csd cycles
*  (1 for non overlap, if used), if this rule is violated the result is undefined.
*
* \param
*  snsClk The divider value for the sense clock.
*
*******************************************************************************/
void capsense_SsSetSnsFirstPhaseWidth(uint32 phaseWidth)
{
    uint32 newRegValue;

    newRegValue = CY_GET_REG32(capsense_CSD_SENSE_DUTY_PTR);
    newRegValue &= (uint32)(~capsense_CSD_SENSE_DUTY_SENSE_WIDTH_MSK);
    newRegValue |= phaseWidth;
    CY_SET_REG32(capsense_CSD_SENSE_DUTY_PTR, newRegValue);
}


/*******************************************************************************
* Function Name: capsense_SsSetSnsClockDivider
****************************************************************************//**
*
* \brief
*  Sets the divider values for the sense clock and then starts
*  the sense clock.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*
* \param
*  snsClk The divider value for the sense clock.
*
*******************************************************************************/
void capsense_SsSetSnsClockDivider(uint32 snsClk)
{
    uint32 newRegValue;

    /*
     * Set divider value for sense clock.
     * 1u is subtracted from snsClk because SENSE_DIV value 0 corresponds
     * to dividing by 1.
     */
    newRegValue = CY_GET_REG32(capsense_CSD_SENSE_PERIOD_PTR);
    newRegValue &= (uint32)(~capsense_CSD_SENSE_PERIOD_SENSE_DIV_MSK);
    newRegValue |= snsClk - 1u;
    CY_SET_REG32(capsense_CSD_SENSE_PERIOD_PTR, newRegValue);
}


/*******************************************************************************
* Function Name: capsense_SsSetClockDividers
****************************************************************************//**
*
* \brief
*  Sets the divider values for sense and modulator clocks and then starts
*  a modulator clock-phase aligned to HFCLK and sense clock-phase aligned to
*  the modulator clock.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*
* \param
*  snsClk The divider value for the sense clock.
*  modClk The divider value for the modulator clock.
*
*******************************************************************************/
void capsense_SsSetClockDividers(uint32 snsClk, uint32 modClk)
{
    /* Configure Mod clock */
    capsense_ModClk_SetDivider((uint32)modClk - 1uL);

    /* Configure Sns clock */
    capsense_SsSetSnsClockDivider(snsClk);
}


#if ((capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN) || \
     (capsense_ENABLE == capsense_CSX_IDAC_AUTOCAL_EN))
    /*******************************************************************************
    * Function Name: capsense_CalibrateWidget
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all the sensors in the specified widget to the default
    *  target, this function detects the sensing method used by the
    *  widget prior to calibration.
    *
    * \details
    *  This function performs exactly the same tasks as
    *  capsense_CalibrateAllWidgets, but only for a specified widget.
    *  This function detects the sensing method used by the widgets and uses
    *  the Enable compensation IDAC parameter.
    *
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.

    *
    * \param widgetId
    *  Specifies the ID number of the widget to calibrate its raw count.
    *  A macro for the widget ID can be found in the
    *  capsense Configuration header file defined as
    *  capsense_<WidgetName>_WDGT_ID.
    *
    * \return
    *  Returns the status of the specified widget calibration:
    *  - CY_RET_SUCCESS - The operation is successfully completed.
    *  - CY_RET_BAD_PARAM - The input parameter is invalid.
    *  - CY_RET_BAD_DATA - The calibration failed and the Component may not
    *    operate as expected.
    *
    *******************************************************************************/
    cy_status capsense_CalibrateWidget(uint32 widgetId)
    {
        cy_status calibrateStatus = CY_RET_SUCCESS;

        do
        {
            if (capsense_TOTAL_WIDGETS <= widgetId)
            {
                calibrateStatus = CY_RET_BAD_PARAM;
            }

            /*
             *  Check if widget id is valid, specified widget did not
             *  detect any faults conditions if BIST is enabled.
             */
            #if (capsense_ENABLE == capsense_SELF_TEST_EN)
                if (0u != capsense_GET_WIDGET_EN_STATUS(widgetId))
                {
                    calibrateStatus = CY_RET_SUCCESS;
                }
                else
                {
                    calibrateStatus = CY_RET_INVALID_STATE;
                }
            #endif  /* (capsense_ENABLE == capsense_SELF_TEST_EN) */

            if (CY_RET_SUCCESS != calibrateStatus)
            {
                /* Exit from the loop because of a fail */
                break;
            }

            /* If both CSD and CSX are enabled, calibrate widget using sensing method */
            #if (capsense_ENABLE == capsense_CSD_CSX_EN)
                /* Check widget sensing method and call appropriate APIs */
                #if (capsense_ENABLE == capsense_CSX_IDAC_AUTOCAL_EN)
                    if (capsense_SENSE_METHOD_CSX_E ==
                        capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[widgetId]))
                    {
                        /* Calibrate CSX widget */
                       capsense_CSXCalibrateWidget(widgetId, capsense_CSX_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (capsense_ENABLE == capsense_CSX_IDAC_AUTOCAL_EN) */

                #if (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN)
                    if (capsense_SENSE_METHOD_CSD_E ==
                        capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[widgetId]))
                    {
                        /* Calibrate CSD widget */
                        calibrateStatus = capsense_CSDCalibrateWidget(widgetId, capsense_CSD_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN) */

            /* If only CSD is enabled, calibrate CSD sensor */
            #elif (capsense_ENABLE == capsense_CSD_EN)
                calibrateStatus = capsense_CSDCalibrateWidget(widgetId, capsense_CSD_RAWCOUNT_CAL_LEVEL);

            /* If only CSX is enabled, call CSX scan */
            #elif (capsense_ENABLE == capsense_CSX_EN)
                capsense_CSXCalibrateWidget(widgetId, capsense_CSX_RAWCOUNT_CAL_LEVEL);

            #else
                calibrateStatus = CY_RET_UNKNOWN;
            #endif /* (capsense_ENABLE == capsense_CSD_CSX_EN) */

             /* Update CRC */
            #if (capsense_ENABLE ==capsense_TST_WDGT_CRC_EN)
                capsense_DsUpdateWidgetCrc(widgetId);
            #endif /* (capsense_ENABLE ==capsense_TST_WDGT_CRC_EN) */

        } while (0u);

        return calibrateStatus;
    }


    /*******************************************************************************
    * Function Name: capsense_CalibrateAllWidgets
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all the widgets in the Component to the default
    *  target, this function detects the sensing method used by the widgets
    *  prior to calibration.
    *
    * \details
    *  Calibrates the IDACs for all the widgets in the Component to the default
    *  target value. This function detects the sensing method used by the widgets
    *  and regards the Enable compensation IDAC parameter.
    *
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.
    *
    * \return
    *  Returns the status of the calibration process:
    *  - CY_RET_SUCCESS - The operation is successfully completed.
    *  - CY_RET_BAD_DATA - The calibration failed and the Component may not
    *    operate as expected.
    *
    *******************************************************************************/
    cy_status capsense_CalibrateAllWidgets(void)
    {
        cy_status calibrateStatus = CY_RET_SUCCESS;
        uint32 wdgtIndex;

        for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
        {
            calibrateStatus |= capsense_CalibrateWidget(wdgtIndex);
        }

        return calibrateStatus;
    }
#endif /* ((capsense_ENABLE == capsense_CSD_IDAC_AUTOCAL_EN) ||
           (capsense_ENABLE == capsense_CSX_IDAC_AUTOCAL_EN)) */


#if (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE)
    /*******************************************************************************
    * Function Name: capsense_SsAutoTune
    ****************************************************************************//**
    *
    * \brief
    *  This API performs the parameters auto-tuning for the optimal capsense operation.
    *
    * \details
    *  This API performs the following:
    *  - Calibrates Modulator and Compensation IDACs.
    *  - Tunes the Sense Clock optimal value to get a Sense Clock period greater than
    *    2*5*R*Cp.
    *  - Calculates the resolution for the optimal finger capacitance.
    *
    * \return
    *  Returns the status of the operation:
    *  - Zero     - All the widgets are auto-tuned successfully.
    *  - Non-zero - Auto-tuning failed for any widget.
    *
    *******************************************************************************/
    cy_status capsense_SsAutoTune(void)
    {
        cy_status autoTuneStatus = CY_RET_SUCCESS;
        uint32 wdgtIndex;

        uint32 cp = 0uL;
        #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
            uint32 cpRow = 0uL;
        #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */

        uint32 cpWidget[capsense_TOTAL_WIDGETS];
        capsense_RAM_WD_BASE_STRUCT *ptrWdgt;
        AUTO_TUNE_CONFIG_TYPE autoTuneConfig;

        /* Configure common config variables */
        autoTuneConfig.snsClkConstantR = capsense_CSD_SNSCLK_R_CONST;
        autoTuneConfig.vRef = capsense_CSD_VREF_MV;
        autoTuneConfig.iDacGain = capsense_CSD_IDAC_GAIN_VALUE_NA;

        /* Calculate snsClk Input Clock in KHz */
        /* Dividers are chained */
        autoTuneConfig.snsClkInputClock = (CYDEV_CLK_PERICLK__KHZ / capsense_dsRam.modCsdClk);

        /* If both CSD and CSX are enabled, calibrate widget using sensing method */
        #if (capsense_ENABLE == capsense_CSD_CSX_EN)
            /* Initialize CSD mode */
            capsense_SsCSDInitialize();
        #endif /* (capsense_ENABLE == capsense_CSD_CSX_EN) */

        /*
        * Autotune phase #1:
        * - performing the first calibration at fixed settings
        * - getting sensor Cp
        * - getting sense clock frequency based on Cp
        */

        /* Tune sense clock for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (capsense_SENSE_METHOD_CSD_E ==
                capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                          capsense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                /* Set calibration resolution to 12 bits */
                ptrWdgt->resolution = capsense_CALIBRATION_RESOLUTION;

                /* Set clock source direct and sense clock frequency to 1.5 MHz */
                ptrWdgt->snsClkSource = (uint8)capsense_CLK_SOURCE_DIRECT;
                ptrWdgt->snsClk = (uint16)((uint32)autoTuneConfig.snsClkInputClock / capsense_CALIBRATION_FREQ_KHZ);
                #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        ptrWdgt->rowSnsClkSource = (uint8)capsense_CLK_SOURCE_DIRECT;
                        ptrWdgt->rowSnsClk = (uint16)((uint32)autoTuneConfig.snsClkInputClock / capsense_CALIBRATION_FREQ_KHZ);
                    }
                #endif

                /* Calibrate CSD widget to the default calibration target */
                (void)capsense_CSDCalibrateWidget(wdgtIndex, capsense_CSD_AUTOTUNE_CAL_LEVEL);

                #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        /* Get pointer to Sense Clock Divider for columns */
                        autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                        /* Get IDAC */
                        autoTuneConfig.iDac = capsense_calibratedIdacRow;

                        /* Calculate achived calibration level */
                        autoTuneConfig.calTarget = (uint16)(((uint32)capsense_calibratedRawcountRow * capsense_CSD_AUTOTUNE_CAL_UNITS) /
                                ((uint32)(0x01uL << capsense_CALIBRATION_RESOLUTION) - 1u));

                        /* Find correct sense clock value */
                        cpRow = SmartSense_TunePrescalers(&autoTuneConfig);

                        if ((capsense_CP_MAX + capsense_CP_ERROR) <= cpRow)
                        {
                            autoTuneStatus = CY_RET_BAD_DATA;
                        }

                        /*
                        * Multiply the sense Clock Divider by 2 while the desired Sense Clock Frequency is greater
                        * than maximum supported Sense Clock Frequency.
                        */
                        while((((uint32)autoTuneConfig.snsClkInputClock) > ((uint32)ptrWdgt->snsClk * capsense_CSD_SNS_FREQ_KHZ_MAX)) ||
                                (capsense_MIN_SNS_CLK_DIVIDER > ptrWdgt->snsClk))
                        {
                            ptrWdgt->snsClk <<= 1u;
                        }
                    }
                #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */

                /* Get pointer to Sense Clock Divider for columns */
                autoTuneConfig.ptrSenseClk = &ptrWdgt->snsClk;

                /* Get IDAC */
                autoTuneConfig.iDac = capsense_calibratedIdac;

                /* Calculate achived calibration level */
                autoTuneConfig.calTarget = (uint16)(((uint32)capsense_calibratedRawcount * capsense_CSD_AUTOTUNE_CAL_UNITS) /
                        ((uint32)(0x01uL << capsense_CALIBRATION_RESOLUTION) - 1u));

                /* Find correct sense clock value */
                cp = SmartSense_TunePrescalers(&autoTuneConfig);

                if ((capsense_CP_MAX + capsense_CP_ERROR) <= cp)
                {
                    autoTuneStatus = CY_RET_BAD_DATA;
                }

                /*
                * Multiply the sense Clock Divider by 2 while the desired Sense Clock Frequency is greater
                * than MAX supported Sense Clock Frequency.
                */
                while((((uint32)autoTuneConfig.snsClkInputClock) > ((uint32)ptrWdgt->snsClk * capsense_CSD_SNS_FREQ_KHZ_MAX)) ||
                        (capsense_MIN_SNS_CLK_DIVIDER > ptrWdgt->snsClk))
                {
                    ptrWdgt->snsClk <<= 1u;
                }

                cpWidget[wdgtIndex] = cp;

                #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        if (cpRow > cp)
                        {
                            cpWidget[wdgtIndex] = cpRow;
                        }
                    }
                #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */
            }
            else
            {
                #if (capsense_ENABLE == capsense_CSX_EN)
                    /* Non-CSD widget */
                    cpWidget[wdgtIndex] = 1u;
                #endif /* (capsense_ENABLE == capsense_CSX_EN) */
            }
        }

        /*
        * Autotune phase #2:
        * - repeating calibration with new sense clock frequency
        * - getting resolution
        */

        /* Tune resolution for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (capsense_SENSE_METHOD_CSD_E ==
                capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                          capsense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                /* Calibrate CSD widget to the default calibration target */
                autoTuneStatus |= capsense_CSDCalibrateWidget(wdgtIndex, capsense_CSD_AUTOTUNE_CAL_LEVEL);

                /* Get pointer to Sense Clock Divider (column or row) */
                autoTuneConfig.ptrSenseClk = &ptrWdgt->snsClk;

                /* Set parasitic capacitance for columns */
                autoTuneConfig.sensorCap = cpWidget[wdgtIndex];

                /* Get IDAC */
                autoTuneConfig.iDac = capsense_calibratedIdac;

                #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType))

                    {
                        /* Set the minimum sense clock frequency to calculate the resolution */
                        if (ptrWdgt->snsClk < ptrWdgt->rowSnsClk)
                        {
                            /* Rewrite pointer to Sense Clock Divider for rows */
                            autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                            /* Set parasitic capacitance for rows */
                            autoTuneConfig.sensorCap = cpWidget[wdgtIndex];

                            /* Get IDAC */
                            autoTuneConfig.iDac = capsense_calibratedIdacRow;
                        }
                    }
                #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */

                /* Get finger capacitance */
                autoTuneConfig.fingerCap = ptrWdgt->fingerCap;

                /* Init pointer to sigPFC */
                autoTuneConfig.sigPFC = &ptrWdgt->sigPFC;

                /* Find resolution */
                ptrWdgt->resolution = SmartSense_TuneSensitivity(&autoTuneConfig);
            }
        }

        /*
        * Autotune phase #3:
        * - selecting a widget clock source if AUTO
        * - repeating calibration with found clock frequency, resolution and clock source
        * - updating sensitivity
        */

        /* Tune sensitivity for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (capsense_SENSE_METHOD_CSD_E ==
                capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                          capsense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                capsense_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);

                /* Calibrate CSD widget to the default calibration target */
                autoTuneStatus |= capsense_CSDCalibrateWidget(wdgtIndex, capsense_CSD_AUTOTUNE_CAL_LEVEL);

                #if (capsense_ENABLE == capsense_TST_WDGT_CRC_EN)
                    capsense_DsUpdateWidgetCrc(wdgtIndex);
                #endif /* (capsense_ENABLE == capsense_TST_WDGT_CRC_EN) */
            }
        }

        return autoTuneStatus;
    }
#endif /* (capsense_CSD_SS_DIS != capsense_CSD_AUTOTUNE)) */


#if (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: capsense_SsChangeClkFreq
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the sensor clock frequency by configuring
    *  the corresponding divider.
    *
    * \details
    *  This function changes the sensor clock frequency by configuring
    *  the corresponding divider.
    *
    * \param chId
    *  The frequency channel ID.
    *
    *******************************************************************************/
    void capsense_SsChangeClkFreq(uint32 chId)
    {
        uint32 snsClkDivider;
        uint32 freqOffset1 = 0u;
        uint32 freqOffset2 = 0u;

        #if (0u != capsense_TOTAL_CSD_WIDGETS)
            uint32 conversionsNum;

            #if((capsense_CLK_SOURCE_PRS8  == capsense_CSD_SNS_CLK_SOURCE) ||\
                (capsense_CLK_SOURCE_PRS12 == capsense_CSD_SNS_CLK_SOURCE) ||\
                (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE))
                uint32 snsClkSrc;
            #endif
        #endif

        #if ((0u != capsense_TOTAL_CSD_WIDGETS) || \
             ((capsense_DISABLE == capsense_CSX_COMMON_TX_CLK_EN) && (0u != capsense_TOTAL_CSX_WIDGETS)))
            capsense_FLASH_WD_STRUCT const *ptrFlashWdgt = &capsense_dsFlash.wdgtArray[capsense_widgetIndex];
            capsense_RAM_WD_BASE_STRUCT const *ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)ptrFlashWdgt->ptr2WdgtRam;
        #endif

        switch(capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[capsense_widgetIndex]))
        {
        #if (0u != capsense_TOTAL_CSD_WIDGETS)
            case capsense_SENSE_METHOD_CSD_E:
                /* Get sensor clock divider value */
                #if (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN)
                    snsClkDivider = capsense_dsRam.snsCsdClk;
                #else /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */
                    #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                        /* Get SnsClck divider for rows or columns */
                        if (capsense_dsFlash.wdgtArray[capsense_widgetIndex].numCols <= capsense_sensorIndex)
                        {
                            snsClkDivider = ptrWdgt->rowSnsClk;
                        }
                        else
                        {
                            snsClkDivider = ptrWdgt->snsClk;
                        }
                    #else
                        snsClkDivider = ptrWdgt->snsClk;
                    #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */
                #endif /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */

                freqOffset1 = capsense_CSD_MFS_DIVIDER_OFFSET_F1;
                freqOffset2 = capsense_CSD_MFS_DIVIDER_OFFSET_F2;

                #if((capsense_CLK_SOURCE_PRS8  == capsense_CSD_SNS_CLK_SOURCE) ||\
                    (capsense_CLK_SOURCE_PRS12 == capsense_CSD_SNS_CLK_SOURCE) ||\
                    (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE))
                    /* Get sense clk source */
                    #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
                        /* Get SnsClc Source for rows or columns */
                        if (capsense_dsFlash.wdgtArray[capsense_widgetIndex].numCols <= capsense_sensorIndex)
                        {
                            snsClkSrc = (uint32)ptrWdgt->rowSnsClkSource;
                        }
                        else
                        {
                            snsClkSrc = (uint32)ptrWdgt->snsClkSource;
                        }
                    #else
                        snsClkSrc = (uint32)ptrWdgt->snsClkSource;
                    #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */

                    switch (snsClkSrc)
                    {
                    case capsense_CLK_SOURCE_PRS8:
                    case capsense_CLK_SOURCE_PRS12:
                        /* Multiply by 2 for PRS8/PRS12 mode */
                        freqOffset1 <<= 1u;
                        freqOffset2 <<= 1u;
                        break;

                    default:
                        break;
                    }
                #endif

                /* Change the divider based on the chId */
                switch (chId)
                {
                    case capsense_FREQ_CHANNEL_1:
                    {
                        snsClkDivider += freqOffset1;
                        break;
                    }
                    case capsense_FREQ_CHANNEL_2:
                    {
                        snsClkDivider += freqOffset2;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                /* Set Number Of Conversions based on scanning resolution */
                conversionsNum = capsense_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution, (uint32)ptrWdgt->snsClkSource);
                CY_SET_REG32(capsense_CSD_SEQ_NORM_CNT_PTR, (conversionsNum & capsense_CSD_SEQ_NORM_CNT_CONV_CNT_MSK));

                #if((capsense_CLK_SOURCE_PRS8  == capsense_CSD_SNS_CLK_SOURCE) ||\
                    (capsense_CLK_SOURCE_PRS12 == capsense_CSD_SNS_CLK_SOURCE) ||\
                    (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE))
                    switch (snsClkSrc)
                    {
                    case capsense_CLK_SOURCE_PRS8:
                    case capsense_CLK_SOURCE_PRS12:
                        /* Divide by 2 for PRS8/PRS12 mode */
                        snsClkDivider >>= 1;
                        if (snsClkDivider == 0u)
                        {
                            snsClkDivider = 1u;
                        }
                        break;

                    default:
                        break;
                    }
                #endif

                /* Configure the new divider */
                capsense_SsSetSnsClockDivider(snsClkDivider);

                break;
        #endif /* #if (0u != capsense_TOTAL_CSD_WIDGETS) */

        #if (0u != capsense_TOTAL_CSX_WIDGETS)
            case capsense_SENSE_METHOD_CSX_E:
                #if (capsense_ENABLE == capsense_CSX_COMMON_TX_CLK_EN)
                    snsClkDivider = capsense_dsRam.snsCsxClk;
                #else /* (capsense_ENABLE == capsense_CSX_COMMON_TX_CLK_EN) */
                    snsClkDivider = ptrWdgt->snsClk;
                #endif /* (capsense_ENABLE == capsense_CSX_COMMON_TX_CLK_EN) */
                freqOffset1 = capsense_CSX_MFS_DIVIDER_OFFSET_F1;
                freqOffset2 = capsense_CSX_MFS_DIVIDER_OFFSET_F2;

                /* Change the divider based on the chId */
                switch (chId)
                {
                    case capsense_FREQ_CHANNEL_1:
                    {
                        snsClkDivider += freqOffset1;
                        break;
                    }
                    case capsense_FREQ_CHANNEL_2:
                    {
                        snsClkDivider += freqOffset2;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                /* Configure the new divider */
                capsense_SsSetSnsClockDivider(snsClkDivider);

                break;
        #endif /* #if (0u != capsense_TOTAL_CSX_WIDGETS) */

        default:
            CY_ASSERT(0 != 0);
            break;
        }
    }
#endif  /* (capsense_ENABLE == capsense_MULTI_FREQ_SCAN_EN) */


#if((capsense_ENABLE == capsense_CSD_EN) ||\
    (capsense_ENABLE == capsense_CSX_EN))
/*******************************************************************************
* Function Name: capsense_SsInitializeSourceSenseClk
****************************************************************************//**
*
* \brief
*  Sets a source for Sense Clk for all CSD widgets.
*
* \details
*  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk.
*  for all CSD widgets.
*
*******************************************************************************/
void capsense_SsInitializeSourceSenseClk(void)
{
    uint32 wdgtIndex;
    capsense_RAM_WD_BASE_STRUCT *ptrWdgt;

    for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
    {
        ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)capsense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

        switch(capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[wdgtIndex]))
        {
        #if (0u != capsense_TOTAL_CSD_WIDGETS)
            case capsense_SENSE_METHOD_CSD_E:
                capsense_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);
                break;
        #endif /* #if (0u != capsense_TOTAL_CSD_WIDGETS) */

        #if (0u != capsense_TOTAL_CSX_WIDGETS)
            case capsense_SENSE_METHOD_CSX_E:
                capsense_SsSetWidgetTxClkSrc(wdgtIndex, ptrWdgt);
                break;
        #endif /* #if (0u != capsense_TOTAL_CSX_WIDGETS) */

        default:
            CY_ASSERT(0 != 0);
            break;
        }

        #if (capsense_ENABLE == capsense_TST_WDGT_CRC_EN)
            capsense_DsUpdateWidgetCrc(wdgtIndex);
        #endif /* (capsense_ENABLE == capsense_TST_WDGT_CRC_EN) */
    }
}
#endif /* ((capsense_ENABLE == capsense_CSD_EN) ||\
           (capsense_ENABLE == capsense_CSX_EN)) */


#if (capsense_ENABLE == capsense_CSD_EN)
    /*******************************************************************************
    * Function Name: capsense_SsSetWidgetSenseClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the sense clock for a widget.
    *
    * \param wdgtIndex
    *  Specifies the ID of the widget.
    * \param ptrWdgt
    *  The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk for a
    *  widget.
    *
    *******************************************************************************/
    static void capsense_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, capsense_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if(capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)
            uint32 conversionsNum;
            uint32 snsClkDivider;
        #endif /* (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE) */

        #if(capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)
            snsClkDivider = capsense_SsCSDGetColSnsClkDivider(wdgtIndex);

                conversionsNum = capsense_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution,
                                                                                             capsense_CLK_SOURCE_DIRECT);
                lfsrSize = capsense_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                if (capsense_CLK_SOURCE_DIRECT == lfsrSize)
                {
                    lfsrSize = capsense_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                }
                lfsrScale = capsense_SsCalcLfsrScale(snsClkDivider, lfsrSize);

                if((lfsrSize == capsense_CLK_SOURCE_PRS8) || (lfsrSize == capsense_CLK_SOURCE_PRS12))
                {
                    capsense_SsCSDSetColSnsClkDivider(wdgtIndex, snsClkDivider);
                }
        #else
            lfsrSize = (uint8)capsense_DEFAULT_MODULATION_MODE;
            lfsrScale = 0u;
            (void)wdgtIndex; /* This parameter is unused in such configurations */
        #endif /* (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << capsense_CLK_SOURCE_LFSR_SCALE_OFFSET);

        #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
            if ((capsense_WD_TOUCHPAD_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                (capsense_WD_MATRIX_BUTTON_E == (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[wdgtIndex].wdgtType))
            {
                #if(capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)
                    snsClkDivider = capsense_SsCSDGetRowSnsClkDivider(wdgtIndex);

                        lfsrSize = capsense_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                        if (capsense_CLK_SOURCE_DIRECT == lfsrSize)
                        {
                            lfsrSize = capsense_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                        }
                        lfsrScale = capsense_SsCalcLfsrScale(snsClkDivider, lfsrSize);

                        if((lfsrSize == capsense_CLK_SOURCE_PRS8) || (lfsrSize == capsense_CLK_SOURCE_PRS12))
                        {
                            capsense_SsCSDSetRowSnsClkDivider(wdgtIndex, snsClkDivider);
                        }
                #else
                    lfsrSize = (uint8)capsense_DEFAULT_MODULATION_MODE;
                    lfsrScale = 0u;
                #endif /* (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE) */
                ptrWdgt->rowSnsClkSource = lfsrSize | (uint8)(lfsrScale << capsense_CLK_SOURCE_LFSR_SCALE_OFFSET);
            }
        #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */
    }
#endif /* (capsense_ENABLE == capsense_CSD_EN) */


#if (capsense_ENABLE == capsense_CSX_EN)
    /*******************************************************************************
    * Function Name: capsense_SsSetWidgetTxClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the Tx clock for a widget.
    *
    * \param wdgtIndex
    *  Specifies the ID of the widget.
    * \param ptrWdgt
    *  The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSource with with a source for Tx Clk for a widget.
    *
    *******************************************************************************/
    __STATIC_INLINE void capsense_SsSetWidgetTxClkSrc(uint32 wdgtIndex, capsense_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if ((capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE) && \
             (capsense_DISABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES))
            uint32 conversionsNum;
            uint32 snsClkDivider;
        #endif

        #if(capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE)
            #if (capsense_DISABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)
                conversionsNum = (uint32)ptrWdgt->resolution;
                snsClkDivider = capsense_SsCSXGetTxClkDivider(wdgtIndex);
                lfsrSize = capsense_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                lfsrScale = capsense_SsCalcLfsrScale(snsClkDivider, lfsrSize);
            #else
                lfsrSize = (uint8)capsense_CLK_SOURCE_DIRECT;
                lfsrScale = 0u;
                /* Unused function argument */
                (void)wdgtIndex;
            #endif /* (capsense_ENABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES) */
        #else
            lfsrSize = (uint8)capsense_CSX_TX_CLK_SOURCE;
            lfsrScale = 0u;
            /* Unused function argument */
            (void)wdgtIndex;
        #endif /* (capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << capsense_CLK_SOURCE_LFSR_SCALE_OFFSET);
    }
#endif /* (capsense_ENABLE == capsense_CSX_EN) */


#if(((capsense_ENABLE == capsense_CSX_EN) && \
     (capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE) && \
     (capsense_DISABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)) ||\
    ((capsense_ENABLE == capsense_CSD_EN) && \
     (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)))
/*******************************************************************************
* Function Name: capsense_SsCalcLfsrSize
****************************************************************************//**
*
* \brief
*  This is an internal function that finds a SSC polynomial size in the Auto mode.
*
* \details
*  The SSC polynomial size in the auto mode is found based on the following
*  requirements:
*  - an LFSR value should be selected so that the max clock dither is limited with +/-10%.
*  - at least one full spread spectrum polynomial should pass during the scan time.
*  - the value of the number of conversions should be an integer multiple of the
*    repeat period of the programmed LFSR_SIZE.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  resolution The widget resolution.
*
* \return lfsrSize The LFSRSIZE value for the SENSE_PERIOD register.
*
*******************************************************************************/
static uint8 capsense_SsCalcLfsrSize(uint32 snsClkDivider, uint32 conversionsNum)
{
    uint8 lfsrSize = 0u;

    /* Find LFSR value */
    if((capsense_SNSCLK_SSC4_THRESHOLD <= snsClkDivider) &&
       (capsense_SNSCLK_SSC4_PERIOD <= conversionsNum) &&
       (0uL == (conversionsNum % capsense_SNSCLK_SSC4_PERIOD)))
    {
        lfsrSize = capsense_CLK_SOURCE_SSC4;
    }
    else if((capsense_SNSCLK_SSC3_THRESHOLD <= snsClkDivider) &&
            (capsense_SNSCLK_SSC3_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % capsense_SNSCLK_SSC3_PERIOD)))
    {
        lfsrSize = capsense_CLK_SOURCE_SSC3;
    }
    else if((capsense_SNSCLK_SSC2_THRESHOLD <= snsClkDivider) &&
            (capsense_SNSCLK_SSC2_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % capsense_SNSCLK_SSC2_PERIOD)))
    {
        lfsrSize = capsense_CLK_SOURCE_SSC2;
    }
    else if((capsense_SNSCLK_SSC1_THRESHOLD <= snsClkDivider) &&
            (capsense_SNSCLK_SSC1_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % capsense_SNSCLK_SSC1_PERIOD)))
    {
        lfsrSize = capsense_CLK_SOURCE_SSC1;
    }
    else
    {
        lfsrSize = (uint8)capsense_CLK_SOURCE_DIRECT;
    }

    return (lfsrSize);
}


/*******************************************************************************
* Function Name: capsense_SsCalcLfsrScale
****************************************************************************//**
*
* \brief
*  This is an internal function that calculates the LFSR scale value.
*
* \details
*  The LFSR scale value is used to increase the clock dither if the desired dither
*  is wider than can be achieved with the current Sense Clock Divider and current LFSR
*  period.
*
*  This returns the LFSR scale value needed to keep the clock dither in
*  range +/-10%.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  lfsrSize The period of the LFSR sequence.
*          capsense_CLK_SOURCE_DIRECT The spreadspectrum is not used.
*          capsense_CLK_SOURCE_SSC1   The length of LFSR sequence is 63 cycles.
*          capsense_CLK_SOURCE_SSC2   The length of LFSR sequence is 127 cycles.
*          capsense_CLK_SOURCE_SSC3   The length of LFSR sequence is 255 cycles.
*          capsense_CLK_SOURCE_SSC4   The length of LFSR sequence is 511 cycles.
*
* \return The LFSR scale value needed to keep the clock dither in range +/-10%.
*
*******************************************************************************/
static uint8 capsense_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize)
{
    uint32 lfsrScale;
    uint32 lfsrRange;
    uint32 lfsrDither;

    /* Initialize the lfsrSize variable with the LFSR Range for given Lfsr Size. */
    switch(lfsrSize)
    {
        case capsense_CLK_SOURCE_SSC1:
        {
            lfsrRange = capsense_SNSCLK_SSC1_RANGE;
            break;
        }
        case capsense_CLK_SOURCE_SSC2:
        {
            lfsrRange = capsense_SNSCLK_SSC2_RANGE;
            break;
        }
        case capsense_CLK_SOURCE_SSC3:
        {
            lfsrRange = capsense_SNSCLK_SSC3_RANGE;
            break;
        }
        case capsense_CLK_SOURCE_SSC4:
        {
            lfsrRange = capsense_SNSCLK_SSC4_RANGE;
            break;
        }
        default:
        {
            lfsrRange = 0u;
            break;
        }
    }

    /* Calculate the LFSR Scale value that is required to keep the Clock dither
     * as close as possible to the +/-10% limit of the used frequency.
     */
    if((lfsrSize != capsense_CLK_SOURCE_DIRECT) && (0u != lfsrRange))
    {
        /* Calculate the LFSR Dither in percents. */
        lfsrDither  = ((lfsrRange * 100uL) / snsClkDivider);
        lfsrScale = 0uL;

        while(lfsrDither < capsense_LFSR_DITHER_PERCENTAGE)
        {
            lfsrScale++;
            lfsrDither <<=1uL;
        }

        if(lfsrDither > capsense_LFSR_DITHER_PERCENTAGE)
        {
            lfsrScale--;
        }
    }
    else
    {
        lfsrScale = 0uL;
    }

    return ((uint8)lfsrScale);
}

#endif /* (((capsense_ENABLE == capsense_CSX_EN) && \
            (capsense_CLK_SOURCE_PRSAUTO == capsense_CSX_TX_CLK_SOURCE) && \
            (capsense_DISABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)) ||\
           ((capsense_ENABLE == capsense_CSD_EN) && \
            (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE))) */


#if (capsense_ENABLE == capsense_CSD_EN)
    /*******************************************************************************
    * Function Name: capsense_SsClearCSDSensors
    ****************************************************************************//**
    *
    * \brief
    *  Resets all the CSD sensors to the non-sampling state by sequentially
    *  disconnecting all the sensors from the Analog MUX bus and putting them to
    *  an inactive state.
    *
    * \details
    *  The function goes through all the widgets and updates appropriate bits in
    *  the IO HSIOM, PC and DR registers depending on the Inactive sensor connection
    *  parameter. DR register bits are set to zero when the Inactive sensor
    *  connection is Ground or Hi-Z.
    *
    *******************************************************************************/
    void capsense_SsClearCSDSensors(void)
    {
        uint32 wdgtIndex;
        uint32 snsIndex;

        #if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
            uint32 pinIndex;
        #endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */

        #if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
            /* Declare ptr to sensor IO structure */
            capsense_FLASH_IO_STRUCT const *curDedicatedSnsIOPtr;
            /* Pointer to Flash structure holding info of sensor to be scanned */
            capsense_FLASH_SNS_STRUCT const *curFlashSnsPtr;
        #endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */
        capsense_FLASH_IO_STRUCT const *curSnsIOPtr;

        for (wdgtIndex = 0u; wdgtIndex < capsense_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (capsense_SENSE_METHOD_CSD_E ==
                capsense_GET_SENSE_METHOD(&capsense_dsFlash.wdgtArray[wdgtIndex]))
            {
                curSnsIOPtr = (capsense_FLASH_IO_STRUCT const *)
                                                capsense_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash;

                /* Go through all the sensors in widget */
                for (snsIndex = 0u; snsIndex < (uint8)capsense_dsFlash.wdgtArray[wdgtIndex].totalNumSns; snsIndex++)
                {
                    #if (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN)
                        /* Check ganged sns flag */
                        if (capsense_GANGED_SNS_MASK == (capsense_dsFlash.wdgtArray[wdgtIndex].staticConfig &
                                                                 capsense_GANGED_SNS_MASK))
                        {
                            /* Get sns pointer */
                            curFlashSnsPtr = (capsense_FLASH_SNS_STRUCT const *)
                                                               capsense_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash +
                                                               snsIndex;

                            for(pinIndex = 0u; pinIndex < curFlashSnsPtr->numPins; pinIndex++)
                            {
                                /* Get IO pointer for dedicated pin */
                                curDedicatedSnsIOPtr = &capsense_ioList[curFlashSnsPtr->firstPinId + pinIndex];

                                /* Disconnect dedicated pin */
                                capsense_CSDDisconnectSns(curDedicatedSnsIOPtr);
                            }
                        }
                        else
                        {
                            /* Disable sensor */
                            capsense_CSDDisconnectSns(&curSnsIOPtr[snsIndex]);
                        }
                    #else
                        /* Disable sensor */
                        capsense_CSDDisconnectSns(&curSnsIOPtr[snsIndex]);
                    #endif /* (capsense_ENABLE == capsense_CSD_GANGED_SNS_EN) */
                }
            }
        }
    }

    /*******************************************************************************
    * Function Name: capsense_SsCSDGetColSnsClkDivider
    ****************************************************************************//**
    *
    * \brief
    *  This function gets the Sense Clock Divider value for one-dimension widgets
    *  and Column Sense Clock divider value for two-dimension widgets.
    *
    * \details
    *  This function gets the Sense Clock Divider value based on the Component
    *  configuration. The function is applicable for one-dimension widgets and for
    *  two-dimension widgets.
    *
    * \param
    *  widgetId Specifies the ID of the widget.
    *
    * \return The Sense Clock Divider value for one-dimension widgets
    *        and the Column Sense Clock divider value for two-dimension widgets.
    *
    *******************************************************************************/
    uint32 capsense_SsCSDGetColSnsClkDivider(uint32 widgetId)
    {
        uint32 retVal;

        /* Get sense divider based on configuration */
        #if (capsense_ENABLE != capsense_CSD_COMMON_SNS_CLK_EN)
            capsense_RAM_WD_BASE_STRUCT *ptrWdgt;

            ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
            capsense_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

            retVal = (uint32)(ptrWdgt->snsClk);
        #else
            retVal = (uint32)capsense_dsRam.snsCsdClk;

            (void)widgetId; /* This parameter is unused in such configurations */
        #endif /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */

        return (retVal);
    }


    /*******************************************************************************
    * Function Name: capsense_SsCSDSetColSnsClkDivider
    ****************************************************************************//**
    *
    * \brief
    *  This function sets the Sense Clock Divider value for one-dimension widgets
    *  and Column Sense Clock divider value for two-dimension widgets.
    *
    * \details
    *  This function sets the Sense Clock Divider value based on the Component
    *  configuration. The function is applicable for one-dimension widgets and for
    *  two-dimension widgets.
    *
    * \param
    *  widgetId Specifies the ID of the widget.
    *
    * \return The Sense Clock Divider value for one-dimension widgets
    *        and the Column Sense Clock divider value for two-dimension widgets.
    *
    *******************************************************************************/
    void capsense_SsCSDSetColSnsClkDivider(uint32 widgetId, uint32 dividerVal)
    {
        /* Get sense divider based on configuration */
        #if (capsense_ENABLE != capsense_CSD_COMMON_SNS_CLK_EN)
            capsense_RAM_WD_BASE_STRUCT *ptrWdgt;

            ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
            capsense_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

            ptrWdgt->snsClk = (uint16)dividerVal;
        #else
            capsense_dsRam.snsCsdClk = (uint16)dividerVal;

            (void)widgetId; /* This parameter is unused in such configurations */
        #endif /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */
    }


    #if (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN)
        /*******************************************************************************
        * Function Name: capsense_SsCSDGetRowSnsClkDivider
        ****************************************************************************//**
        *
        * \brief
        *  This function gets the Sense Clock Divider value for one-dimension widgets
        *  and the Column Sense Clock divider value for two-dimension widgets.
        *
        * \details
        *  This function gets the Sense Clock Divider value based on the Component
        *  configuration. The function is applicable for one-dimension widgets and for
        *  two-dimension widgets.
        *
        * \param
        *  widgetId Specifies the ID of the widget.
        *
        * \return
        *  Returns the sense clock divider value for one-dimension widgets
        *  and column sense clock divider value for two-dimension widgets.
        *
        *******************************************************************************/
        uint32 capsense_SsCSDGetRowSnsClkDivider(uint32 widgetId)
        {
            uint32 retVal;

            /* Get sense divider based on configuration */
            #if (capsense_ENABLE != capsense_CSD_COMMON_SNS_CLK_EN)
                capsense_RAM_WD_BASE_STRUCT *ptrWdgt;

                ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                capsense_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

                retVal = ptrWdgt->rowSnsClk;
            #else
                retVal = (uint32)capsense_dsRam.snsCsdClk;

                (void)widgetId; /* This parameter is unused in such configurations */
            #endif /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */

            return (retVal);
        }


        /*******************************************************************************
        * Function Name: capsense_SsCSDSetRowSnsClkDivider
        ****************************************************************************//**
        *
        * \brief
        *  This function sets the Sense Clock Divider value for one-dimension widgets
        *  and the Column Sense Clock divider value for two-dimension widgets.
        *
        * \details
        *  This function sets the Sense Clock Divider value based on the Component
        *  configuration. The function is applicable for one-dimension widgets and for
        *  two-dimension widgets.
        *
        * \param
        *  widgetId Specifies the ID of the widget.
        *
        * \param
        *  dividerVal Specifies the Sense Clock Divider value.
        *
        *******************************************************************************/
        void capsense_SsCSDSetRowSnsClkDivider(uint32 widgetId, uint32 dividerVal)
        {
            /* Get sense divider based on configuration */
            #if (capsense_ENABLE != capsense_CSD_COMMON_SNS_CLK_EN)
                capsense_RAM_WD_BASE_STRUCT *ptrWdgt;

                ptrWdgt = (capsense_RAM_WD_BASE_STRUCT *)
                capsense_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

                ptrWdgt->rowSnsClk = (uint16)dividerVal;
            #else
                capsense_dsRam.snsCsdClk = (uint16)dividerVal;

                (void)widgetId; /* This parameter is unused in such configurations */
            #endif /* (capsense_ENABLE == capsense_CSD_COMMON_SNS_CLK_EN) */
        }
    #endif /* (capsense_CSD_MATRIX_WIDGET_EN || capsense_CSD_TOUCHPAD_WIDGET_EN) */


    #if (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE)
        /*******************************************************************************
        * Function Name: capsense_SsCSDCalcPrsSize
        ****************************************************************************//**
        *
        * \brief
        *  The function finds PRS polynomial size in the Auto mode.
        *
        * \details
        *  The PRS polynomial size in the Auto mode is found based on the following
        *  requirements:
        *  - at least one full spread spectrum polynomial should pass during scan time.
        *
        * \param
        *  snsClkDivider The divider value for the sense clock.
        *  resolution The widget resolution.
        *
        * \return prsSize PRS value for SENSE_PERIOD register.
        *
        *******************************************************************************/
        uint8 capsense_SsCSDCalcPrsSize(uint32 snsClkDivider, uint32 resolution)
        {
            uint32 prsSize;

            if ((snsClkDivider * capsense_PRS_LENGTH_12_BITS) <= ((0x00000001Lu << resolution) - 1u))
            {
                /* Set PRS12 mode */
                prsSize = capsense_CLK_SOURCE_PRS12;
            }
            else if ((snsClkDivider * capsense_PRS_LENGTH_8_BITS) <= ((0x00000001Lu << resolution) - 1u))
            {
                /* Set PRS8 mode */
                prsSize = capsense_CLK_SOURCE_PRS8;
            }
            else
            {
                /* Set Direct clock mode */
                prsSize = capsense_CLK_SOURCE_DIRECT;
            }

            return (uint8)prsSize;
        }
    #endif /* (capsense_CLK_SOURCE_PRSAUTO == capsense_CSD_SNS_CLK_SOURCE) */
#endif /* (capsense_ENABLE == capsense_CSD_EN) */


/*******************************************************************************
* Function Name: capsense_BistDischargeExtCapacitors
****************************************************************************//**
*
* \brief
*  The function discharge available external capacitors.
*
* \details
*  The function discharge available external capacitors by connection them
*  to GND using STRONG GPIO drive mode. Additionaly, the function disconnects
*  the capacitors from analog mux buses if connected.
*  Note: the function does not restore the connection to analog mux busses
*  and supposes that all the capacitors belong to a single device port.
*
*******************************************************************************/
void capsense_BistDischargeExtCapacitors(void)
{
    #if (capsense_ENABLE == capsense_CSD_EN)
        Cy_GPIO_SetHSIOM(capsense_Cmod_0_PORT, capsense_Cmod_0_NUM, HSIOM_SEL_GPIO);
        Cy_GPIO_Clr(capsense_Cmod_0_PORT, capsense_Cmod_0_NUM);
        Cy_GPIO_SetDrivemode(capsense_Cmod_0_PORT, capsense_Cmod_0_NUM, CY_GPIO_DM_STRONG_IN_OFF);

        #if((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
            (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN))
            Cy_GPIO_SetHSIOM(capsense_Csh_0_PORT, capsense_Csh_0_NUM, HSIOM_SEL_GPIO);
            Cy_GPIO_Clr(capsense_Csh_0_PORT, capsense_Csh_0_NUM);
            Cy_GPIO_SetDrivemode(capsense_Csh_0_PORT, capsense_Csh_0_NUM, CY_GPIO_DM_STRONG_IN_OFF);
        #endif
    #endif

    #if (capsense_ENABLE == capsense_CSX_EN)
        Cy_GPIO_SetHSIOM(capsense_CintA_0_PORT, capsense_CintA_0_NUM, HSIOM_SEL_GPIO);
        Cy_GPIO_Clr(capsense_CintA_0_PORT, capsense_CintA_0_NUM);
        Cy_GPIO_SetDrivemode(capsense_CintA_0_PORT, capsense_CintA_0_NUM, CY_GPIO_DM_STRONG_IN_OFF);

        Cy_GPIO_SetHSIOM(capsense_CintB_0_PORT, capsense_CintB_0_NUM, HSIOM_SEL_GPIO);
        Cy_GPIO_Clr(capsense_CintB_0_PORT, capsense_CintB_0_NUM);
        Cy_GPIO_SetDrivemode(capsense_CintB_0_PORT, capsense_CintB_0_NUM, CY_GPIO_DM_STRONG_IN_OFF);
    #endif

    Cy_SysLib_DelayUs(capsense_EXT_CAP_DISCHARGE_TIME);

    #if (capsense_ENABLE == capsense_CSD_EN)
        Cy_GPIO_SetDrivemode(capsense_Cmod_0_PORT, capsense_Cmod_0_NUM, CY_GPIO_DM_ANALOG);

        #if((capsense_ENABLE == capsense_CSD_SHIELD_EN) && \
            (capsense_ENABLE == capsense_CSD_SHIELD_TANK_EN))
            Cy_GPIO_SetDrivemode(capsense_Csh_0_PORT, capsense_Csh_0_NUM, CY_GPIO_DM_ANALOG);
        #endif
    #endif

    #if (capsense_ENABLE == capsense_CSX_EN)
        Cy_GPIO_SetDrivemode(capsense_CintA_0_PORT, capsense_CintA_0_NUM, CY_GPIO_DM_ANALOG);
        Cy_GPIO_SetDrivemode(capsense_CintB_0_PORT, capsense_CintB_0_NUM, CY_GPIO_DM_ANALOG);
    #endif
}


/* [] END OF FILE */
