/***************************************************************************//**
* \file capsense_Structure.c
* \version 2.0
*
* \brief
*   This file defines the data structure global variables and provides implementation
*   for the high-level and low-level APIs of the Data Structure module.
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
#include <stddef.h>
#include <string.h>
#include "cyfitter_gpio.h"

#include "capsense_Structure.h"
#include "capsense_Configuration.h"

#if (0u != capsense_ADC_EN)
    #include "capsense_Adc.h"
#endif /* (0u != capsense_ADC_EN) */

#if (capsense_ENABLE == capsense_SELF_TEST_EN)
    #include "capsense_SelfTest.h"
#endif

#if (capsense_ENABLE == capsense_CENTROID_5X5_CSD_EN)
    #include "capsense_AdvancedCentroid_LL.h"
#endif

#if (capsense_ENABLE == capsense_GES_GLOBAL_EN)
    #include "capsense_Gesture.h"
#endif /* (capsense_ENABLE == capsense_GES_GLOBAL_EN) */

/*******************************************************************************
* Defines the RAM Data Structure variables and their init data in flash
*******************************************************************************/
/**
* \cond SECTION_GLOBAL_VARIABLES
* \addtogroup group_global_variables
* \{
*/

/**
* The variable that contains the capsense configuration, settings and
* scanning results. capsense_dsRam represents RAM Data Structure.
*/
capsense_RAM_STRUCT capsense_dsRam;
/** \}
* \endcond */

/*******************************************************************************
* Declares Widget's De-bounce Counters
*******************************************************************************/
static uint8 capsense_debounceLinearSlider0[1u];


/***************************************************************************//**
* Declares Noise Envelope data structures
*******************************************************************************/
static SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT capsense_noiseEnvlpLinearSlider0[capsense_LINEARSLIDER0_NUM_SENSORS];

/*******************************************************************************
* Defines and initializes the Flash Data Structure
*******************************************************************************/



/**
* \cond SECTION_API_CONSTANTS
* \addtogroup group_api_constants
* \{
*/
/**
* Constant for the FLASH Data Structure
*/
const capsense_FLASH_STRUCT capsense_dsFlash =
{
    /* Flash Widget Initialization Data */
    {
        { /* LinearSlider0 */
            &capsense_ioList[0u],
            (void *)&capsense_dsRam.wdgtList.linearslider0,
            capsense_dsRam.snsList.linearslider0,
            (void *)0u,
            capsense_debounceLinearSlider0,
            capsense_LINEARSLIDER0_STATIC_CONFIG,
            capsense_LINEARSLIDER0_NUM_SENSORS,
            (uint8)capsense_WD_LINEAR_SLIDER_E,
            capsense_LINEARSLIDER0_NUM_SENSORS,
            capsense_LINEARSLIDER0_X_RESOLUTION,
            capsense_LINEARSLIDER0_X_CENT_MULT,
            capsense_noiseEnvlpLinearSlider0,
            capsense_LINEARSLIDER0_IIR_FILTER_COEFF,
        },
    },
};

/**
* The array of the pointers to the electrode specific register.
*/
const capsense_FLASH_IO_STRUCT capsense_ioList[capsense_TOTAL_ELECTRODES] =
{
    { /* 0: LinearSlider0_Sns0 */
        capsense_Sns_0_PORT,
        (uint8)capsense_Sns_0_NUM,
    },
    { /* 1: LinearSlider0_Sns1 */
        capsense_Sns_1_PORT,
        (uint8)capsense_Sns_1_NUM,
    },
    { /* 2: LinearSlider0_Sns2 */
        capsense_Sns_2_PORT,
        (uint8)capsense_Sns_2_NUM,
    },
    { /* 3: LinearSlider0_Sns3 */
        capsense_Sns_3_PORT,
        (uint8)capsense_Sns_3_NUM,
    },
    { /* 4: LinearSlider0_Sns4 */
        capsense_Sns_4_PORT,
        (uint8)capsense_Sns_4_NUM,
    },
};



/** \}
* \endcond */

/* Initialization data for RAM widget list */
const capsense_RAM_WD_LIST_STRUCT capsense_ramWidgetInit =
{
    { /* LinearSlider0 */
        capsense_LINEARSLIDER0_RESOLUTION,
        capsense_LINEARSLIDER0_FINGER_TH,
        capsense_LINEARSLIDER0_NOISE_TH,
        capsense_LINEARSLIDER0_NNOISE_TH,
        capsense_LINEARSLIDER0_HYSTERESIS,
        capsense_LINEARSLIDER0_ON_DEBOUNCE,
        capsense_LINEARSLIDER0_LOW_BSLN_RST,
        {
            capsense_LINEARSLIDER0_IDAC_MOD0,
        },
        capsense_LINEARSLIDER0_SNS_CLK,
        capsense_LINEARSLIDER0_SNS_CLK_SOURCE,
        capsense_LINEARSLIDER0_FINGER_CAP,
        capsense_LINEARSLIDER0_SIGPFC,
        {
            capsense_LINEARSLIDER0_POSITION,
        },
    },
};


/* IDAC Initialization Data */
const uint8 capsense_ramIdacInit[capsense_TOTAL_SENSORS] =
{
    /* LinearSlider0 */
    capsense_LINEARSLIDER0_SNS0_IDAC_COMP0,
    capsense_LINEARSLIDER0_SNS1_IDAC_COMP0,
    capsense_LINEARSLIDER0_SNS2_IDAC_COMP0,
    capsense_LINEARSLIDER0_SNS3_IDAC_COMP0,
    capsense_LINEARSLIDER0_SNS4_IDAC_COMP0,
};



/*******************************************************************************
* Defines internal data types and statements
*******************************************************************************/
#define PARAM_TYPE_OFFSET       (30u)
#define PARAM_TYPE_MASK         (3Lu << PARAM_TYPE_OFFSET)
#define PARAM_RWBIT_OFFSET      (29u)
#define PARAM_RWBIT_MASK        (1Lu << PARAM_RWBIT_OFFSET)
#define PARAM_FBIT_OFFSET       (28u)
#define PARAM_FBIT_MASK         (1Lu << PARAM_FBIT_OFFSET)
#define PARAM_UBIT_OFFSET       (23u)
#define PARAM_UBIT_MASK         (1Lu << PARAM_UBIT_OFFSET)
#define PARAM_WDID_OFFSET       (16u)
#define PARAM_WDID_MASK         (0x7FLu << PARAM_WDID_OFFSET)
#define PARAM_CRC_OFFSET        (24u)
#define PARAM_CRC_MASK          (0x0FLu << PARAM_CRC_OFFSET)

/* Defines align masks for uint16 and uint32 */
#define UINT16_ALIGN_MASK       (1Lu)
#define UINT32_ALIGN_MASK       (3Lu)

/* Defines supported parameter types */
#define DS_PARAM_TYPE_UINT8     (0x01u)
#define DS_PARAM_TYPE_UINT16    (0x02u)
#define DS_PARAM_TYPE_UINT32    (0x03u)


/* Defines PARAM_ID structure */
typedef struct
{
    uint16 offset;      /* parameter offset */
    uint8  widgetId;    /* widget Id parameter belongs to */
    uint8  affectsCrc;  /* parameter affects widget CRC */
    uint8  crc;         /* parameter CRC */
    uint8  flash;       /* parameter located in FLASH flag */
    uint8  rw;          /* parameter is read/write flag */
    uint8  type;        /* parameter type: uint8/uint16/uint32 */
} PARAM_ID_STRUCT;


/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/



static cy_status DsParseParamId(uint32 paramId, PARAM_ID_STRUCT *pData);

/** \}
* \endcond */


/*******************************************************************************
* Function Name: capsense_DsInitialize
****************************************************************************//**
*
* \brief
*   This function initializes the Data Structure storage.
*
* \details
*   This function copies the default widget configuration parameters
*   from the Flash storage into the RAM Data Structure.
*
*******************************************************************************/
void capsense_DsInitialize(void)
{
    #if ((capsense_ENABLE == capsense_CSX_EN) || \
         (capsense_ENABLE == capsense_CSD_IDAC_COMP_EN) || \
        ((capsense_ENABLE == capsense_CSX_EN) && \
         (capsense_ENABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)))
        uint32 snsId;
        uint32 wdgtId;
        capsense_RAM_SNS_STRUCT * ptrSnsTmp;
        capsense_FLASH_WD_STRUCT const * ptrFlashWdgt;
    #endif

    #if ((capsense_ENABLE == capsense_CSX_EN) && \
         (capsense_ENABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES))
        uint16 subConvNumber;
    #endif

    #if (0u != capsense_CSX_EN) || (0 != capsense_CSD_IDAC_COMP_EN)
        uint8 const * ptrIdacInit = capsense_ramIdacInit;
    #endif

    /* Reset RAM data structure content */
    (void)memset(&capsense_dsRam, 0, sizeof(capsense_dsRam));

    /* Initialize configId, deviceId and hwClock registers */
    capsense_dsRam.configId = capsense_CONFIG_ID;
    capsense_dsRam.deviceId = capsense_DEVICE_ID;
    capsense_dsRam.hwClock = capsense_HW_CLOCK;

    /* Initialize global RAM data */
    capsense_dsRam.csd0Config = capsense_CSD0_CONFIG;

    #if (0u != capsense_TOTAL_CSD_WIDGETS)
        capsense_dsRam.modCsdClk = capsense_CSD_SCANSPEED_DIVIDER;

        #if (0u != capsense_CSD_COMMON_SNS_CLK_EN)
            capsense_dsRam.snsCsdClk = capsense_CSD_SNS_CLK_DIVIDER;
        #endif /* #if (0u != capsense_CSD_COMMON_SNS_CLK_EN) */
    #endif /* #if (0u != capsense_TOTAL_CSD_WIDGETS) */

    #if (0u != capsense_CSX_EN)
        capsense_dsRam.modCsxClk = capsense_CSX_SCANSPEED_DIVIDER;

        #if (0u != capsense_CSX_COMMON_TX_CLK_EN)
            capsense_dsRam.snsCsxClk = capsense_CSX_TX_CLK_DIVIDER;
        #endif /* #if (0u != capsense_CSX_COMMON_TX_CLK_EN) */
    #endif /* #if (0u != capsense_CSX_EN) */

    #if (0u != capsense_SELF_TEST_EN)
        capsense_dsRam.glbCrc = capsense_GLB_CRC;
    #endif /* #if (0u != capsense_SELF_TEST_EN) */

    /* Initialize RAM widget data */
    capsense_dsRam.wdgtList = capsense_ramWidgetInit;

    /* Initialize Gesture configuration structure */
    #if ((0u != capsense_GES_GLOBAL_EN) || (0u != capsense_BALLISTIC_MULTIPLIER_EN))
        capsense_dsRam.timestampInterval = capsense_TIMESTAMP_INTERVAL;
    #endif /* ((0u != capsense_GES_GLOBAL_EN) || (0u != capsense_BALLISTIC_MULTIPLIER_EN)) */

    #if (0u != capsense_GES_GLOBAL_EN)
        capsense_dsRam.gestures = capsense_ramGesturesInit;
    #endif /* (0u != capsense_GES_GLOBAL_EN) */

    #if (0u != capsense_BALLISTIC_MULTIPLIER_EN)
        capsense_dsRam.ballisticConfig = capsense_ramBallisticInit;
    #endif /* (0u != capsense_BALLISTIC_MULTIPLIER_EN) */

    #if (0u != capsense_CSX_EN) || (0 != capsense_CSD_IDAC_COMP_EN)
        /* Initialize IDAC data */
        ptrFlashWdgt = capsense_dsFlash.wdgtArray;

        for (wdgtId = capsense_TOTAL_WIDGETS; wdgtId-- > 0u; )
        {
            ptrSnsTmp = ptrFlashWdgt->ptr2SnsRam;

            for (snsId = capsense_GET_SNS_CNT_BY_PTR(ptrFlashWdgt); snsId-- > 0u;)
            {
                ptrSnsTmp->idacComp[0u] = *ptrIdacInit;
                ptrIdacInit++;

                #if (0u != capsense_MULTI_FREQ_SCAN_EN)
                    ptrSnsTmp->idacComp[1u] = *ptrIdacInit;
                    ptrIdacInit++;

                    ptrSnsTmp->idacComp[2u] = *ptrIdacInit;
                    ptrIdacInit++;
                #endif /* #if (0u != capsense_MULTI_FREQ_SCAN_EN) */

                ptrSnsTmp++;
            }

            ptrFlashWdgt++; /* Move to next widget */
        }
    #endif /*(0u != capsense_CSX_EN) || (0 != capsense_CSD_IDAC_COMP_EN)*/

    #if ((capsense_ENABLE == capsense_CSX_EN) && \
         (capsense_ENABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES))
        /* Initialize Sub-Conversion Number for each node of each CSX widget */
        ptrFlashWdgt = capsense_dsFlash.wdgtArray;
        for (wdgtId = capsense_TOTAL_WIDGETS; wdgtId-- > 0u;)
        {
            /* Check for widget sense method */
            if (capsense_SENSE_METHOD_CSX_E == capsense_GET_SENSE_METHOD(ptrFlashWdgt))
            {
                /* Take the sub-convesion number of a CSX widget and initialize each node of this widget */
                subConvNumber = ((capsense_RAM_WD_BASE_STRUCT *)(ptrFlashWdgt->ptr2WdgtRam))->resolution;
                ptrSnsTmp = ptrFlashWdgt->ptr2SnsRam;
                for (snsId = capsense_GET_SNS_CNT_BY_PTR(ptrFlashWdgt); snsId-- > 0u;)
                {
                    ptrSnsTmp->subConvNum = subConvNumber;
                    ptrSnsTmp++;
                }
            }
            /* Move to the next widget */
            ptrFlashWdgt++;
        }
    #endif /* ((capsense_ENABLE == capsense_CSX_EN) && \
               (capsense_ENABLE == capsense_CSX_SKIP_OVSMPL_SPECIFIC_NODES)) */

    #if (0u != capsense_ADC_EN)
        capsense_AdcDsInitialize();
    #endif /* (0u != capsense_ADC_EN) */
}


#if (0u != capsense_ADC_EN)
    /*******************************************************************************
    * Function Name: capsense_AdcDsInitialize
    ****************************************************************************//**
    *
    * \brief
    *   Configures the initial Adc datastructure members.
    *
    *******************************************************************************/
    void capsense_AdcDsInitialize(void)
    {
        capsense_dsRam.adcResolution = capsense_ADC_RESOLUTION;
        capsense_dsRam.adcIdac = (uint8)(capsense_ADC_IDAC_DEFAULT);
        capsense_dsRam.adcActiveCh = capsense_AdcNO_CHANNEL;
    }
#endif /* (0u != capsense_ADC_EN) */


/*******************************************************************************
* Function Name: DsParseParamId
****************************************************************************//**
*
* \brief
*   This function parses the parameter ID and checks its correctness.
*
* \details
*   This function checks the parameter CRC if the self-test is enabled,
*   validates the offset value and parameter type.
*
* \param paramId The parameter to parse.
* \param pData   The pointer to the structure that will hold parsed data.
*
* \return CY_RET_SUCCESS     If the parameter is valid
*         CY_RET_BAD_PARAM   If  the parameter is invalid
*
*******************************************************************************/
static cy_status DsParseParamId(uint32 paramId, PARAM_ID_STRUCT *pData)
{
    cy_status result = CY_RET_SUCCESS;

    #if (0u != capsense_SELF_TEST_EN)
        const uint8 crcTable[] =
        {
            0x00u, 0x09u, 0x0Bu, 0x02u, 0x0Fu, 0x06u, 0x04u, 0x0Du,
            0x07u, 0x0Eu, 0x0Cu, 0x05u, 0x08u, 0x01u, 0x03u, 0x0Au
        };

        uint32 i;
        uint32 actualCrc = 0u;
        uint32 crcIndex;
    #endif /* #if (0u != capsense_SELF_TEST_EN) */

    /* Extract parameter data */
    pData->offset     = CY_LO16(paramId);
    pData->affectsCrc = CY_LO8((paramId & PARAM_UBIT_MASK)  >> PARAM_UBIT_OFFSET);
    pData->widgetId   = CY_LO8((paramId & PARAM_WDID_MASK)  >> PARAM_WDID_OFFSET);
    pData->type       = CY_LO8((paramId & PARAM_TYPE_MASK)  >> PARAM_TYPE_OFFSET);
    pData->flash      = CY_LO8((paramId & PARAM_FBIT_MASK)  >> PARAM_FBIT_OFFSET);
    pData->rw         = CY_LO8((paramId & PARAM_RWBIT_MASK) >> PARAM_RWBIT_OFFSET);
    pData->crc        = CY_LO8((paramId & PARAM_CRC_MASK)   >> PARAM_CRC_OFFSET);

    /* Check parameter CRC if self-test is enabled */
    #if (0u != capsense_SELF_TEST_EN)

        /* Calculate CRC for bits 0..24.
         * The CRC is calculated using nibbles (4-bits).
         * So for 0..24 bits there are in total 6 nibbles. */
        for (i = 6u; i > 0u; i--)
        {
            crcIndex = actualCrc ^ (paramId & 0x0Fu);
            actualCrc = crcTable[crcIndex];
            paramId >>= 4u;
        }

        /* Add bits 28..32 */
        crcIndex = actualCrc ^ ((paramId >> 4u) & 0x0Fu);
        actualCrc = crcTable[crcIndex];

        /* Add bits 24..28 (CRC) */
        crcIndex = actualCrc ^ (paramId & 0x0Fu);
        actualCrc = crcTable[crcIndex];

        if (0u != actualCrc)
        {
            /* CRC mismatch */
            result = CY_RET_BAD_PARAM;
        }

        if ((CY_RET_SUCCESS == result) &&
            (0u != pData->affectsCrc) && (pData->widgetId >= capsense_TOTAL_WIDGETS ))
        {
            /* Wrong widgetId for protected parameter */
            result = CY_RET_BAD_PARAM;
        }

        /* Check offset value range */
        if (CY_RET_SUCCESS == result)
        {
            if (0u == pData->flash)
            {
                /* Check offset for RAM Data Structure range */
                if (pData->offset >= sizeof(capsense_dsRam))
                {
                    result = CY_RET_BAD_PARAM;
                }
            }
            else
            {
                /* Check offset for Flash Data Structure range */
                if (pData->offset >= sizeof(capsense_dsFlash))
                {
                    result = CY_RET_BAD_PARAM;
                }

                /* Check if it is marked as Read Only */
                if (0u != pData->rw)
                {
                    result = CY_RET_BAD_PARAM;
                }
            }
        }
    #else /* #if (0u != capsense_SELF_TEST_EN) */
        /* Check offset value range */
        if (0u == pData->flash)
        {
            /* Check offset for RAM Data Structure range */
            if (pData->offset >= sizeof(capsense_dsRam))
            {
                result = CY_RET_BAD_PARAM;
            }
        }
        else
        {
            /* Check offset for Flash Data Structure range */
            if (pData->offset >= sizeof(capsense_dsFlash))
            {
                result = CY_RET_BAD_PARAM;
            }

            /* Check if it is marked as Read Only */
            if (0u != pData->rw)
            {
                result = CY_RET_BAD_PARAM;
            }
        }
    #endif /* #if (0u != capsense_SELF_TEST_EN) */

    if (CY_RET_SUCCESS == result)
    {
        /* Check parameter type, offset alignment */
        switch (pData->type)
        {
        case DS_PARAM_TYPE_UINT16:
            if (0u != (pData->offset & UINT16_ALIGN_MASK))
            {
                result = CY_RET_BAD_PARAM;
            }
            break;

        case DS_PARAM_TYPE_UINT32:
            if (0u != (pData->offset & UINT32_ALIGN_MASK))
            {
                result = CY_RET_BAD_PARAM;
            }
            break;

        case DS_PARAM_TYPE_UINT8:
            break;

        default:
            result = CY_RET_BAD_PARAM;
            break;
        }
    }

    return result;
}

/*******************************************************************************
* Function Name: capsense_GetParam
****************************************************************************//**
*
* \brief
*  Gets the specified parameter value from the \ref group_structures.
*
* \details
*  This function gets the value of the specified parameter by the paramId
*  argument. The paramId for each register is available in the
*  capsense RegisterMap header file as
*  capsense_<ParameterName>_PARAM_ID. The paramId is a special
*  enumerated value generated by the customizer. The format of paramId is as
*  follows:
*    1. [ byte 3 byte 2 byte 1 byte 0 ]
*    2. [ TTWFCCCC UIIIIIII MMMMMMMM LLLLLLLL ]
*    3. T - encodes the parameter type:
*      - 01b: uint8
*      - 10b: uint16
*      - 11b: uint32
*    4. W - indicates whether the parameter is writable:
*      - 0: ReadOnly
*      - 1: Read/Write
*    5. C - 4 bit CRC (X^3 + 1) of the whole paramId word, the C bits are
*       filled with 0s when the CRC is calculated.
*    6. U - indicates if the parameter affects the RAM Widget Object CRC.
*    7. I - specifies that the widgetId parameter belongs to
*    8. M,L - the parameter offset MSB and LSB accordingly in:
*      - Flash Data Structure if W bit is 0.
*      - RAM Data Structure if W bit is 1.
*
*  Refer to the \ref group_structures section for details of the data structure
*  organization and examples of its register access.
*
* \param paramId
*  Specifies the ID of parameter to get its value.
*  A macro for the parameter ID can be found in the capsense RegisterMap header
*  file defined as capsense_<ParameterName>_PARAM_ID.
*
* \param  value
*  The pointer to a variable to be updated with the obtained value.
*
* \return
*  Returns the status of the operation:
*    - CY_RET_SUCCESS - The operation is successfully completed.
*    - CY_RET_BAD_PARAM - The input parameter is invalid.
*
*******************************************************************************/
cy_status capsense_GetParam(uint32 paramId, uint32 *value)
{
    PARAM_ID_STRUCT pData;
    cy_status result;

    union
    {
        volatile void   const * ptr;
        volatile uint8  const * ptrUint8;
        volatile uint16 const * ptrUint16;
        volatile uint32 const * ptrUint32;
    } ptrData;

    result = DsParseParamId(paramId, &pData);

    /* Parse and validate paramId */
    if (CY_RET_SUCCESS == result)
    {
        /* Get base address of parameter */
        if (0u == pData.flash)
        {
            ptrData.ptr = &capsense_dsRam;
        }
        else
        {
            ptrData.ptr = &capsense_dsFlash;
        }

        /* Add offset to base address */
        ptrData.ptrUint8 += pData.offset;

        /* Read data */
        if (DS_PARAM_TYPE_UINT8 == pData.type)
        {
            *value = (uint32) (*ptrData.ptrUint8);
        }
        else if (DS_PARAM_TYPE_UINT16 == pData.type)
        {
            *value = (uint32) (*ptrData.ptrUint16);
        }
        else if (DS_PARAM_TYPE_UINT32 == pData.type)
        {
            *value = *ptrData.ptrUint32;
        }
        else
        {
            /* Parameter comes here already validated. */
            CY_ASSERT(0 != 0);
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: capsense_SetParam
****************************************************************************//**
*
* \brief
*  Sets a new value for the specified parameter in the \ref group_structures.
*
* \details
*  This function sets the value of the specified parameter by the paramId
*  argument. The paramId for each register is available in the
*  capsense RegisterMap header file as
*  capsense_<ParameterName>_PARAM_ID. The paramId is a special
*  enumerated value generated by the customizer. The format of paramId is as
*  follows:
*    1. [ byte 3 byte 2 byte 1 byte 0 ]
*    2. [ TTWFCCCC UIIIIIII MMMMMMMM LLLLLLLL ]
*    3. T - encodes the parameter type:
*      - 01b: uint8
*      - 10b: uint16
*      - 11b: uint32
*    4. W - indicates whether the parameter is writable:
*      - 0: ReadOnly
*      - 1: Read/Write
*    5. C - 4 bit CRC (X^3 + 1) of the whole paramId word, the C bits are
*       filled with 0s when the CRC is calculated.
*    6. U - indicates if the parameter affects the RAM Widget Object CRC.
*    7. I - specifies that the widgetId parameter belongs to
*    8. M,L - the parameter offset MSB and LSB accordingly in:
*      - Flash Data Structure if W bit is 0.
*      - RAM Data Structure if W bit is 1.
*
*  Refer to the \ref group_structures section for details of the data structure
*  organization and examples of its register access.
*
*  This function writes specified value into the desired register without
*  other registers update. It is application layer responsibility to keep all
*  the data structure registers aligned. Repeated call of
*  capsense_Start() function helps aligning dependent register values.
*
* \param paramId
*  Specifies the ID of parameter to set its value.
*  A macro for the parameter ID can be found in the capsense RegisterMap header
*  file defined as capsense_<ParameterName>_PARAM_ID.

* \param value
*  Specifies the new parameter's value.
*
* \return
*  Returns the status of the operation:
*    - CY_RET_SUCCESS - The operation is successfully completed.
*    - CY_RET_BAD_PARAM - The input parameter is invalid.
*
*******************************************************************************/
cy_status capsense_SetParam(uint32 paramId, uint32 value)
{
    cy_status result;
    PARAM_ID_STRUCT pData;

    union
    {
        volatile void   * ptr;
        volatile uint8  * ptrUint8;
        volatile uint16 * ptrUint16;
        volatile uint32 * ptrUint32;
    } ptrData;

    result = DsParseParamId(paramId, &pData);

    /* verify if parameter is writable */
    if (CY_RET_SUCCESS == result)
    {
        if (0u == pData.rw)
        {
            result = CY_RET_BAD_PARAM;
        }
    }

    /* Parse and validate paramId */
    if (CY_RET_SUCCESS == result)
    {
        /* Get base address of parameter */
        ptrData.ptr = &capsense_dsRam;

        /* Add offset to base address */
        ptrData.ptrUint8 += pData.offset;

        /* Write data */
        if (DS_PARAM_TYPE_UINT8 == pData.type)
        {
            *ptrData.ptrUint8 = CY_LO8(value);
        }
        else if (DS_PARAM_TYPE_UINT16 == pData.type)
        {
            *ptrData.ptrUint16 = CY_LO16(value);
        }
        else if (DS_PARAM_TYPE_UINT32 == pData.type)
        {
            *ptrData.ptrUint32 = value;
        }
        else
        {
            /* Parameter comes here already validated. */
            CY_ASSERT(0 != 0);
        }

        #if (capsense_ENABLE == (capsense_TST_WDGT_CRC_EN || capsense_TST_GLOBAL_CRC_EN))
            /* Update widget or global CRC if self-test is enabled and parameter affects it */
            if (0u != pData.affectsCrc)
            {
                if ((ptrData.ptrUint16 >= &capsense_dsRam.csd0Config) &&
                    (ptrData.ptrUint16 <  &capsense_dsRam.glbCrc))
                {

                    #if (capsense_ENABLE == capsense_TST_GLOBAL_CRC_EN)
                        capsense_DsUpdateGlobalCrc();
                    #endif /* (capsense_ENABLE == capsense_TST_GLOBAL_CRC_EN) */

                }
                else
                {
                    #if (capsense_ENABLE == capsense_TST_WDGT_CRC_EN)
                        capsense_DsUpdateWidgetCrc((uint32)pData.widgetId);
                    #endif /* (capsense_ENABLE == capsense_TST_WDGT_CRC_EN) */
                }
            }
        #endif /* (capsense_ENABLE == (capsense_TST_WDGT_CRC_EN || capsense_TST_GLOBAL_CRC_EN)) */
    }

    return result;
}

/*******************************************************************************
* Function Name: capsense_IsAnyWidgetActive
****************************************************************************//**
*
* \brief
*  Reports if any widget has detected a touch.
*
* \details
*  This function reports if any widget has detected a touch or not by extracting
*  information from the wdgtStatus registers
*  (capsense_WDGT_STATUS<X>_VALUE). This function does not process a
*  widget but extracts processed results from the \ref group_structures.
*
* \return
*   Returns the touch detection status of all the widgets:
*     - Zero - No touch is detected in all the widgets or sensors.
*     - Non-zero - At least one widget or sensor detected a touch.
*
*******************************************************************************/
uint32 capsense_IsAnyWidgetActive(void)
{
    uint32 result = 0Lu;
    uint32 wdWord;

    for (wdWord = capsense_WDGT_STATUS_WORDS; wdWord-- > 0u;)
    {
        result |= capsense_dsRam.wdgtStatus[wdWord];
    }

    return result;
}


/*******************************************************************************
* Function Name: capsense_IsWidgetActive
****************************************************************************//**
*
* \brief
*  Reports if the specified widget detects a touch on any of its sensors.
*
* \details
*  This function reports if the specified widget has detected a touch or not by
*  extracting information from the wdgtStatus registers
*  (capsense_WDGT_STATUS<X>_VALUE). This function does not process the
*  widget but extracts processed results from the \ref group_structures.
*
* \param widgetId
*  Specifies the ID number of the widget to get its status.
*  A macro for the widget ID can be found in the capsense Configuration header
*  file defined as capsense_<WidgetName>_WDGT_ID.
*
* \return
*  Returns the touch detection status of the specified widgets:
*    - Zero - No touch is detected in the specified widget or a wrong widgetId
*      is specified.
*    - Non-zero if at least one sensor of the specified widget is active, i.e.
*      a touch is detected.
*
*******************************************************************************/
uint32 capsense_IsWidgetActive(uint32 widgetId)
{
    uint32 result = 0Lu;

    if (widgetId < capsense_TOTAL_WIDGETS)
    {
        result = capsense_GET_WIDGET_ACTIVE_STATUS(widgetId);
    }
    return result;
}


/*******************************************************************************
* Function Name: capsense_IsSensorActive
****************************************************************************//**
*
* \brief
*  Reports if the specified sensor in the widget detects a touch.
*
* \details
*  This function reports if the specified sensor in the widget has detected a
*  touch or not by extracting information from the wdgtStatus registers
*  (capsense_WDGT_STATUS<X>_VALUE). This function does not process the
*  widget or sensor but extracts processed results from the \ref group_structures.
*
*  For proximity sensors, this function returns the proximity detection status. To
*  get the finger touch status of proximity sensors, use the
*  capsense_IsProximitySensorActive() function.
*
* \param widgetId
*  Specifies the ID number of the widget.
*  A macro for the widget ID can be found in the capsense Configuration header
*  file defined as capsense_<WidgetName>_WDGT_ID.

* \param sensorId
*  Specifies the ID number of the sensor within the widget to get its touch
*  detection status.
*  A macro for the sensor ID within the specified widget can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns the touch detection status of the specified sensor / widget:
*    - Zero if no touch is detected in the specified sensor / widget or a wrong
*      widget ID / sensor ID is specified.
*    - Non-zero if the specified sensor is active i.e. touch is detected. If the
*      specific sensor belongs to a proximity widget, the proximity detection
*      status is returned.
*
*******************************************************************************/
uint32 capsense_IsSensorActive(uint32 widgetId, uint32 sensorId)
{
    uint32 result = 0Lu;

    if ((widgetId < capsense_TOTAL_WIDGETS) &&
        (sensorId < capsense_GET_SENSOR_COUNT(widgetId)))
    {
        #if (capsense_ENABLE == capsense_PROXIMITY_WIDGET_EN)
            if (capsense_WD_PROXIMITY_E ==
                (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[widgetId].wdgtType)
            {
                sensorId = capsense_PROX_STS_OFFSET(sensorId);
            }
        #endif /* (capsense_ENABLE != capsense_PROXIMITY_WIDGET_EN) */
        result = capsense_dsRam.snsStatus[widgetId] & (1Lu << sensorId);
    }

    return result;
}

#if (0u != capsense_MATRIX_WIDGET_EN)
/*******************************************************************************
* Function Name: capsense_IsMatrixButtonsActive
****************************************************************************//**
*
* \brief
*  Reports the status of the specified matrix button widget.
*
* \details
*  This function reports if the specified matrix widget has detected a touch or
*  not by extracting information from the wdgtStatus registers
*  (capsense_WDGT_STATUS<X>_VALUE for the CSD widgets and
*  capsense_SNS_STATUS<WidgetId>_VALUE for CSX widget). In addition, the
*  function provides details of the active sensor including active rows/columns
*  for the CSD widgets. This function is used only with the matrix button
*  widgets. This function does not process the widget but extracts
*  processed results from the \ref group_structures.
*
* \param widgetId
*  Specifies the ID number of the matrix button widget to check the status of its
*  sensors.
*  A macro for the widget ID can be found in the capsense Configuration header file
*  defined as capsense_<WidgetName>_WDGT_ID
*
* \return
*  Returns the touch detection status of the sensors in the specified matrix
*  buttons widget. Zero indicates that no touch is detected in the specified
*  widget or a wrong widgetId is specified.
*    1. For the matrix buttons widgets with the CSD sensing mode:
*      - Bit [31] if set, indicates that one or more sensors in the widget
*        detected a touch.
*      - Bits [30..24] are reserved
*      - Bits [23..16] indicate the logical sensor number of the sensor that
*        detected a touch. If more than one sensor detected a touch for the CSD
*        widget, no status is reported because more than one touch is invalid for the
*        CSD matrix buttons widgets.
*      - Bits [15..8] indicate the active row number.
*      - Bits [7..0] indicate the active column number.
*    2. For the matrix buttons widgets with the CSX widgets, each bit (31..0)
*      corresponds to the TX/RX intersection.
*
*******************************************************************************/
uint32 capsense_IsMatrixButtonsActive(uint32 widgetId)
{
    uint32 result = 0Lu;
    cy_status state = CY_RET_SUCCESS;
    capsense_FLASH_WD_STRUCT const *ptrFlashWdgt = 0u;

    #if (0u != capsense_CSD_MATRIX_WIDGET_EN)
        capsense_RAM_WD_CSD_MATRIX_STRUCT *wdCsdMatrix;
    #endif

    if (widgetId >= capsense_TOTAL_WIDGETS)
    {
        state = CY_RET_BAD_PARAM;
    }
    else
    {
        ptrFlashWdgt = &capsense_dsFlash.wdgtArray[widgetId];

        if ((capsense_WD_TYPE_ENUM)ptrFlashWdgt->wdgtType != capsense_WD_MATRIX_BUTTON_E)
        {
            state = CY_RET_BAD_PARAM;
        }
        else if (0u == capsense_GET_WIDGET_ACTIVE_STATUS(widgetId))
        {
            state = CY_RET_BAD_PARAM;
        }
        else
        {
            /* input parameters are OK */
        }
    }

    if (CY_RET_SUCCESS == state)
    {
        switch(capsense_GET_SENSE_METHOD(ptrFlashWdgt))
        {
        #if (0u != capsense_CSD_MATRIX_WIDGET_EN)
            case capsense_SENSE_METHOD_CSD_E:
                wdCsdMatrix = ptrFlashWdgt->ptr2WdgtRam;

                result = capsense_MATRIX_BUTTONS_TOUCHED |
                         ((uint32)wdCsdMatrix->posSnsId << 16u)  |
                         ((uint32)wdCsdMatrix->posRow   << 8u)   |
                         (uint32)wdCsdMatrix->posCol;
                break;
        #endif /* #if (0u != capsense_CSD_MATRIX_WIDGET_EN) */

        #if (0u != capsense_CSX_MATRIX_WIDGET_EN)
            case capsense_SENSE_METHOD_CSX_E:
                result = capsense_dsRam.snsStatus[widgetId];
                break;
        #endif /* #if (0u != capsense_CSX_MATRIX_WIDGET_EN) */

        default:
            CY_ASSERT(0 != 0);
            break;
        }
    }

    return result;
}
#endif /* #if (0u != capsense_MATRIX_WIDGET_EN) */

#if (0u != capsense_PROXIMITY_WIDGET_EN)
/*******************************************************************************
* Function Name: capsense_IsProximitySensorActive
****************************************************************************//**
*
* \brief
*  Reports the finger detection status of the specified proximity widget/sensor.
*
* \details
*  This function reports if the specified proximity sensor has detected a touch
*  or not by extracting information from the wdgtStatus registers
*  (capsense_SNS_STATUS<WidgetId>_VALUE). This function is used
*  only with proximity sensor widgets. This function does not process the
*  widget but extracts processed results from the \ref group_structures.
*
* \param widgetId
*  Specifies the ID number of the proximity widget.
*  A macro for the widget ID can be found in the capsense Configuration header
*  file defined as capsense_<WidgetName>_WDGT_ID
*
* \param proxId
*  Specifies the ID number of the proximity sensor within the proximity widget to
*  get its touch detection status.
*  A macro for the proximity ID within a specified widget can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_SNS<SensorNumber>_ID
*
* \return
*  Returns the status of the specified sensor of the proximity widget. Zero
*  indicates that no touch is detected in the specified sensor / widget or a
*  wrong widgetId / proxId is specified.
*    - Bits [31..2] are reserved.
*    - Bit [1] indicates that a touch is detected.
*    - Bit [0] indicates that a proximity is detected.
*
*******************************************************************************/
uint32 capsense_IsProximitySensorActive(uint32 widgetId, uint32 proxId)
{
    uint32 result = 0Lu;

    if ((widgetId < capsense_TOTAL_WIDGETS) && (proxId < capsense_GET_SENSOR_COUNT(widgetId)) &&
        (capsense_WD_PROXIMITY_E ==
            (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[widgetId].wdgtType))
    {
        result = capsense_dsRam.snsStatus[widgetId];
        result >>= capsense_PROX_STS_OFFSET(proxId);
        result &=  capsense_PROX_STS_MASK;
    }

    return result;
}
#endif /* #if (0u != capsense_PROXIMITY_WIDGET_EN) */


#if (0u != capsense_SLIDER_WIDGET_EN)
/*******************************************************************************
* Function Name: capsense_GetCentroidPos
****************************************************************************//**
*
* \brief
*  Reports the centroid position for the specified slider widget.
*
* \details
*  This function reports the centroid value of a specified radial or linear
*  slider widget by extracting information from the wdgtStatus registers
*  (capsense_<WidgetName>_POSITION<X>_VALUE). This function is
*  used only with radial or linear slider widgets. This function does not
*  process the widget but extracts processed results from the \ref group_structures.
*
* \param widgetId
*  Specifies the ID number of a slider widget to get the centroid of the
*  detected touch.
*  A macro for the widget ID can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_WDGT_ID
*
* \return
*  Returns the centroid position of a specified slider widget:
*    - The centroid position if a touch is detected.
*    - capsense_SLIDER_NO_TOUCH - No touch is detected or a wrong
*      widgetId is specified.
*
*******************************************************************************/
uint32 capsense_GetCentroidPos(uint32 widgetId)
{
    uint32 result = capsense_SLIDER_NO_TOUCH;
    capsense_RAM_WD_SLIDER_STRUCT *wdSlider;

    if ((widgetId < capsense_TOTAL_WIDGETS) &&
        ((capsense_WD_LINEAR_SLIDER_E ==
            (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[widgetId].wdgtType) ||
         (capsense_WD_RADIAL_SLIDER_E ==
            (capsense_WD_TYPE_ENUM)capsense_dsFlash.wdgtArray[widgetId].wdgtType)))
    {
        wdSlider = capsense_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;
        result = (uint32)wdSlider->position[0u];
    }

    return result;
}
#endif /* #if (0u != capsense_SLIDER_WIDGET_EN) */


#if (0u != capsense_TOUCHPAD_WIDGET_EN)
/*******************************************************************************
* Function Name: capsense_GetXYCoordinates
****************************************************************************//**
*
* \brief
*  Reports the X/Y position detected for the specified touchpad widget.
*
* \details
*  This function reports a touch position (X and Y coordinates) value of a
*  specified touchpad widget by extracting information from the wdgtStatus
*  registers (capsense_<WidgetName>_POS_Y_VALUE). This function should
*  be used only with the touchpad widgets. This function does not process the
*  widget but extracts processed results from the \ref group_structures.
*
* \param widgetId
*  Specifies the ID number of a touchpad widget to get the X/Y position of a
*  detected touch.
*  A macro for the widget ID can be found in the
*  capsense Configuration header file defined as
*  capsense_<WidgetName>_WDGT_ID.
*
* \return
*  Returns the touch position of a specified touchpad widget:
*    1. If a touch is detected:
*      - Bits [31..16] indicate the Y coordinate.
*      - Bits [15..0] indicate the X coordinate.
*    2. If no touch is detected or a wrong widgetId is specified:
*      - capsense_TOUCHPAD_NO_TOUCH.
*
*******************************************************************************/
uint32 capsense_GetXYCoordinates(uint32 widgetId)
{
    capsense_FLASH_WD_STRUCT const *ptrFlashWdgt = NULL;

    #if (0u != capsense_CSD_TOUCHPAD_WIDGET_EN)
        capsense_RAM_WD_CSD_TOUCHPAD_STRUCT *wdCsdTouchpad;
    #endif /* #if (0u != capsense_CSD_TOUCHPAD_WIDGET_EN) */

    #if (0u != capsense_CSX_TOUCHPAD_WIDGET_EN)
        capsense_RAM_WD_CSX_TOUCHPAD_STRUCT *wdCsxTouchpad;
    #endif /* #if (0u != capsense_CSX_TOUCHPAD_WIDGET_EN) */

    cy_status state = CY_RET_SUCCESS;
    uint32 result = capsense_TOUCHPAD_NO_TOUCH;

    if (widgetId >= capsense_TOTAL_WIDGETS)
    {
        state = CY_RET_BAD_PARAM;
    }
    else
    {
        ptrFlashWdgt = &capsense_dsFlash.wdgtArray[widgetId];

        if ((capsense_WD_TYPE_ENUM)ptrFlashWdgt->wdgtType != capsense_WD_TOUCHPAD_E)
        {
            state = CY_RET_BAD_PARAM;
        }
    }

    if (CY_RET_SUCCESS == state)
    {
        switch(capsense_GET_SENSE_METHOD(ptrFlashWdgt))
        {
        #if (0u != capsense_CSD_TOUCHPAD_WIDGET_EN)
            case capsense_SENSE_METHOD_CSD_E:
                wdCsdTouchpad = ptrFlashWdgt->ptr2WdgtRam;
                #if (capsense_ENABLE == capsense_CENTROID_5X5_CSD_EN)
                    #if (capsense_ENABLE == capsense_CENTROID_3X3_CSD_EN)
                        if (0 != (ptrFlashWdgt->staticConfig & capsense_CENTROID_5X5_MASK))
                        {
                    #endif
                        result = ((uint32)wdCsdTouchpad->position.pos[0u].x) |
                                 ((uint32)wdCsdTouchpad->position.pos[0u].y << 16u);
                    #if (capsense_ENABLE == capsense_CENTROID_3X3_CSD_EN)
                        }
                    #endif
                #endif

                #if (capsense_ENABLE == capsense_CENTROID_3X3_CSD_EN)
                    #if (capsense_ENABLE == capsense_CENTROID_5X5_CSD_EN)
                        if (0 != (ptrFlashWdgt->staticConfig & capsense_CENTROID_3X3_MASK))
                        {
                    #endif
                        result = ((uint32)wdCsdTouchpad->posX) |
                                 ((uint32)wdCsdTouchpad->posY << 16u);
                    #if (capsense_ENABLE == capsense_CENTROID_5X5_CSD_EN)
                        }
                    #endif
                #endif

                break;
        #endif /* #if (0u != capsense_CSD_TOUCHPAD_WIDGET_EN) */

        #if (0u != capsense_CSX_TOUCHPAD_WIDGET_EN)
            case capsense_SENSE_METHOD_CSX_E:
                wdCsxTouchpad = ptrFlashWdgt->ptr2WdgtRam;

                result = ((uint32)wdCsxTouchpad->touch[0u].x) |
                         ((uint32)wdCsxTouchpad->touch[0u].y << 16u);
                break;
        #endif /* #if (0u != capsense_CSX_TOUCHPAD_WIDGET_EN) */

        default:
            CY_ASSERT(0 != 0);
            break;
        }
    }

    return result;
}
#endif /* #if (0u != capsense_TOUCHPAD_WIDGET_EN) */


/* [] END OF FILE */
