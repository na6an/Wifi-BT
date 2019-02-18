/***************************************************************************//**
* \file capsense_RegisterMap.h
* \version 2.0
*
* \brief
*   This file provides the definitions for the Component data structure register.
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

#if !defined(CY_SENSE_capsense_REGISTER_MAP_H)
#define CY_SENSE_capsense_REGISTER_MAP_H

#include "syslib/cy_syslib.h"
#include "capsense_Configuration.h"
#include "capsense_Structure.h"

/*****************************************************************************/
/* RAM Data structure register definitions                                   */
/*****************************************************************************/
#define capsense_CONFIG_ID_VALUE                            (capsense_dsRam.configId)
#define capsense_CONFIG_ID_OFFSET                           (0u)
#define capsense_CONFIG_ID_SIZE                             (2u)
#define capsense_CONFIG_ID_PARAM_ID                         (0x87000000u)

#define capsense_DEVICE_ID_VALUE                            (capsense_dsRam.deviceId)
#define capsense_DEVICE_ID_OFFSET                           (2u)
#define capsense_DEVICE_ID_SIZE                             (2u)
#define capsense_DEVICE_ID_PARAM_ID                         (0x8B000002u)

#define capsense_HW_CLOCK_VALUE                             (capsense_dsRam.hwClock)
#define capsense_HW_CLOCK_OFFSET                            (4u)
#define capsense_HW_CLOCK_SIZE                              (2u)
#define capsense_HW_CLOCK_PARAM_ID                          (0x86000004u)

#define capsense_TUNER_CMD_VALUE                            (capsense_dsRam.tunerCmd)
#define capsense_TUNER_CMD_OFFSET                           (6u)
#define capsense_TUNER_CMD_SIZE                             (2u)
#define capsense_TUNER_CMD_PARAM_ID                         (0xA1000006u)

#define capsense_SCAN_COUNTER_VALUE                         (capsense_dsRam.scanCounter)
#define capsense_SCAN_COUNTER_OFFSET                        (8u)
#define capsense_SCAN_COUNTER_SIZE                          (2u)
#define capsense_SCAN_COUNTER_PARAM_ID                      (0x85000008u)

#define capsense_STATUS_VALUE                               (capsense_dsRam.status)
#define capsense_STATUS_OFFSET                              (12u)
#define capsense_STATUS_SIZE                                (4u)
#define capsense_STATUS_PARAM_ID                            (0xCB00000Cu)

#define capsense_WDGT_ENABLE0_VALUE                         (capsense_dsRam.wdgtEnable[0u])
#define capsense_WDGT_ENABLE0_OFFSET                        (16u)
#define capsense_WDGT_ENABLE0_SIZE                          (4u)
#define capsense_WDGT_ENABLE0_PARAM_ID                      (0xE6000010u)

#define capsense_WDGT_STATUS0_VALUE                         (capsense_dsRam.wdgtStatus[0u])
#define capsense_WDGT_STATUS0_OFFSET                        (20u)
#define capsense_WDGT_STATUS0_SIZE                          (4u)
#define capsense_WDGT_STATUS0_PARAM_ID                      (0xCC000014u)

#define capsense_SNS_STATUS0_VALUE                          (capsense_dsRam.snsStatus[0u])
#define capsense_SNS_STATUS0_OFFSET                         (24u)
#define capsense_SNS_STATUS0_SIZE                           (1u)
#define capsense_SNS_STATUS0_PARAM_ID                       (0x48000018u)

#define capsense_CSD0_CONFIG_VALUE                          (capsense_dsRam.csd0Config)
#define capsense_CSD0_CONFIG_OFFSET                         (26u)
#define capsense_CSD0_CONFIG_SIZE                           (2u)
#define capsense_CSD0_CONFIG_PARAM_ID                       (0xAA80001Au)

#define capsense_MOD_CSD_CLK_VALUE                          (capsense_dsRam.modCsdClk)
#define capsense_MOD_CSD_CLK_OFFSET                         (28u)
#define capsense_MOD_CSD_CLK_SIZE                           (1u)
#define capsense_MOD_CSD_CLK_PARAM_ID                       (0x6F80001Cu)

#define capsense_LINEARSLIDER0_RESOLUTION_VALUE             (capsense_dsRam.wdgtList.linearslider0.resolution)
#define capsense_LINEARSLIDER0_RESOLUTION_OFFSET            (30u)
#define capsense_LINEARSLIDER0_RESOLUTION_SIZE              (2u)
#define capsense_LINEARSLIDER0_RESOLUTION_PARAM_ID          (0x8080001Eu)

#define capsense_LINEARSLIDER0_FINGER_TH_VALUE              (capsense_dsRam.wdgtList.linearslider0.fingerTh)
#define capsense_LINEARSLIDER0_FINGER_TH_OFFSET             (32u)
#define capsense_LINEARSLIDER0_FINGER_TH_SIZE               (2u)
#define capsense_LINEARSLIDER0_FINGER_TH_PARAM_ID           (0x80800020u)

#define capsense_LINEARSLIDER0_NOISE_TH_VALUE               (capsense_dsRam.wdgtList.linearslider0.noiseTh)
#define capsense_LINEARSLIDER0_NOISE_TH_OFFSET              (34u)
#define capsense_LINEARSLIDER0_NOISE_TH_SIZE                (1u)
#define capsense_LINEARSLIDER0_NOISE_TH_PARAM_ID            (0x44800022u)

#define capsense_LINEARSLIDER0_NNOISE_TH_VALUE              (capsense_dsRam.wdgtList.linearslider0.nNoiseTh)
#define capsense_LINEARSLIDER0_NNOISE_TH_OFFSET             (35u)
#define capsense_LINEARSLIDER0_NNOISE_TH_SIZE               (1u)
#define capsense_LINEARSLIDER0_NNOISE_TH_PARAM_ID           (0x42800023u)

#define capsense_LINEARSLIDER0_HYSTERESIS_VALUE             (capsense_dsRam.wdgtList.linearslider0.hysteresis)
#define capsense_LINEARSLIDER0_HYSTERESIS_OFFSET            (36u)
#define capsense_LINEARSLIDER0_HYSTERESIS_SIZE              (1u)
#define capsense_LINEARSLIDER0_HYSTERESIS_PARAM_ID          (0x49800024u)

#define capsense_LINEARSLIDER0_ON_DEBOUNCE_VALUE            (capsense_dsRam.wdgtList.linearslider0.onDebounce)
#define capsense_LINEARSLIDER0_ON_DEBOUNCE_OFFSET           (37u)
#define capsense_LINEARSLIDER0_ON_DEBOUNCE_SIZE             (1u)
#define capsense_LINEARSLIDER0_ON_DEBOUNCE_PARAM_ID         (0x4F800025u)

#define capsense_LINEARSLIDER0_LOW_BSLN_RST_VALUE           (capsense_dsRam.wdgtList.linearslider0.lowBslnRst)
#define capsense_LINEARSLIDER0_LOW_BSLN_RST_OFFSET          (38u)
#define capsense_LINEARSLIDER0_LOW_BSLN_RST_SIZE            (1u)
#define capsense_LINEARSLIDER0_LOW_BSLN_RST_PARAM_ID        (0x45800026u)

#define capsense_LINEARSLIDER0_IDAC_MOD0_VALUE              (capsense_dsRam.wdgtList.linearslider0.idacMod[0u])
#define capsense_LINEARSLIDER0_IDAC_MOD0_OFFSET             (39u)
#define capsense_LINEARSLIDER0_IDAC_MOD0_SIZE               (1u)
#define capsense_LINEARSLIDER0_IDAC_MOD0_PARAM_ID           (0x4E000027u)

#define capsense_LINEARSLIDER0_SNS_CLK_VALUE                (capsense_dsRam.wdgtList.linearslider0.snsClk)
#define capsense_LINEARSLIDER0_SNS_CLK_OFFSET               (40u)
#define capsense_LINEARSLIDER0_SNS_CLK_SIZE                 (2u)
#define capsense_LINEARSLIDER0_SNS_CLK_PARAM_ID             (0x82800028u)

#define capsense_LINEARSLIDER0_SNS_CLK_SOURCE_VALUE         (capsense_dsRam.wdgtList.linearslider0.snsClkSource)
#define capsense_LINEARSLIDER0_SNS_CLK_SOURCE_OFFSET        (42u)
#define capsense_LINEARSLIDER0_SNS_CLK_SOURCE_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS_CLK_SOURCE_PARAM_ID      (0x4680002Au)

#define capsense_LINEARSLIDER0_FINGER_CAP_VALUE             (capsense_dsRam.wdgtList.linearslider0.fingerCap)
#define capsense_LINEARSLIDER0_FINGER_CAP_OFFSET            (44u)
#define capsense_LINEARSLIDER0_FINGER_CAP_SIZE              (2u)
#define capsense_LINEARSLIDER0_FINGER_CAP_PARAM_ID          (0xA500002Cu)

#define capsense_LINEARSLIDER0_SIGPFC_VALUE                 (capsense_dsRam.wdgtList.linearslider0.sigPFC)
#define capsense_LINEARSLIDER0_SIGPFC_OFFSET                (46u)
#define capsense_LINEARSLIDER0_SIGPFC_SIZE                  (2u)
#define capsense_LINEARSLIDER0_SIGPFC_PARAM_ID              (0xA900002Eu)

#define capsense_LINEARSLIDER0_POSITION0_VALUE              (capsense_dsRam.wdgtList.linearslider0.position[0u])
#define capsense_LINEARSLIDER0_POSITION0_OFFSET             (48u)
#define capsense_LINEARSLIDER0_POSITION0_SIZE               (2u)
#define capsense_LINEARSLIDER0_POSITION0_PARAM_ID           (0x88000030u)

#define capsense_LINEARSLIDER0_SNS0_RAW0_VALUE              (capsense_dsRam.snsList.linearslider0[0u].raw[0u])
#define capsense_LINEARSLIDER0_SNS0_RAW0_OFFSET             (50u)
#define capsense_LINEARSLIDER0_SNS0_RAW0_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS0_RAW0_PARAM_ID           (0x84000032u)

#define capsense_LINEARSLIDER0_SNS0_BSLN0_VALUE             (capsense_dsRam.snsList.linearslider0[0u].bsln[0u])
#define capsense_LINEARSLIDER0_SNS0_BSLN0_OFFSET            (52u)
#define capsense_LINEARSLIDER0_SNS0_BSLN0_SIZE              (2u)
#define capsense_LINEARSLIDER0_SNS0_BSLN0_PARAM_ID          (0x89000034u)

#define capsense_LINEARSLIDER0_SNS0_BSLN_EXT0_VALUE         (capsense_dsRam.snsList.linearslider0[0u].bslnExt[0u])
#define capsense_LINEARSLIDER0_SNS0_BSLN_EXT0_OFFSET        (54u)
#define capsense_LINEARSLIDER0_SNS0_BSLN_EXT0_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS0_BSLN_EXT0_PARAM_ID      (0x4D000036u)

#define capsense_LINEARSLIDER0_SNS0_DIFF_VALUE              (capsense_dsRam.snsList.linearslider0[0u].diff)
#define capsense_LINEARSLIDER0_SNS0_DIFF_OFFSET             (56u)
#define capsense_LINEARSLIDER0_SNS0_DIFF_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS0_DIFF_PARAM_ID           (0x8A000038u)

#define capsense_LINEARSLIDER0_SNS0_NEG_BSLN_RST_CNT0_VALUE (capsense_dsRam.snsList.linearslider0[0u].negBslnRstCnt[0u])
#define capsense_LINEARSLIDER0_SNS0_NEG_BSLN_RST_CNT0_OFFSET (58u)
#define capsense_LINEARSLIDER0_SNS0_NEG_BSLN_RST_CNT0_SIZE  (1u)
#define capsense_LINEARSLIDER0_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID (0x4E00003Au)

#define capsense_LINEARSLIDER0_SNS0_IDAC_COMP0_VALUE        (capsense_dsRam.snsList.linearslider0[0u].idacComp[0u])
#define capsense_LINEARSLIDER0_SNS0_IDAC_COMP0_OFFSET       (59u)
#define capsense_LINEARSLIDER0_SNS0_IDAC_COMP0_SIZE         (1u)
#define capsense_LINEARSLIDER0_SNS0_IDAC_COMP0_PARAM_ID     (0x4800003Bu)

#define capsense_LINEARSLIDER0_SNS1_RAW0_VALUE              (capsense_dsRam.snsList.linearslider0[1u].raw[0u])
#define capsense_LINEARSLIDER0_SNS1_RAW0_OFFSET             (60u)
#define capsense_LINEARSLIDER0_SNS1_RAW0_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS1_RAW0_PARAM_ID           (0x8B00003Cu)

#define capsense_LINEARSLIDER0_SNS1_BSLN0_VALUE             (capsense_dsRam.snsList.linearslider0[1u].bsln[0u])
#define capsense_LINEARSLIDER0_SNS1_BSLN0_OFFSET            (62u)
#define capsense_LINEARSLIDER0_SNS1_BSLN0_SIZE              (2u)
#define capsense_LINEARSLIDER0_SNS1_BSLN0_PARAM_ID          (0x8700003Eu)

#define capsense_LINEARSLIDER0_SNS1_BSLN_EXT0_VALUE         (capsense_dsRam.snsList.linearslider0[1u].bslnExt[0u])
#define capsense_LINEARSLIDER0_SNS1_BSLN_EXT0_OFFSET        (64u)
#define capsense_LINEARSLIDER0_SNS1_BSLN_EXT0_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS1_BSLN_EXT0_PARAM_ID      (0x42000040u)

#define capsense_LINEARSLIDER0_SNS1_DIFF_VALUE              (capsense_dsRam.snsList.linearslider0[1u].diff)
#define capsense_LINEARSLIDER0_SNS1_DIFF_OFFSET             (66u)
#define capsense_LINEARSLIDER0_SNS1_DIFF_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS1_DIFF_PARAM_ID           (0x86000042u)

#define capsense_LINEARSLIDER0_SNS1_NEG_BSLN_RST_CNT0_VALUE (capsense_dsRam.snsList.linearslider0[1u].negBslnRstCnt[0u])
#define capsense_LINEARSLIDER0_SNS1_NEG_BSLN_RST_CNT0_OFFSET (68u)
#define capsense_LINEARSLIDER0_SNS1_NEG_BSLN_RST_CNT0_SIZE  (1u)
#define capsense_LINEARSLIDER0_SNS1_NEG_BSLN_RST_CNT0_PARAM_ID (0x43000044u)

#define capsense_LINEARSLIDER0_SNS1_IDAC_COMP0_VALUE        (capsense_dsRam.snsList.linearslider0[1u].idacComp[0u])
#define capsense_LINEARSLIDER0_SNS1_IDAC_COMP0_OFFSET       (69u)
#define capsense_LINEARSLIDER0_SNS1_IDAC_COMP0_SIZE         (1u)
#define capsense_LINEARSLIDER0_SNS1_IDAC_COMP0_PARAM_ID     (0x45000045u)

#define capsense_LINEARSLIDER0_SNS2_RAW0_VALUE              (capsense_dsRam.snsList.linearslider0[2u].raw[0u])
#define capsense_LINEARSLIDER0_SNS2_RAW0_OFFSET             (70u)
#define capsense_LINEARSLIDER0_SNS2_RAW0_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS2_RAW0_PARAM_ID           (0x87000046u)

#define capsense_LINEARSLIDER0_SNS2_BSLN0_VALUE             (capsense_dsRam.snsList.linearslider0[2u].bsln[0u])
#define capsense_LINEARSLIDER0_SNS2_BSLN0_OFFSET            (72u)
#define capsense_LINEARSLIDER0_SNS2_BSLN0_SIZE              (2u)
#define capsense_LINEARSLIDER0_SNS2_BSLN0_PARAM_ID          (0x88000048u)

#define capsense_LINEARSLIDER0_SNS2_BSLN_EXT0_VALUE         (capsense_dsRam.snsList.linearslider0[2u].bslnExt[0u])
#define capsense_LINEARSLIDER0_SNS2_BSLN_EXT0_OFFSET        (74u)
#define capsense_LINEARSLIDER0_SNS2_BSLN_EXT0_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS2_BSLN_EXT0_PARAM_ID      (0x4C00004Au)

#define capsense_LINEARSLIDER0_SNS2_DIFF_VALUE              (capsense_dsRam.snsList.linearslider0[2u].diff)
#define capsense_LINEARSLIDER0_SNS2_DIFF_OFFSET             (76u)
#define capsense_LINEARSLIDER0_SNS2_DIFF_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS2_DIFF_PARAM_ID           (0x8900004Cu)

#define capsense_LINEARSLIDER0_SNS2_NEG_BSLN_RST_CNT0_VALUE (capsense_dsRam.snsList.linearslider0[2u].negBslnRstCnt[0u])
#define capsense_LINEARSLIDER0_SNS2_NEG_BSLN_RST_CNT0_OFFSET (78u)
#define capsense_LINEARSLIDER0_SNS2_NEG_BSLN_RST_CNT0_SIZE  (1u)
#define capsense_LINEARSLIDER0_SNS2_NEG_BSLN_RST_CNT0_PARAM_ID (0x4D00004Eu)

#define capsense_LINEARSLIDER0_SNS2_IDAC_COMP0_VALUE        (capsense_dsRam.snsList.linearslider0[2u].idacComp[0u])
#define capsense_LINEARSLIDER0_SNS2_IDAC_COMP0_OFFSET       (79u)
#define capsense_LINEARSLIDER0_SNS2_IDAC_COMP0_SIZE         (1u)
#define capsense_LINEARSLIDER0_SNS2_IDAC_COMP0_PARAM_ID     (0x4B00004Fu)

#define capsense_LINEARSLIDER0_SNS3_RAW0_VALUE              (capsense_dsRam.snsList.linearslider0[3u].raw[0u])
#define capsense_LINEARSLIDER0_SNS3_RAW0_OFFSET             (80u)
#define capsense_LINEARSLIDER0_SNS3_RAW0_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS3_RAW0_PARAM_ID           (0x8F000050u)

#define capsense_LINEARSLIDER0_SNS3_BSLN0_VALUE             (capsense_dsRam.snsList.linearslider0[3u].bsln[0u])
#define capsense_LINEARSLIDER0_SNS3_BSLN0_OFFSET            (82u)
#define capsense_LINEARSLIDER0_SNS3_BSLN0_SIZE              (2u)
#define capsense_LINEARSLIDER0_SNS3_BSLN0_PARAM_ID          (0x83000052u)

#define capsense_LINEARSLIDER0_SNS3_BSLN_EXT0_VALUE         (capsense_dsRam.snsList.linearslider0[3u].bslnExt[0u])
#define capsense_LINEARSLIDER0_SNS3_BSLN_EXT0_OFFSET        (84u)
#define capsense_LINEARSLIDER0_SNS3_BSLN_EXT0_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS3_BSLN_EXT0_PARAM_ID      (0x46000054u)

#define capsense_LINEARSLIDER0_SNS3_DIFF_VALUE              (capsense_dsRam.snsList.linearslider0[3u].diff)
#define capsense_LINEARSLIDER0_SNS3_DIFF_OFFSET             (86u)
#define capsense_LINEARSLIDER0_SNS3_DIFF_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS3_DIFF_PARAM_ID           (0x82000056u)

#define capsense_LINEARSLIDER0_SNS3_NEG_BSLN_RST_CNT0_VALUE (capsense_dsRam.snsList.linearslider0[3u].negBslnRstCnt[0u])
#define capsense_LINEARSLIDER0_SNS3_NEG_BSLN_RST_CNT0_OFFSET (88u)
#define capsense_LINEARSLIDER0_SNS3_NEG_BSLN_RST_CNT0_SIZE  (1u)
#define capsense_LINEARSLIDER0_SNS3_NEG_BSLN_RST_CNT0_PARAM_ID (0x45000058u)

#define capsense_LINEARSLIDER0_SNS3_IDAC_COMP0_VALUE        (capsense_dsRam.snsList.linearslider0[3u].idacComp[0u])
#define capsense_LINEARSLIDER0_SNS3_IDAC_COMP0_OFFSET       (89u)
#define capsense_LINEARSLIDER0_SNS3_IDAC_COMP0_SIZE         (1u)
#define capsense_LINEARSLIDER0_SNS3_IDAC_COMP0_PARAM_ID     (0x43000059u)

#define capsense_LINEARSLIDER0_SNS4_RAW0_VALUE              (capsense_dsRam.snsList.linearslider0[4u].raw[0u])
#define capsense_LINEARSLIDER0_SNS4_RAW0_OFFSET             (90u)
#define capsense_LINEARSLIDER0_SNS4_RAW0_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS4_RAW0_PARAM_ID           (0x8100005Au)

#define capsense_LINEARSLIDER0_SNS4_BSLN0_VALUE             (capsense_dsRam.snsList.linearslider0[4u].bsln[0u])
#define capsense_LINEARSLIDER0_SNS4_BSLN0_OFFSET            (92u)
#define capsense_LINEARSLIDER0_SNS4_BSLN0_SIZE              (2u)
#define capsense_LINEARSLIDER0_SNS4_BSLN0_PARAM_ID          (0x8C00005Cu)

#define capsense_LINEARSLIDER0_SNS4_BSLN_EXT0_VALUE         (capsense_dsRam.snsList.linearslider0[4u].bslnExt[0u])
#define capsense_LINEARSLIDER0_SNS4_BSLN_EXT0_OFFSET        (94u)
#define capsense_LINEARSLIDER0_SNS4_BSLN_EXT0_SIZE          (1u)
#define capsense_LINEARSLIDER0_SNS4_BSLN_EXT0_PARAM_ID      (0x4800005Eu)

#define capsense_LINEARSLIDER0_SNS4_DIFF_VALUE              (capsense_dsRam.snsList.linearslider0[4u].diff)
#define capsense_LINEARSLIDER0_SNS4_DIFF_OFFSET             (96u)
#define capsense_LINEARSLIDER0_SNS4_DIFF_SIZE               (2u)
#define capsense_LINEARSLIDER0_SNS4_DIFF_PARAM_ID           (0x80000060u)

#define capsense_LINEARSLIDER0_SNS4_NEG_BSLN_RST_CNT0_VALUE (capsense_dsRam.snsList.linearslider0[4u].negBslnRstCnt[0u])
#define capsense_LINEARSLIDER0_SNS4_NEG_BSLN_RST_CNT0_OFFSET (98u)
#define capsense_LINEARSLIDER0_SNS4_NEG_BSLN_RST_CNT0_SIZE  (1u)
#define capsense_LINEARSLIDER0_SNS4_NEG_BSLN_RST_CNT0_PARAM_ID (0x44000062u)

#define capsense_LINEARSLIDER0_SNS4_IDAC_COMP0_VALUE        (capsense_dsRam.snsList.linearslider0[4u].idacComp[0u])
#define capsense_LINEARSLIDER0_SNS4_IDAC_COMP0_OFFSET       (99u)
#define capsense_LINEARSLIDER0_SNS4_IDAC_COMP0_SIZE         (1u)
#define capsense_LINEARSLIDER0_SNS4_IDAC_COMP0_PARAM_ID     (0x42000063u)

#define capsense_SNR_TEST_WIDGET_ID_VALUE                   (capsense_dsRam.snrTestWidgetId)
#define capsense_SNR_TEST_WIDGET_ID_OFFSET                  (100u)
#define capsense_SNR_TEST_WIDGET_ID_SIZE                    (1u)
#define capsense_SNR_TEST_WIDGET_ID_PARAM_ID                (0x62000064u)

#define capsense_SNR_TEST_SENSOR_ID_VALUE                   (capsense_dsRam.snrTestSensorId)
#define capsense_SNR_TEST_SENSOR_ID_OFFSET                  (101u)
#define capsense_SNR_TEST_SENSOR_ID_SIZE                    (1u)
#define capsense_SNR_TEST_SENSOR_ID_PARAM_ID                (0x64000065u)

#define capsense_SNR_TEST_SCAN_COUNTER_VALUE                (capsense_dsRam.snrTestScanCounter)
#define capsense_SNR_TEST_SCAN_COUNTER_OFFSET               (102u)
#define capsense_SNR_TEST_SCAN_COUNTER_SIZE                 (2u)
#define capsense_SNR_TEST_SCAN_COUNTER_PARAM_ID             (0x8D000066u)

#define capsense_SNR_TEST_RAW_COUNT0_VALUE                  (capsense_dsRam.snrTestRawCount[0u])
#define capsense_SNR_TEST_RAW_COUNT0_OFFSET                 (104u)
#define capsense_SNR_TEST_RAW_COUNT0_SIZE                   (2u)
#define capsense_SNR_TEST_RAW_COUNT0_PARAM_ID               (0x82000068u)


/*****************************************************************************/
/* Flash Data structure register definitions                                 */
/*****************************************************************************/
#define capsense_LINEARSLIDER0_PTR2SNS_FLASH_VALUE          (capsense_dsFlash.wdgtArray[0].ptr2SnsFlash)
#define capsense_LINEARSLIDER0_PTR2SNS_FLASH_OFFSET         (0u)
#define capsense_LINEARSLIDER0_PTR2SNS_FLASH_SIZE           (4u)
#define capsense_LINEARSLIDER0_PTR2SNS_FLASH_PARAM_ID       (0xD1000000u)

#define capsense_LINEARSLIDER0_PTR2WD_RAM_VALUE             (capsense_dsFlash.wdgtArray[0].ptr2WdgtRam)
#define capsense_LINEARSLIDER0_PTR2WD_RAM_OFFSET            (4u)
#define capsense_LINEARSLIDER0_PTR2WD_RAM_SIZE              (4u)
#define capsense_LINEARSLIDER0_PTR2WD_RAM_PARAM_ID          (0xD0000004u)

#define capsense_LINEARSLIDER0_PTR2SNS_RAM_VALUE            (capsense_dsFlash.wdgtArray[0].ptr2SnsRam)
#define capsense_LINEARSLIDER0_PTR2SNS_RAM_OFFSET           (8u)
#define capsense_LINEARSLIDER0_PTR2SNS_RAM_SIZE             (4u)
#define capsense_LINEARSLIDER0_PTR2SNS_RAM_PARAM_ID         (0xD3000008u)

#define capsense_LINEARSLIDER0_PTR2FLTR_HISTORY_VALUE       (capsense_dsFlash.wdgtArray[0].ptr2FltrHistory)
#define capsense_LINEARSLIDER0_PTR2FLTR_HISTORY_OFFSET      (12u)
#define capsense_LINEARSLIDER0_PTR2FLTR_HISTORY_SIZE        (4u)
#define capsense_LINEARSLIDER0_PTR2FLTR_HISTORY_PARAM_ID    (0xD200000Cu)

#define capsense_LINEARSLIDER0_PTR2DEBOUNCE_VALUE           (capsense_dsFlash.wdgtArray[0].ptr2DebounceArr)
#define capsense_LINEARSLIDER0_PTR2DEBOUNCE_OFFSET          (16u)
#define capsense_LINEARSLIDER0_PTR2DEBOUNCE_SIZE            (4u)
#define capsense_LINEARSLIDER0_PTR2DEBOUNCE_PARAM_ID        (0xD4000010u)

#define capsense_LINEARSLIDER0_STATIC_CONFIG_VALUE          (capsense_dsFlash.wdgtArray[0].staticConfig)
#define capsense_LINEARSLIDER0_STATIC_CONFIG_OFFSET         (20u)
#define capsense_LINEARSLIDER0_STATIC_CONFIG_SIZE           (4u)
#define capsense_LINEARSLIDER0_STATIC_CONFIG_PARAM_ID       (0xD5000014u)

#define capsense_LINEARSLIDER0_TOTAL_NUM_SNS_VALUE          (capsense_dsFlash.wdgtArray[0].totalNumSns)
#define capsense_LINEARSLIDER0_TOTAL_NUM_SNS_OFFSET         (24u)
#define capsense_LINEARSLIDER0_TOTAL_NUM_SNS_SIZE           (2u)
#define capsense_LINEARSLIDER0_TOTAL_NUM_SNS_PARAM_ID       (0x99000018u)

#define capsense_LINEARSLIDER0_TYPE_VALUE                   (capsense_dsFlash.wdgtArray[0].wdgtType)
#define capsense_LINEARSLIDER0_TYPE_OFFSET                  (26u)
#define capsense_LINEARSLIDER0_TYPE_SIZE                    (1u)
#define capsense_LINEARSLIDER0_TYPE_PARAM_ID                (0x5D00001Au)

#define capsense_LINEARSLIDER0_NUM_COLS_VALUE               (capsense_dsFlash.wdgtArray[0].numCols)
#define capsense_LINEARSLIDER0_NUM_COLS_OFFSET              (27u)
#define capsense_LINEARSLIDER0_NUM_COLS_SIZE                (1u)
#define capsense_LINEARSLIDER0_NUM_COLS_PARAM_ID            (0x5B00001Bu)

#define capsense_LINEARSLIDER0_X_RESOLUTION_VALUE           (capsense_dsFlash.wdgtArray[0].xResolution)
#define capsense_LINEARSLIDER0_X_RESOLUTION_OFFSET          (28u)
#define capsense_LINEARSLIDER0_X_RESOLUTION_SIZE            (2u)
#define capsense_LINEARSLIDER0_X_RESOLUTION_PARAM_ID        (0x9800001Cu)

#define capsense_LINEARSLIDER0_X_CENT_MULT_VALUE            (capsense_dsFlash.wdgtArray[0].xCentroidMultiplier)
#define capsense_LINEARSLIDER0_X_CENT_MULT_OFFSET           (32u)
#define capsense_LINEARSLIDER0_X_CENT_MULT_SIZE             (4u)
#define capsense_LINEARSLIDER0_X_CENT_MULT_PARAM_ID         (0xDB000020u)

#define capsense_LINEARSLIDER0_PTR2NOISE_ENVLP_VALUE        (capsense_dsFlash.wdgtArray[0].ptr2NoiseEnvlp)
#define capsense_LINEARSLIDER0_PTR2NOISE_ENVLP_OFFSET       (36u)
#define capsense_LINEARSLIDER0_PTR2NOISE_ENVLP_SIZE         (4u)
#define capsense_LINEARSLIDER0_PTR2NOISE_ENVLP_PARAM_ID     (0xDA000024u)

#define capsense_LINEARSLIDER0_IIR_FILTER_COEFF_VALUE       (capsense_dsFlash.wdgtArray[0].iirFilterCoeff)
#define capsense_LINEARSLIDER0_IIR_FILTER_COEFF_OFFSET      (40u)
#define capsense_LINEARSLIDER0_IIR_FILTER_COEFF_SIZE        (1u)
#define capsense_LINEARSLIDER0_IIR_FILTER_COEFF_PARAM_ID    (0x5E000028u)


#endif /* End CY_SENSE_capsense_REGISTER_MAP_H */


/* [] END OF FILE */
