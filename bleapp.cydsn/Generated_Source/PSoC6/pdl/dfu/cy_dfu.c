/***************************************************************************//**
* \file cy_dfu.c
* \version 3.0
*
*  This file provides the implementation of Cypress DFU SDK.
*
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <string.h>
#include "cy_dfu.h"


#define CySoftwareReset() NVIC_SystemReset()

/** \cond INTERNAL */
CY_SECTION(".cy_boot_noinit.appId") __USED static uint8_t cy_dfu_appId;

/* The timeout for Cy_DFU_Continue(), in milliseconds */
#define UPDATE_TIMEOUT                      (20u)
/* The timeout for the default Cy_DFU__TransportWrite() call in milliseconds */
#define TRANSPORT_WRITE_TIMEOUT             (150u)   
/* The number of bytes per app in the metadata section */
#define METADATA_BYTES_PER_APP              (8u)

#define UINT16_SIZE                         (2u)
#define UINT32_SIZE                         (4u)

#define NIBBLE_POS                          (4u)
#define NIBBLE_MSK                          (0xFu)

#define SHA1_CHECKSUM_LENGTH                (20u)   /* The SHA1 length is 20 bytes */
/* A number of uint32_t elements in the SHA1 buffer */
#define SHA1_BUF_SIZE_UINT32                (SHA1_CHECKSUM_LENGTH/UINT32_SIZE)

#define RSA_CHECKSUM_LENGTH                 (256u)  /* The RSA public key modulo length is 2048 bits = 256 bytes */

#define CRC_CHECKSUM_LENGTH                 (4u)    /* The size of metadata flash row CRC in bytes */
#define CRC_POLYNOMIAL                      (0x1EDC6F41u) /* The CRC 32 polynomial */
#define CRC_LFSR_SEED                       (0xFFFFFFFFu)
#define CRC_DATA_REVERSE                    (1u)
#define CRC_DATA_XOR                        (0u)
#define CRC_REM_REVERSE                     (1u)
#define CRC_REM_XOR                         (0xFFFFFFFFu)
#define CRC_TABLE_SIZE                      (16u)           /* A number of uint32_t elements in the CRC32 table */
#define CRC_INIT                            (0xFFFFFFFFu)

#define CRC_CCITT_INIT                      (0xFFFFu)
#define CRC_CCITT_POLYNOMIAL                (0x8408u)

#define STATUS_BYTE_MSK                     (0xFFu)

/* The size in bytes of the DFU command parameters */
#define PARAMS_SIZE                         (8u) 
/* The length in bytes of data in the "Set App Metadata" DFU command */
#define DATA_LENGTH                         (9u) 
/* Possible sizes of the data field in the DFU packets */
#define DATA_PACKET_SIZE_4BYTES             (4u)
#define DATA_PACKET_SIZE_6BYTES             (6u)
#define DATA_PACKET_SIZE_8BYTES             (8u)
#define DATA_PACKET_SIZE_16BYTES            (16u)

#define PACKET_DATA_NO_OFFSET               (0u) 
#define PROGRAM_DATA_CRC_OFFSET             (4u) /* The offset in bytes to the CRC field in the Program Data command */
#define VERIFY_DATA_CRC_OFFSET              (4u) /* The offset in bytes to the CRC field in the Verify Data command */

/* The size in bytes of the data field in the Verify Application command */
#define VERIFY_APP_DATA_SIZE                (1u) 
/* The offset in bytes to the Application Length in the application metadata */
#define METADATA_APP_LENGTH_OFFSET          (4u) 
/* The offset in bytes to the application length in the data field of the Set Application Metadata command packet*/
#define SET_APP_METADATA_OFFSET             (1u) 
/* The offset to the application start address in the data field of the Set Application Metadata command packet */
#define SET_APP_METADATA_LENGTH_OFFSET      (5u) 
/* The offset to  the "to" part of the data field in the Get Metadata packet */
#define GET_METADATA_TO_OFFSET              (2u) 
  
/* The size in bytes of the App Size field in Cypress Simplified User Application Object */ 
#define SIMPLIFIED_APP_APPSIZE_SIZE         (4u)        

/* The offset in bytes to the VT offset in the Cypress Standard User Application Object */
#define CYPRESS_APP_VTOFFSET_OFFSET_BYTES   (0x10u) 
/* The offset in uint32 to the VT offset in the Cypress Standard User Application Object */
#define CYPRESS_APP_VTOFFSET_OFFSET_UINT32  (CYPRESS_APP_VTOFFSET_OFFSET_BYTES/UINT32_SIZE)
#define TOC_EMPTY                           (0UL) /* Both TOC2 and RTOC2 are empty */
#define TOC_INVALID                         (1UL) /* Either TOC2 or RTOC2 is invalid */
#define PUBLIC_KEY_IDX                      (9UL) /* The TOC item at index 9 is a public Key object  */
#define PUBLIC_KEY_OFFSET                   (8UL) /* The Public Key offset in the Public key Object */

/* The address of the the verify application function entry in the Flash Boot shared functions table */
#define VERIFY_APP_TABLE_ADDR               (0x16002040UL) 
/* The address of the the verify key function entry in the Flash Boot shared functions table */
#define IS_VALID_KEY_TABLE_ADDR             (0x16002044UL)
/* The address of the the verify TOC function entry in the Flash Boot shared functions table */
#define VALIDATE_TOC_TABLE_ADDR             (0x1600204CUL)

/* For the DFU packet */
#define PACKET_SOP_VALUE                    (0x01u)
#define PACKET_EOP_VALUE                    (0x17u)
#define PACKET_SOP_IDX                      (0x00u)
#define PACKET_CMD_IDX                      (0x01u)
#define PACKET_SIZE_IDX                     (0x02u)
#define PACKET_DATA_IDX                     (0x04u)
#define PACKET_CHECKSUM_LENGTH              (2u)    /* The length in bytes of a packet checksum field */


/* The Flash Boot verification functions*/
#if(CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP)
typedef bool (*Cy_FB_VerifyApp_t)(uint32_t address, uint32_t length, uint32_t signature, uint32_t publicKeyAddr);
typedef bool (*Cy_FB_IsValidKey_t)(uint32_t tocAddr, uint32_t publicKeyAddr);
typedef uint32_t (*Cy_FB_ValidateToc_t)(uint32_t tocAddress);
#endif /* (CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP) */

/** \endcond */


/* The static functions declaration */
static uint32_t ElfSymbolToAddr(void volatile const *symbol);
static void SwitchToApp(uint32_t stackPointer, uint32_t address);
#if ((CY_DFU_OPT_CRYPTO_HW != 0) && (CY_DFU_APP_FORMAT == CY_DFU_BASIC_APP))
    static bool ComputeSha1(uint32_t address, uint32_t length, uint8_t *result);
#endif /*((CY_DFU_OPT_CRYPTO_HW != 0) && (CY_DFU_APP_FORMAT == CY_DFU_BASIC_APP))*/

static uint16_t GetU16(uint8_t const array[]);
static uint32_t GetU32(uint8_t const array[]);
static void     PutU16(uint8_t array[], uint32_t offset, uint32_t value);

/* Because PutU32() is used only when updating the metadata */
#if CY_DFU_METADATA_WRITABLE != 0
    static void PutU32(uint8_t array[], uint32_t offset, uint32_t value);
#endif
static uint32_t PacketChecksumIndex(uint32_t size);
static uint32_t PacketEopIndex(uint32_t size);
static uint32_t GetPacketCommand(const uint8_t packet[]);
static uint32_t GetPacketDSize(const uint8_t packet[]);
static uint8_t* GetPacketData(uint8_t packet[], uint32_t offset);
static uint32_t GetPacketChecksum(const uint8_t packet[], uint32_t packetSize);
static uint32_t ValidatePacketFooter(const uint8_t packet[], uint32_t packetSize);
static void SetPacketHeader(uint8_t packet[]);
static void SetPacketCmd(uint8_t packet[], uint32_t cmd);
static void SetPacketDSize(uint8_t packet[], uint32_t size);
static void SetPacketChecksum(uint8_t packet[], uint32_t size, uint32_t checksum);
static void SetPacketFooter(uint8_t packet[], uint32_t size);

static uint32_t PacketChecksum(const uint8_t buffer[], uint32_t size);
static cy_en_dfu_status_t VerifyPacket(uint32_t numberRead, const uint8_t packet[]);
static cy_en_dfu_status_t ReadVerifyPacket(uint8_t packet[], bool *noResponse, uint32_t timeout);
static cy_en_dfu_status_t WritePacket(cy_en_dfu_status_t status, uint8_t *packet, uint32_t rspSize);
static void EnterResponse(uint8_t *packet, uint32_t *rspSize, uint32_t *state);

static cy_en_dfu_status_t CopyToDataBuffer(uint8_t dataBuffer[], uint32_t *dataOffset, uint8_t const packet[], 
                                                uint32_t packetSize);

static cy_en_dfu_status_t CommandEnter(uint8_t *packet, uint32_t *rspSize, uint32_t *state, 
                                            cy_stc_dfu_params_t *params);
static cy_en_dfu_status_t CommandProgramData(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);

#if CY_DFU_OPT_ERASE_DATA != 0
static cy_en_dfu_status_t CommandEraseData(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);
#endif
    
#if CY_DFU_OPT_VERIFY_DATA != 0
static cy_en_dfu_status_t CommandVerifyData(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);
#endif /* CY_DFU_OPT_VERIFY_DATA != 0*/

#if CY_DFU_OPT_SEND_DATA != 0
static cy_en_dfu_status_t CommandSendData(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);
#endif /* CY_DFU_OPT_SEND_DATA != 0 */

#if CY_DFU_OPT_VERIFY_APP != 0
static cy_en_dfu_status_t CommandVerifyApp(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);
#endif /* CY_DFU_OPT_VERIFY_APP != 0 */

static cy_en_dfu_status_t CommandSetAppMetadata(uint8_t *packet, uint32_t *rspSize, 
                                                     cy_stc_dfu_params_t *params);

#if CY_DFU_OPT_GET_METADATA != 0
static cy_en_dfu_status_t CommandGetMetadata(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params);
#endif /* CY_DFU_OPT_GET_METADATA != 0 */

#if CY_DFU_OPT_SET_EIVECTOR != 0
    static cy_en_dfu_status_t CommandSetEIVector(uint8_t *packet, uint32_t *rspSize,
                                                      cy_stc_dfu_params_t *params);
#endif /* CY_DFU_OPT_SET_EIVECTOR != 0 */

static cy_en_dfu_status_t CommandUnsupported(uint8_t *packet, uint32_t *rspSize, 
                                                              cy_stc_dfu_params_t *params );
static cy_en_dfu_status_t ContinueHelper(uint32_t command, uint8_t *packet, uint32_t *rspSize,
                                              cy_stc_dfu_params_t *params, bool *noResponse);
#if(CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP)
    #if(CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST)
        static bool VerifySecureAppShort(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr);
    #else
        static bool VerifySecureAppFull(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr);
    #endif /* CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST */
    static bool VerifySecureApp(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr);
#endif/*(CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP)*/


/*******************************************************************************
* Function Name: Cy_DFU_Complete
****************************************************************************//**
*
* This function starts the application download and install operations.
* It calls Cy_DFU_Init() and then continually calls Cy_DFU_Continue().
* It does not return until the DFU operation is complete or an error
* is detected.
* This function provides an example of how to use the DFU SDK to quickly
* design a DFU system. \note This function blocks, suspending all other
* tasks until the DFU operation is complete. To execute other tasks
* while updating, design your code to call Cy_DFU_Init() and
* Cy_DFU_Continue() directly.
* Only one updating operation can be done at a time - the user's code must
* ensure this. 
* \note The timeout for a communication interface is 20 ms - enough for
* UART (at 115200 bps), I2C, SPI, USB-HID, and BLE OTA in most cases.
* \note The function enables global interrupts.
*
* \param state      The pointer to a updating state variable.
*                   See \ref group_dfu_macro_state.
* \param timeout    The timeout in milliseconds for updating.
*
* \return See \ref cy_en_dfu_status_t.
*
*******************************************************************************/
cy_en_dfu_status_t  Cy_DFU_Complete(uint32_t *state, uint32_t timeout)
{   
    /* The DFU parameters, used to configure a DFU */
    cy_stc_dfu_params_t params;
    
    /* The status code for the DFU SDK API */
    cy_en_dfu_status_t status;
    
    /*
    * Used to count seconds, to convert counts to seconds use
    * counterTimeoutSeconds(SECONDS, DFU_DO_TIMEOUT)
    */
    uint32_t count = 0u;
    bool finished = false; /* To exit do {} while () loop */
    
    /* The buffer to save DFU commands to */
    CY_ALIGN(4) uint8_t buffer[CY_DFU_SIZEOF_DATA_BUFFER];
    /* The buffer for packets sent and received with the Transport API */
    CY_ALIGN(4) uint8_t packet[CY_DFU_SIZEOF_CMD_BUFFER ];
    
    /* Enable the global interrupts */
    __enable_irq();
    
    params.dataBuffer =   &buffer[0];
    params.packetBuffer = &packet[0];

#if CY_DFU_OPT_SET_EIVECTOR != 0
    uint8_t elVectorBuffer[DATA_PACKET_SIZE_16BYTES];
    params.encryptionVector = &elVectorBuffer[0];
#endif /* CY_DFU_OPT_SET_EIVECTOR != 0 */
    
    status = Cy_DFU_Init(state, &params);


    if (status != CY_DFU_SUCCESS)
    {
        Cy_SysLib_Halt(0x00u);
    }
    
    /* Run the DFU */
    params.timeout = UPDATE_TIMEOUT;
    Cy_DFU_TransportStart();

    do
    {
        
        status = Cy_DFU_Continue(state, &params);
        if (*state == CY_DFU_STATE_FINISHED)
        {
            /* The DFU of the application image is finished, the image is valid */
            finished = true;
        }
        else if (*state == CY_DFU_STATE_FAILED)
        {
            /* The DFU has failed */
            finished = true;
        }
        else if (*state == CY_DFU_STATE_UPDATING)
        {
            count = 0u;
        }
        else
        {
            /* Empty */
        }
        ++count;

        /* No DFU command received during the timeout period */
        if ( ( (count * UPDATE_TIMEOUT) >= timeout)
          && (*state == CY_DFU_STATE_NONE   )  )
        {
            finished = true;
        }
    }
    while (!finished);
        
    Cy_DFU_TransportStop();
    return (status);
}


/*******************************************************************************
* Function Name: Cy_DFU_Init
****************************************************************************//**
*
* This function starts the application download and install operations.
* Make subsequent calls to Cy_DFU_Continue() to continue the
* process. \n
* Returns immediately, reporting success or failure. \n
* Only one updating operation can be done at a time - the user's code must
* ensure this.
*
* \param state      The pointer to a updating state variable
*                   See \ref group_dfu_macro_state
* \param params     The pointer to a DFU parameters structure
*                   See \ref cy_stc_dfu_params_t
*
* \return See \ref cy_en_dfu_status_t.
* - \ref CY_DFU_SUCCESS if successful.
* - \ref CY_DFU_ERROR_UNKNOWN if a NULL pointer is passed.
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_Init(uint32_t *state, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;

    if ( (state == NULL) || (params == NULL) )
    {
        status = CY_DFU_ERROR_UNKNOWN;
    }

    if (status == CY_DFU_SUCCESS)
    {
        *state = CY_DFU_STATE_NONE;
        params->dataOffset = 0u;
    }
    return (status);
}

/*******************************************************************************
* Function Name: Cy_DFU_ExecuteApp
****************************************************************************//**
*
* This function transfers control from the current application to another 
* application.
* \note It is assumed appId is a valid application number.
*
* \param appId  An application number of the application to switch to.
*
*******************************************************************************/
void Cy_DFU_ExecuteApp(uint32_t appId)
{
    CY_ASSERT(appId < CY_DFU_MAX_APPS);
    cy_dfu_appId = (uint8_t)appId;
    CySoftwareReset();
}

/* Moves the argument address (R1) into the PC, moving execution to the address */
#if defined (__ARMCC_VERSION)
    static __asm void SwitchToApp(uint32_t stackPointer, uint32_t address)
    {
        MSR MSP, R0
        BX  R1
        ALIGN
    }
#elif defined(__GNUC__) 
    static void SwitchToApp(uint32_t stackPointer, uint32_t address )
    {
        __asm volatile("    MSR msp, %[sp]\n"
                       "    BX %[address] \n" : : [sp]"r"(stackPointer), [address]"r"(address) : "sp", "memory");
    }
#elif defined(__ICCARM__)
    #pragma optimize=no_inline
    static void SwitchToApp(uint32_t stackPointer, uint32_t address )
    {
        __asm volatile("MSR msp, R0\n"
                       "BX  R1     \n" );
    }
#endif  /* (__ARMCC_VERSION) */


/*******************************************************************************
* Function Name: Cy_DFU_SwitchToApp
****************************************************************************//**
*
* This function switches to the application through the jump instruction.
*
* Before calling this function, ensure all the peripherals and bus masters are
* in a known state.
*
* \note It is assumed appId is a valid application number.
*
* \param appId      An application number of the application to switch to.
*
* \return If failed, returns the status code. See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_SwitchToApp(uint32_t appId)
{
    uint32_t startAddress;
    cy_en_dfu_status_t status;
    
    CY_ASSERT(appId < CY_DFU_MAX_APPS);
    
    status = Cy_DFU_GetAppMetadata(appId, &startAddress, NULL);
    if (status == CY_DFU_SUCCESS)
    {
    #if(CY_DFU_APP_FORMAT == CY_DFU_SIMPLIFIED_APP)
        uint32_t offsetVt = ((uint32_t *)(startAddress + SIMPLIFIED_APP_APPSIZE_SIZE))[0];
        startAddress += SIMPLIFIED_APP_APPSIZE_SIZE + offsetVt; 
    #elif(CY_DFU_APP_FORMAT == CY_DFU_CYPRESS_APP)
        uint32_t offsetVt = ((uint32_t *)startAddress)[CYPRESS_APP_VTOFFSET_OFFSET_UINT32];
        startAddress += CYPRESS_APP_VTOFFSET_OFFSET_BYTES + offsetVt; 
    #else
        /* No format: a basic application*/
    #endif
        uint32_t stackPointer = ((uint32_t *)startAddress)[0]; /* The Stack Pointer of the app to switch to */
        uint32_t resetHandler = ((uint32_t *)startAddress)[1]; /* Reset_Handler() address */
        SwitchToApp(stackPointer, resetHandler);
    }
    return (status);
}


/*******************************************************************************
* Function Name: ElfSymbolToAddr
****************************************************************************//**
* 
* This function is used to convert an ELF file symbol address to uint32_t. \n
* This is safer than casting a symbol address to an integer because the 
* function does not produce a MISRA warning at the call side.
* Also, a function call is more readable and easier to search with the text 
* editor.
*
* \param symbol The address of the ELF file symbol to get the uint32_t value for.
*
* \return The address of the ELF file symbol as uint32_t.
*
*******************************************************************************/
static uint32_t ElfSymbolToAddr(void volatile const *symbol)
{
    return (uint32_t) symbol;
}


/*******************************************************************************
* Function Name: Cy_DFU_GetAppMetadata
****************************************************************************//**
*
* Reads application metadata to \c verifyAddress and \c verifySize.
* The metadata is supposed to be located in internal Flash memory.
*
* This is a weak function and the user may override it in the user's code by 
* providing a function with the same name.
* This allows the user to place metadata in any NVM.
*
* \note It is assumed appId is a valid application number.
*
* \param appId          The application number.
* \param verifyAddress  The pointer to a variable where an application
*                       verified area start address is stored. 
* \param verifySize     The pointer to a variable where a size of verified  
*                       application area is stored.
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_GetAppMetadata(uint32_t appId, uint32_t *verifyAddress, uint32_t *verifySize)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS; 
    
    CY_ASSERT(appId < CY_DFU_MAX_APPS);
    
    uint32_t *ptr = (uint32_t*) ( ElfSymbolToAddr(&__cy_boot_metadata_addr) + (appId * METADATA_BYTES_PER_APP) );
    
    if (verifyAddress != NULL)
    {
        *verifyAddress = ptr[0];
    }
    if (verifySize != NULL)
    {
        *verifySize      = ptr[1];
    }

    return (status);
}


/* Use PDL Hardware Crypto API */
#if ((CY_DFU_OPT_CRYPTO_HW != 0) && (CY_DFU_APP_FORMAT == CY_DFU_BASIC_APP))
/*******************************************************************************
* Function Name: ComputeSha1
****************************************************************************//**
*
* This function computes SHA1 for the message.
*
* \note Ensure the Crypto block is properly initialized
* and \ref CY_DFU_OPT_CRYPTO_HW is set.
* 
* \param address    The pointer to a buffer containing data to compute
*                   the checksum for. \n
*                   It must be 4-byte aligned.
* \param length     The number of bytes in the buffer to compute SHA1 for.
* \param result     The pointer to a buffer to store the SHA1 output.
*                   It must be 4-byte aligned.
*
* \return
* - true  - If calculation is successful.
* - false - If calculation is unsuccessful.
* 
*******************************************************************************/
static bool ComputeSha1(uint32_t address, uint32_t length, uint8_t *result)
{

    cy_stc_crypto_context_sha_t cryptoShaContext;
    cy_en_crypto_status_t cryptoStatus;
    bool statusOk = true; 
    
    cryptoStatus = Cy_Crypto_Enable();
    if (cryptoStatus == CY_CRYPTO_SUCCESS)
    {
        cryptoStatus = Cy_Crypto_Sha_Run((uint32_t *)address, length, (uint32_t *)result, CY_CRYPTO_MODE_SHA1,
                                          &cryptoShaContext);
        if (cryptoStatus == CY_CRYPTO_SUCCESS)
        {
            /* Waiting for SHA1 calculation is finished. */
            cryptoStatus = Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);
        }
        (void) Cy_Crypto_Disable();
    }
    if (cryptoStatus != CY_CRYPTO_SUCCESS)
    {
        statusOk = false;
    }
    return (statusOk);
}
#endif /* ((CY_DFU_OPT_CRYPTO_HW != 0) && (CY_DFU_APP_FORMAT == CY_DFU_BASIC_APP)) */


#if(CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP)
#if(CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST)
/*******************************************************************************
* Function Name: VerifySecureAppShort
****************************************************************************//**
*
* This function reports whether or not the specified application is valid.
*
* \param verifyStartAddr The start address of the application to verify.
* \param verifyLength The length of the application to verify.
* \param signatureAddr The address of the application signature.
*
* \return
*       - true - If the application is valid.
*       - false - If the application is invalid
*
*******************************************************************************/
static bool VerifySecureAppShort(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr)
{
    uint32_t publicKeyAddr = ( (uint32_t)& SFLASH->PUBLIC_KEY[0] ) + PUBLIC_KEY_OFFSET;
    Cy_FB_VerifyApp_t Cy_FB_VerifyApp = (Cy_FB_VerifyApp_t) (*(uint32_t *) VERIFY_APP_TABLE_ADDR);
    return Cy_FB_VerifyApp(verifyStartAddr, verifyLength, signatureAddr, publicKeyAddr );
}


#else
/*******************************************************************************
* Function Name: VerifySecureAppFull
****************************************************************************//**
*
* This function reports whether or not the specified application, TOC, and key 
* are valid.
*
* \param verifyStartAddr The start address of the application to verify.
* \param verifyLength The length of the application to verify.
* \param signatureAddr The address of the application signature.
*
* \return
*       - true - If the application is valid.
*       - false - If the application is invalid.
*
*******************************************************************************/
static bool VerifySecureAppFull(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr)
{
    Cy_FB_ValidateToc_t Cy_FB_ValidateToc = (Cy_FB_ValidateToc_t) (*(uint32_t *) VALIDATE_TOC_TABLE_ADDR);
    Cy_FB_VerifyApp_t   Cy_FB_VerifyApp   = (Cy_FB_VerifyApp_t)   (*(uint32_t *) VERIFY_APP_TABLE_ADDR  );
    Cy_FB_IsValidKey_t  Cy_FB_IsValidKey  = (Cy_FB_IsValidKey_t)  (*(uint32_t *) IS_VALID_KEY_TABLE_ADDR);
    bool status = true;
    
    uint32_t tocAddr = Cy_FB_ValidateToc((uint32_t)& SFLASH->TOC2_OBJECT_SIZE);
    
    if ((tocAddr == TOC_EMPTY) || (tocAddr == TOC_INVALID))
    {
        status = false;
    }
    else
    {
        uint32_t publicKeyAddr = *(uint32_t *)(tocAddr + (sizeof(uint32_t) * PUBLIC_KEY_IDX))
                                + PUBLIC_KEY_OFFSET;
        status = Cy_FB_IsValidKey(tocAddr, publicKeyAddr);
        if (status)
        {
            status = Cy_FB_VerifyApp(verifyStartAddr, verifyLength, signatureAddr, publicKeyAddr );
        }
    }
    return (status);
}
#endif /* CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST */


/*******************************************************************************
* Function Name: VerifySecureApp
****************************************************************************//**
*
* This function reports whether or not the specified application is valid.
*
* \param verifyStartAddr The start address of the application to verify.
* \param verifyLength The length of the application to verify.
* \param signatureAddr The address of the application signature.
*
* \return
*       - true - If the application is valid.
*       - false - If the application is invalid.
*
*******************************************************************************/
static bool VerifySecureApp(uint32_t verifyStartAddr, uint32_t verifyLength, uint32_t signatureAddr)
{
#if(CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST)
    return VerifySecureAppShort(verifyStartAddr, verifyLength, signatureAddr);
#else
    return VerifySecureAppFull (verifyStartAddr, verifyLength, signatureAddr);
#endif /* CY_DFU_SEC_APP_VERIFY_TYPE == CY_DFU_VERIFY_FAST */
}
#endif/*(CY_DFU_APP_FORMAT != CY_DFU_BASIC_APP)*/


/*******************************************************************************
* Function Name: Cy_DFU_ValidateApp
****************************************************************************//**
*
* This function reports whether or not the specified application is valid. \n
* This is a weak function and the user may override it in the user's code by 
* providing a function with the same name.
*
* \note It is assumed appId is a valid application number.
*
* \param appId      The application number of the application to be validated.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
* 
* \return See \ref cy_en_dfu_status_t.
* - \ref CY_DFU_SUCCESS If the application is valid.
* - \ref CY_DFU_ERROR_VERIFY If the application is invalid.
* 
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_ValidateApp(uint32_t appId, cy_stc_dfu_params_t *params)
{
    
    uint32_t appVerifyStartAddress;
    uint32_t appVerifySize;
    
    (void)params;
    
    CY_ASSERT(appId < CY_DFU_MAX_APPS);
    
    cy_en_dfu_status_t status = Cy_DFU_GetAppMetadata(appId, &appVerifyStartAddress, &appVerifySize);
    

    if (status == CY_DFU_SUCCESS)
    {
    #if(CY_DFU_APP_FORMAT == CY_DFU_CYPRESS_APP)
        status = (VerifySecureApp(appVerifyStartAddress, appVerifySize, appVerifyStartAddress + appVerifySize))? 
                                  CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
    #elif (CY_DFU_APP_FORMAT == CY_DFU_SIMPLIFIED_APP)
        status = (VerifySecureApp(appVerifyStartAddress, appVerifySize, appVerifyStartAddress - RSA_CHECKSUM_LENGTH))?
                                  CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
    #else
        #if(CY_DFU_OPT_CRYPTO_HW != 0)
            uint32_t sha1buf[SHA1_BUF_SIZE_UINT32]; 
            uint32_t appFooterAddress = appVerifyStartAddress + appVerifySize;
            if (ComputeSha1(appVerifyStartAddress, appVerifySize, (uint8_t*)&sha1buf))
            { 
                status = (memcmp((const void *)sha1buf, (const void *)appFooterAddress, SHA1_CHECKSUM_LENGTH) == 0)? 
                                                                        CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
            }
            else
            {
                status = CY_DFU_ERROR_VERIFY;
            }
        #else

            uint32_t appCrc = Cy_DFU_DataChecksum((uint8_t *)appVerifyStartAddress, appVerifySize, params);
            uint32_t appFooterAddress = (appVerifyStartAddress + appVerifySize);
            status = (*(uint32_t*)appFooterAddress == appCrc) ? CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
        #endif/* (CY_DFU_OPT_CRYPTO_HW != 0) */
    #endif /* (CY_DFU_APP_FORMAT == CY_DFU_CYPRESS_APP) */
    }
    return (status);
}


/*******************************************************************************
* Function Name: Cy_DFU_GetRunningApp
****************************************************************************//**
*
* This function reports the application number of the currently running 
* application.
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
uint32_t Cy_DFU_GetRunningApp(void)
{
    return ElfSymbolToAddr(&__cy_app_id);
}


/*******************************************************************************
* Function Name: Cy_DFU_CopyApp
****************************************************************************//**
*
* This function copies an application from a temporary location in Flash to its 
* destination location in Flash. This function is typically called when updating 
* an application used as part of an update process, for example updating
* a BLE stack.
* \note This API is only for demonstration purpose, use it only when copying 
* from internal Flash to internal Flash. For other user cases, implement a 
* custom, more general function.
*
* \param destAddress  The start address of the application to copy to.
* \param srcAddress   The start address of the copy of the application to be 
*                     copied.
* \param length       The number of bytes to copy.
* \param rowSize      The size of a Flash row in bytes.
* \param params       The pointer to a DFU parameters structure.
*                     See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_CopyApp(uint32_t destAddress, uint32_t srcAddress, uint32_t length,
                                            uint32_t rowSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_UNKNOWN;
    uint32_t writeAddr  = destAddress;
    uint32_t readAddr   = srcAddress;
    uint32_t endAddress = destAddress + length;

    while (writeAddr < endAddress)
    {
        status = Cy_DFU_ReadData(readAddr, rowSize, CY_DFU_IOCTL_READ, params);
        if (status == CY_DFU_SUCCESS)
        {
            status = Cy_DFU_WriteData(writeAddr, rowSize, CY_DFU_IOCTL_WRITE, params);
        }
        if (status != CY_DFU_SUCCESS)
        {
            break;
        }
        writeAddr += rowSize;
        readAddr  += rowSize;
    }
    
    return (status);
}


/*******************************************************************************
* Function Name: Cy_DFU_OnResetApp0
****************************************************************************//**
*
* This function is used in an App0 firmware image in Reset_Handler() only.
* Checks if switching to the other application is scheduled with
* \ref Cy_DFU_ExecuteApp(). \n
* If the switch is scheduled, then it validates the application and transfers
* control to it.
*
*******************************************************************************/
void Cy_DFU_OnResetApp0(void)
{
    /* Set cy_dfu_appId to ZERO under a non-software reset. This means
     * that the DFU application is scheduled - the initial clean state.
     * The value of cy_dfu_appId is valid only under a software reset.
     */    
    if (Cy_SysLib_GetResetReason() != CY_SYSLIB_RESET_SOFT)
    {
        cy_dfu_appId = 0u;
    }
    else
    {
        if (cy_dfu_appId != 0u)
        {
            (void) Cy_DFU_SwitchToApp((uint32_t) cy_dfu_appId);
        }
    }
}


/*******************************************************************************
* Function Name: Cy_DFU_ReadData
****************************************************************************//**
*
* This function must be implemented in the user's code.
* 
* Reads \c buffer from Flash, SMIF, or any other external memory type with
* custom pre and post read commands.
* 
* \param address    The address from where to read data, must be aligned to
*                   a Flash row, SMIF page, etc.
* \param length     The length in bytes of data to read, must be multiple of
*                   a Flash row, SMIF page, etc.
* \param ctl        Additional features of the read function:
* - CY_DFU_IOCTL_READ    - Only read.
* - CY_DFU_IOCTL_COMPARE - Compare the data in the buffer with the data in
*                               memory.
* - CY_DFU_IOCTL_BHP     - Decrypt data before comparing the buffer with
*                               memory,
*   if the DFU Host provided encrypted data.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
* \return See \ref cy_en_dfu_status_t
* - CY_DFU_SUCCESS - If successful.
* - CY_DFU_ERROR_LENGTH if \c The length value is invalid.
* - CY_DFU_ERROR_ADDRESS if \c The address is invalid.
* 
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_ReadData (uint32_t address, uint32_t length, uint32_t ctl, 
                                                     cy_stc_dfu_params_t *params)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
    
    (void) address;
    (void) length;
    (void) ctl;
    (void) params;
    
    return (CY_DFU_SUCCESS);
}


/*******************************************************************************
* Function Name: Cy_DFU_WriteData
****************************************************************************//**
*
* This function must be implemented in the user's code.
* 
* Writes the \c buffer to Flash, SMIF, or any other external memory type with
* custom pre and post write commands.
* 
* \param address    The address to write data to, must be aligned to a Flash 
*                   row, SMIF page, etc.
* \param length     The length in bytes of data to be written, must be multiple 
*                   of a Flash row, SMIF page, etc.
* \param ctl        Additional features of the write function: 
* - CY_DFU_IOCTL_WRITE - Only write.
* - CY_DFU_IOCTL_ERASE - Erase the sector, the sector size can be bigger
*   than the size of the page to write.
* - CY_DFU_IOCTL_BHP   - Decrypt data before writing to memory, if
*   the DFU Host provided encrypted data.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
* \return See \ref cy_en_dfu_status_t.
* - CY_DFU_SUCCESS - If successful.
* - CY_DFU_ERROR_LENGTH if \c The length value is invalid.
* - CY_DFU_ERROR_ADDRESS if \c The address is invalid.
* 
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_WriteData(uint32_t address, uint32_t length, uint32_t ctl, 
                                                     cy_stc_dfu_params_t *params)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
    
    (void) address;
    (void) length;
    (void) ctl;
    (void) params;
    
    return (CY_DFU_SUCCESS);
}


/*******************************************************************************
* Function Name: Cy_DFU_TransportRead
****************************************************************************//**
* This function must be implemented in the user's code.
* 
* This function receives a command packet from the DFU Host via the
* communication channel. The function waits for a timeout until all bytes are
* received.
*
* \param buffer The pointer to a buffer to store a received command.
* \param size   The number of bytes to read.
* \param count  The pointer to the variable that contains the number of received
*               bytes.
* \param timeout The time to wait before the function returns because of a 
*                timeout, in milliseconds.
*
* \return The status of the transmit operation:
* - CY_DFU_SUCCESS - If successful.
* - CY_DFU_ERROR_TIMEOUT - If no data is received.
* - See \ref cy_en_dfu_status_t.
*
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_TransportRead(uint8_t buffer[], uint32_t size, uint32_t *count, 
                                                         uint32_t timeout)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
    
    (void) buffer;
    (void) size;
    (void) count;
    (void) timeout;
    
    return (CY_DFU_SUCCESS);
}


/*******************************************************************************
* Function Name: Cy_DFU_TransportWrite
****************************************************************************//**
* This function must be implemented in the user's code.
*
* This function transmits a response packet to the DFU host via the
* communication channel. The function waits for a timeout until all bytes are
* sent.
*
* \param buffer The pointer response packet buffer.
* \param size   The number of bytes to transmit.
* \param count  The pointer to the actual number of transmitted bytes.
* \param timeout The time to wait before the function returns because of a 
*        timeout, in milliseconds
*
* \return See \ref cy_en_dfu_status_t.
* The status of the transmit operation:
* - CY_DFU_SUCCESS - If successful.
* - CY_DFU_ERROR_TIMEOUT - If no data is transmitted.
* 
*******************************************************************************/
__WEAK cy_en_dfu_status_t Cy_DFU_TransportWrite(uint8_t buffer[], uint32_t size, uint32_t *count, 
                                                          uint32_t timeout)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
    
    (void) buffer;
    (void) size;
    (void) count;
    (void) timeout;
    
    return (CY_DFU_SUCCESS);
}


/*******************************************************************************
* Function Name: Cy_DFU_TransportReset
****************************************************************************//**
*
* This function must be implemented in the user's code. \n
* Resets the communication interface with clearing buffers, offsets, length, 
* etc.
*
*******************************************************************************/
__WEAK void Cy_DFU_TransportReset(void)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
}


/*******************************************************************************
* Function Name: Cy_DFU_TransportStart
****************************************************************************//**
*
* This function must be implemented in the user's code. \n
* Starts the communication interface through which updating will be working.
*
*******************************************************************************/
__WEAK void Cy_DFU_TransportStart(void)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
}


/*******************************************************************************
* Function Name: Cy_DFU_TransportStop
****************************************************************************//**
*
* This function must be implemented in the user's code. \n
* Stops the communication interface through which updating will be working.
*
*******************************************************************************/
__WEAK void Cy_DFU_TransportStop(void)
{
    /*
    * This function does nothing, weak implementation.
    * The purpose of this code is to disable compiler warnings for Non-optimized
    * builds which do not remove unused functions and require them for the 
    * completeness of the linking step.
    */
}


/*******************************************************************************
*        Cy_DFU_Continue related code, till the EOF
*******************************************************************************/


/*******************************************************************************
* Function Name: GetU16
****************************************************************************//**
*
* This function is used to read a uint16_t value from a byte array. \n
* MISRA is informed with #pragma that the function has no side effects.
*
* \param array      The pointer to the byte array to get data from.
*
* \return The uint16_t value from the \c array parameter.
* 
*******************************************************************************/
static uint16_t GetU16(uint8_t const array[])
{
    uint16_t temp;
    (void) memcpy( &temp, &array[0], UINT16_SIZE);
    return (temp);
}


/*******************************************************************************
* Function Name: GetU32
****************************************************************************//**
*
* This function is used to read an uint32_t value from a byte array. \n
* MISRA is informed with #pragma that the function has no side effects.
*
* \param array      The pointer to the byte array to get data from.
*
* \return The uint32_t value from the \c array parameter.
* 
*******************************************************************************/
static uint32_t GetU32(uint8_t const array[])
{
    uint32_t temp;
    (void) memcpy( &temp, &array[0], UINT32_SIZE);
    return (temp);
}


/*******************************************************************************
* Function Name: PutU16
****************************************************************************//**
*
* This function is used to write an uint16_t value to a byte array.
*
* \param array      The pointer to the byte array.
* \param offset     The offset within the byte array to write data to.
* \param value      The value to be stored to the byte array.
*
*******************************************************************************/
static void PutU16(uint8_t array[], uint32_t offset, uint32_t value)
{
    (void) memcpy( &array[offset], &value, UINT16_SIZE);
}


#if CY_DFU_METADATA_WRITABLE != 0
    /*******************************************************************************
    * Function Name: PutU32
    ****************************************************************************//**
    *
    * This function is used to write an uint32_t value to a byte array.
    *
    * \param array      The pointer to the byte array.
    * \param offset     The offset within the byte array to write data to.
    * \param value      The value to be stored to the byte array.
    *
    *******************************************************************************/
    static void PutU32(uint8_t array[], uint32_t offset, uint32_t value)
    {
        (void) memcpy( &array[offset], &value, UINT32_SIZE);
    }
#endif /* CY_DFU_METADATA_WRITABLE != 0 */


/*******************************************************************************
* Function Name: PacketChecksumIndex
****************************************************************************//**
*
* This function returns an index to the checksum field in a received packet 
*
* \param size The DFU packet size value.
*
* \return The index to the checksum.
* 
*******************************************************************************/
static uint32_t PacketChecksumIndex(uint32_t size)
{
    return (PACKET_DATA_IDX + size);
}


/*******************************************************************************
* Function Name: PacketEopIndex
****************************************************************************//**
*
* This function returns an index to the end of the packet field in a received 
* packet.
*
* \param size  The DFU packet size value.
*
* \return Returns an index to the end of the packet.
* 
*******************************************************************************/
static uint32_t PacketEopIndex(uint32_t size)
{ 
    return (PACKET_DATA_IDX + size + PACKET_CHECKSUM_LENGTH);
}


/*******************************************************************************
* Function Name: GetPacketCommand
****************************************************************************//**
*
* This function returns a DFU packet command value for a given \c packet.
*
* \param packet  The pointer to the byte array containing DFU packet
*                data.
*
* \return A DFU packet command value.
* 
*******************************************************************************/
static uint32_t GetPacketCommand(const uint8_t packet[])
{
    return ( (uint32_t) packet[PACKET_CMD_IDX] );
}


/*******************************************************************************
* Function Name: GetPacketDSize
****************************************************************************//**
*
* This function returns a data size in the DFU packet. \n
* MISRA requires this function have no side effects.
*
* \param packet  The pointer to the byte array containing DFU packet
*                data.
*
* \return A size of the data in bytes in the packet.
* 
*******************************************************************************/
static uint32_t GetPacketDSize(const uint8_t packet[])
{
    return ( (uint32_t) GetU16( &packet[PACKET_SIZE_IDX] ) );
}


/*******************************************************************************
* Function Name: GetPacketData
****************************************************************************//**
*
* This function is used to get packet data.
*
* \param packet  The pointer to the byte array containing DFU packet
*                data.
* \param offset  The offset within the data bytes in the packet.
*                E.g. 0 means the first byte of the data.
* 
* \return A pointer to the data bytes with the offset in the packet.
*
*******************************************************************************/
static uint8_t* GetPacketData(uint8_t packet[], uint32_t offset)
{
    return ( &packet[PACKET_DATA_IDX + offset] );
}


/*******************************************************************************
* Function Name: GetPacketChecksum
****************************************************************************//**
*
* This function is used to get a DFU packet checksum.
*
* \param packet      The pointer to a byte array containing the
*                    DFU packet data.
* \param packetSize  The offset within the data bytes in the packet. \n
*                    E.g. 0 means the first byte of the data.
* 
* \return The DFU packet checksum.
*
*******************************************************************************/
static uint32_t GetPacketChecksum(const uint8_t packet[], uint32_t packetSize)
{
    return ( (uint32_t) GetU16( &packet[ PacketChecksumIndex(packetSize) ] ) );
}


/*******************************************************************************
* Function Name: ValidatePacketFooter
****************************************************************************//**
*
* This function is used to validate a DFU packet footer.
*
* \param packet      The pointer to a byte array containing the 
*                    DFU packet data.
* \param packetSize  The offset within the data bytes in the packet. \n
*                    E.g. 0 means the first byte of the data.
* \return
* - 0, if the DFU packet footer is invalid.
* - 1, if the DFU packet footer is valid.
*
*******************************************************************************/
static uint32_t ValidatePacketFooter(const uint8_t packet[], uint32_t packetSize)
{
    return ( (packet[PacketEopIndex(packetSize)] == PACKET_EOP_VALUE)? 1ul : 0ul );
}


/*******************************************************************************
* Function Name: SetPacketHeader
****************************************************************************//**
*
* This function is used to set a DFU packet header value.
*
* \param packet     The pointer to a byte array containing the DFU packet
*                   data.
*
*******************************************************************************/
static void SetPacketHeader(uint8_t packet[])
{
    packet[PACKET_SOP_IDX] = PACKET_SOP_VALUE;
}


/*******************************************************************************
* Function Name: SetPacketCmd
****************************************************************************//**
*
* This function is used to set a DFU packet command value.
*
* \param packet     The pointer to a byte array containing the
*                   DFU packet data.
* \param cmd        The command value to be set in the DFU packet.
*
*******************************************************************************/
static void SetPacketCmd(uint8_t packet[], uint32_t cmd)
{
    packet[PACKET_CMD_IDX] = (uint8_t)cmd;
}


/*******************************************************************************
* Function Name: SetPacketDSize
****************************************************************************//**
*
* This function is used to set a DFU packet size value.
*
* \param packet     The pointer to a byte array containing the 
*                   DFU packet data.
* \param size       The value for the DFU packet size.
*
*******************************************************************************/
static void SetPacketDSize(uint8_t packet[], uint32_t size)
{
    PutU16(packet, PACKET_SIZE_IDX, size);
}


/*******************************************************************************
* Function Name: SetPacketChecksum
****************************************************************************//**
*
* This function is used to set a DFU packet checksum value.
*
* \param packet     The pointer to a byte array containing the 
*                   DFU packet data.
* \param size       The DFU packet size value.
* \param checksum   The value for a DFU packet checksum.
*
*******************************************************************************/
static void SetPacketChecksum(uint8_t packet[], uint32_t size, uint32_t checksum)
{
    PutU16(packet, PacketChecksumIndex(size), checksum);
}


/*******************************************************************************
* Function Name: SetPacketFooter
****************************************************************************//**
*
* This function is used to set a DFU packet footer value.
*
* \param packet     The pointer to a byte array containing the
*                   DFU packet data.
* \param size       The DFU packet size value.
*
*******************************************************************************/
static void SetPacketFooter(uint8_t packet[], uint32_t size)
{
    packet[PacketEopIndex(size)] = PACKET_EOP_VALUE;
}


/*******************************************************************************
* Function Name: Cy_DFU_ValidateMetadata
****************************************************************************//**
*
* The function checks if the DFU metadata is valid.
*
* \return See \ref cy_en_dfu_status_t
* - \ref CY_DFU_SUCCESS - If metadata is valid.
* - \ref CY_DFU_ERROR_VERIFY - If metadata is not valid.
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_ValidateMetadata(uint32_t metadataAddress, cy_stc_dfu_params_t *params)
{
    const uint32_t metadataLength = ElfSymbolToAddr(&__cy_boot_metadata_length);
    
    uint32_t crc = Cy_DFU_DataChecksum( (uint8_t *)metadataAddress, metadataLength - CRC_CHECKSUM_LENGTH, params);
    uint32_t crcMeta = *(uint32_t *)(metadataAddress + (metadataLength - CRC_CHECKSUM_LENGTH) );
    cy_en_dfu_status_t status = (crc == crcMeta) ? CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
    return (status);
}


/*******************************************************************************
* Function Name: PacketChecksum
****************************************************************************//**
*
* This function computes a 16-bit checksum for the provided number of bytes
* contained
* in the provided buffer. \n
* This function is used to calculate the checksum of DFU packets.
* 
* MISRA requires this function have no side effects.
*
* \param buffer     The buffer containing the data to compute the checksum for.
* \param size       The number of bytes in the buffer to compute the checksum
* for.
*
* \return A 16-bit checksum for the provided data
* 
*******************************************************************************/
static uint32_t PacketChecksum(const uint8_t buffer[], uint32_t size)
{
#if (CY_DFU_OPT_PACKET_CRC != 0u)
    uint16_t crc = CRC_CCITT_INIT;
    uint16_t tmp;
    uint32_t i;
    uint16_t tmpIndex;
    
    size += PACKET_DATA_IDX; /* 4 bytes before data in Cypress DFU packet */
    
    tmpIndex = (uint16_t)size;

    if(0u == size)
    {
        crc = ~crc;
    }
    else
    {
        do
        {
            tmp = buffer[tmpIndex - size];

            for (i = 0u; i < 8u; i++)
            {
                if (0u != ((crc & 0x0001u) ^ (tmp & 0x0001u)))
                {
                    crc = (crc >> 1u) ^ CRC_CCITT_POLYNOMIAL;
                }
                else
                {
                    crc >>= 1u;
                }

                tmp >>= 1u;
            }

            size--;
        }
        while(0u != size);

        crc = ~crc;
        tmp = crc;
        crc = ((uint16_t)(crc << 8u) | (tmp >> 8u) ) & 0xFFFFu;
    }

    return ((uint32_t)crc);
#else
    uint16_t sum = 0u;
    size += PACKET_DATA_IDX; /* 4 bytes before data in Cypress DFU packet */
    
    while (size > 0u)
    {
        size--;
        sum += buffer[size];
    }

    return ( (uint32_t) (  (1u + ~(uint32_t)sum) & 0xFFFFu  ) );
#endif /* CY_DFU_OPT_PACKET_CRC != 0u */
}


/*******************************************************************************
* Function Name: Cy_DFU_DataChecksum
****************************************************************************//**
*
* This function computes a CRC-32C for the provided number of bytes contained
* in the provided buffer. \n
* This function is used to validate the Program Data and Verify Data DFU
* commands and a metadata row.
* \note Ensure the Crypto block is properly initialized
* if \ref CY_DFU_OPT_CRYPTO_HW is set.
* 
* \param address    The pointer to a buffer containing the data to compute
*                   the checksum for.
* \param length     The number of bytes in the buffer to compute the checksum
*                   for.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return CRC-32C for the provided data.
* 
*******************************************************************************/
uint32_t Cy_DFU_DataChecksum(const uint8_t *address, uint32_t length, cy_stc_dfu_params_t *params)
{
#if CY_DFU_OPT_CRYPTO_HW != 0 /* Use PDL Hardware Crypto API */
    /* Note that the length will be < 64KB due to the hardware limitation in the Crypto Block. */
    /* If the block size is bigger, use software implementation instead. */

    
    
    uint32_t crcOut = 0u;
    /* Note: param is unused by the current implementation */
    /* but it may be used in the future, if the Crypto API changes */
    (void)params;
    
    cy_stc_crypto_context_crc_t cryptoCrcContext;
    cy_en_crypto_status_t cryptoStatus;
    
    cryptoStatus = Cy_Crypto_Enable();
    if (cryptoStatus == CY_CRYPTO_SUCCESS)
    {
        cryptoStatus = Cy_Crypto_Crc_Init( CRC_POLYNOMIAL,     CRC_DATA_REVERSE,
                                           CRC_DATA_XOR  ,     CRC_REM_REVERSE ,
                                           CRC_REM_XOR,        &cryptoCrcContext );
        if (cryptoStatus == CY_CRYPTO_SUCCESS)
        {
            cryptoStatus = Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);
        }
        if (cryptoStatus == CY_CRYPTO_SUCCESS)
        {
            cryptoStatus = Cy_Crypto_Crc_Run (
                /* dataPtr */ (void *)address,          /* length        */ (uint16_t) length,
                /* crcPtr  */ &crcOut,                  /* lfsrInitState */ CRC_LFSR_SEED,
                /* cfContext     */ &cryptoCrcContext  );
        }
        if (cryptoStatus == CY_CRYPTO_SUCCESS)
        {
            cryptoStatus = Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);
        }

        (void) Cy_Crypto_Disable();
    }
    if (cryptoStatus != CY_CRYPTO_SUCCESS)
    {
        Cy_SysLib_Halt(0x00u);
    }
    
    return (crcOut);
#else /* Use software implementation */
    /* Contains generated values to calculate CRC-32C by 4 bits per iteration*/
    const static uint32_t crcTable[CRC_TABLE_SIZE] = 
    {
        0x00000000u, 0x105ec76fu, 0x20bd8edeu, 0x30e349b1u,
        0x417b1dbcu, 0x5125dad3u, 0x61c69362u, 0x7198540du,
        0x82f63b78u, 0x92a8fc17u, 0xa24bb5a6u, 0xb21572c9u,
        0xc38d26c4u, 0xd3d3e1abu, 0xe330a81au, 0xf36e6f75u,
    };
    
    uint32_t crc = CRC_INIT;
    if (length != 0u)
    {
        do
        {
            crc = crc ^ *address;
            crc = (crc >> NIBBLE_POS) ^ crcTable[crc & NIBBLE_MSK];
            crc = (crc >> NIBBLE_POS) ^ crcTable[crc & NIBBLE_MSK];
            --length;
            ++address;
        } while (length != 0u);
    }
    return (~crc);
#endif /* CY_DFU_OPT_CRYPTO_HW != 0 */
}


/*******************************************************************************
* Function Name: VerifyPacket
****************************************************************************//**
*
* This function is used inside DFU_ReadVerifyPacket() to verify if a
* received packet is correct.
*
* \param numberRead     The number of bytes read from the communication
*                       interface.
* \param packet         The pointer to the DFU packet buffer.
*
* \return  See \ref cy_en_dfu_status_t.
*
*******************************************************************************/
static cy_en_dfu_status_t VerifyPacket(uint32_t numberRead, const uint8_t packet[])
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;

    if ((numberRead < CY_DFU_PACKET_MIN_SIZE) || (packet[PACKET_SOP_IDX] != PACKET_SOP_VALUE))
    {
        status = CY_DFU_ERROR_DATA;
    }
    else
    {
        uint32_t packetSize = GetPacketDSize(packet);

        /*
         * If the whole packet length exceeds the number of bytes that have
         * been read by the communication component or the size of
         * the buffer that is reserved for the packet, then give an error.
         */
        if (   ((packetSize + CY_DFU_PACKET_MIN_SIZE) > numberRead)
            || ((packetSize + CY_DFU_PACKET_MIN_SIZE) > CY_DFU_SIZEOF_CMD_BUFFER)  )
        {
            status = CY_DFU_ERROR_LENGTH;
        }
        else /* The packet length is OK */
        {
            if ( ValidatePacketFooter(packet, packetSize) == 0u )
            {
                status = CY_DFU_ERROR_DATA;
            }
            else
            {
                uint32_t pktChecksum = GetPacketChecksum(packet, packetSize);
                if (pktChecksum != PacketChecksum(packet, packetSize) )
                {
                    status = CY_DFU_ERROR_CHECKSUM;
                }
            }
        }
    }
    return (status);
}


/*******************************************************************************
* Function Name: ReadVerifyPacket
****************************************************************************//**
*
* This function is used inside Cy_DFU_Continue to read and verify
* a received DFU packet.
*
* \param packet       The pointer to the DFU packet buffer.
* \param noResponse   The pointer to a variable that states whether to send
*                      a response back to a DFU Host or not.
* \param timeout      The timeout in milliseconds to wait.
*
* \return See \ref cy_en_dfu_status_t
* - \ref CY_DFU_SUCCESS - If a packet is successfully received.
* - \ref CY_DFU_ERROR_TIMEOUT - If no packet is received during
*   the timeout period.
* 
*******************************************************************************/
static cy_en_dfu_status_t ReadVerifyPacket(uint8_t packet[], bool *noResponse, uint32_t timeout)
{
    cy_en_dfu_status_t status;
    uint32_t numberRead = 0U;
    
    status = Cy_DFU_TransportRead( packet, CY_DFU_SIZEOF_CMD_BUFFER, &numberRead, timeout );

    if (status == CY_DFU_ERROR_TIMEOUT)
    {
        *noResponse = true;
    }
    if (status == CY_DFU_SUCCESS)
    {
        status = VerifyPacket(numberRead, packet);
    }
    return (status);
}


/*******************************************************************************
* Function Name: WritePacket
****************************************************************************//**
*
* This function creates a DFU response packet and transmits it back to
* the DFU host application over the already established communications
* protocol.
*
* \param status     The error response code of the DFU response packet.
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The number of bytes contained within the \c packet
*                   to pass back to a DFU Host.
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t WritePacket(cy_en_dfu_status_t status, uint8_t *packet, uint32_t rspSize)
{
    uint32_t checksum;
    /*
    * The DFU Host expects only one byte of the status,
    * its value must be compatible with the PSoC 3/4/5LP Bootloader Component
    */
    uint32_t statusCode = (uint32_t)status & STATUS_BYTE_MSK;

    /* Build a packet */
    SetPacketHeader(packet);
    SetPacketCmd   (packet, statusCode);
    SetPacketDSize (packet, rspSize);

    checksum = PacketChecksum(packet, rspSize);
    SetPacketChecksum(packet, rspSize, checksum);
    SetPacketFooter  (packet, rspSize);

    return ( Cy_DFU_TransportWrite(packet, rspSize + CY_DFU_PACKET_MIN_SIZE, &rspSize, 
                                        TRANSPORT_WRITE_TIMEOUT));
}


/*******************************************************************************0
* Function Name: EnterResponse
****************************************************************************//**
*
* This function is used inside Cy_DFU_ContinueEntr() to add version and
* size fields.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param state      The pointer to a updating state variable.
*                   See \ref group_dfu_macro_state.
*
*******************************************************************************/
static void EnterResponse(uint8_t *packet, uint32_t *rspSize, uint32_t *state)
{
    static const cy_stc_dfu_enter_t dfuVersion =
    {
        CY_DFU_SILICON_ID,
        (uint8_t) CY_DFU_SILICON_REV,
        {
            (uint8_t) CY_DFU_SDK_VERSION_MINOR,
            (uint8_t) CY_DFU_SDK_VERSION_MAJOR,
            0x01u /* Used for BWC with the Bootloader component */
        }
    };
    *state = CY_DFU_STATE_UPDATING;
    *rspSize = sizeof(dfuVersion);
    (void) memcpy( GetPacketData(packet, PACKET_DATA_NO_OFFSET), &dfuVersion, *rspSize);
}


/*******************************************************************************
* Function Name: CommandEnter
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
*
* This function is called to handle Command Enter, see the DFU Commands
* section in the DFU SDK User's Guide.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param state      The pointer to a updating state variable.
*                   See \ref group_dfu_macro_state.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandEnter(uint8_t *packet, uint32_t *rspSize, uint32_t *state, 
                                            cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_LENGTH;
    volatile uint32_t productId = ElfSymbolToAddr(&__cy_product_id);
    uint32_t packetSize = GetPacketDSize(packet);
    *rspSize = CY_DFU_RSP_SIZE_0;
    
    if (packetSize == 0u)  /* 'Product ID' not demanded */
    {
        status = (productId == 0u)? CY_DFU_SUCCESS : CY_DFU_ERROR_LENGTH;
        if (status == CY_DFU_SUCCESS)
        {
            EnterResponse(packet, rspSize, state);
        }
    }
    else if ( (packetSize == DATA_PACKET_SIZE_4BYTES) || (packetSize == DATA_PACKET_SIZE_6BYTES) )
    {
        status = CY_DFU_ERROR_DATA;
        if (productId == GetU32(GetPacketData(packet, PACKET_DATA_NO_OFFSET) ) )
        {
            EnterResponse(packet, rspSize, state);
            status = CY_DFU_SUCCESS;
        }
    }
    else
    {
        /* Empty */
    }
    
    params->appId = params->appId; /* Remove the unused warning */

    return (status);
}


/*******************************************************************************
* Function Name: CopyToDataBuffer
****************************************************************************//**
*
* This is a helper function, called in the following functions:
* - DFU_CommandProgramData
* - CommandVerifyData
* - CommandSendData
*
* This is used to copy packet data to dataBuffer and increase dataOffset.
*
* \param dataBuffer     The pointer to a buffer containing the data to be
*                       written to an NVM.
* \param dataOffset     The offset within dataBuffer, indicates the current
*                       dataBuffer length.
* \param packet         The pointer to the DFU packet buffer.
* \param packetSize     The length in bytes of the data in the DFU
*                       packet buffer.
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t CopyToDataBuffer(uint8_t dataBuffer[], uint32_t *dataOffset, uint8_t const packet[], 
                                                uint32_t packetSize)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_UNKNOWN;
    if ( (dataBuffer != NULL) && (dataOffset != NULL) && (packet != NULL) )
    {
        status = CY_DFU_ERROR_LENGTH;
        if ( (*dataOffset + packetSize) <= CY_DFU_SIZEOF_DATA_BUFFER )
        {
            status = CY_DFU_SUCCESS;
            (void) memcpy( &dataBuffer[*dataOffset], packet, packetSize);
            *dataOffset += packetSize;
        }
    }
    return (status);
}


/*******************************************************************************
* Function Name: CommandProgramData
****************************************************************************//**
*
* This a helper function for Cy_DFU_Continue().
* This function is used to program data to an NVM.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandProgramData(uint8_t  *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{   
    cy_en_dfu_status_t status = CY_DFU_ERROR_LENGTH;
    uint8_t  *dataBufferLocal =  params->dataBuffer;
    uint32_t *dataOffsetLocal = &params->dataOffset;
    uint32_t  packetSize =  GetPacketDSize(packet);

    *rspSize = CY_DFU_RSP_SIZE_0;
    if (packetSize >= PARAMS_SIZE)
    {
        uint32_t address =  GetU32( GetPacketData(packet, PACKET_DATA_NO_OFFSET) );
        uint32_t crc = GetU32( GetPacketData(packet, PROGRAM_DATA_CRC_OFFSET) );

        /* Data may be sent with the Program Data DFU command, so copy it to dataBuffer */
        status = CopyToDataBuffer(dataBufferLocal, dataOffsetLocal, GetPacketData(packet, PARAMS_SIZE),
                                  packetSize - PARAMS_SIZE );
        
        if (status == CY_DFU_SUCCESS)
        {
            if (crc != Cy_DFU_DataChecksum(dataBufferLocal, *dataOffsetLocal, params) )
            {
                status = CY_DFU_ERROR_CHECKSUM;
            }
        }
        if (status == CY_DFU_SUCCESS)
        {
            status = Cy_DFU_WriteData(address, *dataOffsetLocal, CY_DFU_IOCTL_BHP, params);
        }
        if (status == CY_DFU_SUCCESS)
        {
            status = Cy_DFU_ReadData (address, *dataOffsetLocal, CY_DFU_IOCTL_COMPARE, params);
        }
    } /* if (packetSize >= PARAMS_SIZE) */
    *dataOffsetLocal = 0u;
    return (status);
}


#if CY_DFU_OPT_ERASE_DATA != 0
/*******************************************************************************
* Function Name: CommandEraseData
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function erases an NVM row or page.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandEraseData(uint8_t  *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_LENGTH;
    *rspSize = CY_DFU_RSP_SIZE_0;
    if (GetPacketDSize(packet) == DATA_PACKET_SIZE_4BYTES)
    {
        uint32_t address = GetU32( GetPacketData(packet, PACKET_DATA_NO_OFFSET) );
        status = Cy_DFU_WriteData(address, 0u, CY_DFU_IOCTL_ERASE, params);
    }
    params->dataOffset = 0u;
    return (status);
}
#endif /* CY_DFU_OPT_ERASE_DATA != 0 */


#if CY_DFU_OPT_VERIFY_DATA != 0
/*******************************************************************************
* Function Name: CommandVerifyData
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function verifies an NVM row or page.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandVerifyData(uint8_t  *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status    = CY_DFU_ERROR_LENGTH;
    uint8_t  *dataBufferLocal =  params->dataBuffer;
    uint32_t *dataOffsetLocal = &params->dataOffset;

    uint32_t  packetSize = GetPacketDSize(packet);
    *rspSize = CY_DFU_RSP_SIZE_0;
    if (packetSize >= PARAMS_SIZE)
    {
        uint32_t address = GetU32( GetPacketData(packet, PACKET_DATA_NO_OFFSET) );
        uint32_t crc     = GetU32( GetPacketData(packet, VERIFY_DATA_CRC_OFFSET) );

        /* Data may be sent with the Program Data DFU command, so copy it to dataBuffer */
        status = CopyToDataBuffer(dataBufferLocal, dataOffsetLocal, GetPacketData(packet, PARAMS_SIZE),
                                  packetSize - PARAMS_SIZE );
        
        if (status == CY_DFU_SUCCESS)
        {
            if (crc != Cy_DFU_DataChecksum(dataBufferLocal, *dataOffsetLocal, params) )
            {
                status = CY_DFU_ERROR_CHECKSUM;
            }
        }

        if (status == CY_DFU_SUCCESS)
        {
            status = Cy_DFU_ReadData(address, *dataOffsetLocal, CY_DFU_IOCTL_COMPARE, params);
            status = (status == CY_DFU_SUCCESS) ? CY_DFU_SUCCESS : CY_DFU_ERROR_VERIFY;
        }
    }
    *dataOffsetLocal = 0u;
    return (status);
}
#endif /* CY_DFU_OPT_VERIFY_DATA != 0 */


#if CY_DFU_OPT_SEND_DATA != 0
/*******************************************************************************
* Function Name: CommandSendData
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function handles the Send Data DFU command.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandSendData(uint8_t  *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status;
    uint32_t packetSize = GetPacketDSize(packet);
    uint8_t  *dataBufferLocal =  params->dataBuffer;
    uint32_t *dataOffsetLocal = &params->dataOffset;
    *rspSize = CY_DFU_RSP_SIZE_0;
    
    /* Data may be sent with the Program Data DFU command, so copy it to dataBuffer */
    status = CopyToDataBuffer(dataBufferLocal, dataOffsetLocal, 
                                GetPacketData(packet, PACKET_DATA_NO_OFFSET), 
                                packetSize);
    
    return (status);
}
#endif /* CY_DFU_OPT_SEND_DATA != 0 */


#if CY_DFU_OPT_VERIFY_APP != 0
/*******************************************************************************
* Function Name: CommandVerifyApp
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function handles the Verify Application DFU command.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandVerifyApp(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status  = CY_DFU_ERROR_LENGTH;
    uint32_t packetSize = GetPacketDSize(packet);
    *rspSize = CY_DFU_RSP_SIZE_0;
    

    if ( (packetSize == sizeof(uint32_t) ) || (packetSize == VERIFY_APP_DATA_SIZE) )
    {
        uint32_t app = (uint32_t) *GetPacketData(packet,PACKET_DATA_NO_OFFSET);
        if (app < CY_DFU_MAX_APPS)
        {
            status = Cy_DFU_ValidateApp(app, params);
        }
        else
        {
            status = CY_DFU_ERROR_VERIFY;
        }
    }

    if ( (status == CY_DFU_SUCCESS) || (status == CY_DFU_ERROR_VERIFY) )
    {
        uint8_t *valid = GetPacketData(packet, PACKET_DATA_NO_OFFSET);
        *valid = (status == CY_DFU_SUCCESS) ? 1u : 0u;
        status = CY_DFU_SUCCESS;
        *rspSize = CY_DFU_RSP_SIZE_VERIFY_APP;
    }
    return (status);
}
#endif /* CY_DFU_OPT_VERIFY_APP != 0 */

#if CY_DFU_METADATA_WRITABLE != 0
/*******************************************************************************
* Function Name: Cy_DFU_SetAppMetadata
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function sets application metadata and updates a metadata checksum.
* \note If the application metadata is the same as already
* present in the NVM, then the NVM is not rewritten and this function only exits.
* \note This function uses params->dataBuffer for the read and write NVM.
*
* \param appId           The application number to update metadata for.
* \param verifyAddress   An application verified area start address   
* \param verifySize      The size of verified application area
* \param params          The pointer to a DFU parameters structure.
*                        See \ref cy_stc_dfu_params_t.
* 
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_SetAppMetadata(uint32_t appId, uint32_t verifyAddress, uint32_t verifySize, 
                                                   cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;
    uint32_t metadataAddress = 0u;
    uint32_t metadataLength  = 0u;
    if ((params == NULL) || (appId >= CY_DFU_MAX_APPS))
    {
        status = CY_DFU_ERROR_UNKNOWN;
    }
    if (status == CY_DFU_SUCCESS)
    {
        metadataAddress = ElfSymbolToAddr(&__cy_boot_metadata_addr  );
        metadataLength  = ElfSymbolToAddr(&__cy_boot_metadata_length);
        status = Cy_DFU_ReadData(metadataAddress, metadataLength, CY_DFU_IOCTL_READ, params);
    }
    if (status == CY_DFU_SUCCESS)
    {    
        uint32_t getVerifyAddress = GetU32(params->dataBuffer + (appId * METADATA_BYTES_PER_APP));
        uint32_t getVerifySize      = GetU32(params->dataBuffer + (appId * METADATA_BYTES_PER_APP) + 
                                            METADATA_APP_LENGTH_OFFSET);
        if ( (getVerifyAddress != verifyAddress) || ( getVerifySize != verifySize) )
        {
            uint32_t crc;
            PutU32(params->dataBuffer, (appId * METADATA_BYTES_PER_APP)     , verifyAddress);
            PutU32(params->dataBuffer, (appId * METADATA_BYTES_PER_APP) + METADATA_APP_LENGTH_OFFSET, verifySize);
            crc = Cy_DFU_DataChecksum(params->dataBuffer, metadataLength - CRC_CHECKSUM_LENGTH, params);
            PutU32(params->dataBuffer, metadataLength - CRC_CHECKSUM_LENGTH, crc);
            status = Cy_DFU_WriteData(metadataAddress, metadataLength, CY_DFU_IOCTL_WRITE, params);
        }
    }
    return (status);
}
#endif /* CY_DFU_METADATA_WRITABLE != 0 */


/*******************************************************************************
* Function Name: CommandSetAppMetadata
****************************************************************************//**
* This is a helper function for Cy_DFU_Continue().
* This function handles the Set Application Metadata DFU command.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandSetAppMetadata(uint8_t *packet, uint32_t *rspSize, 
                                                     cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;

    *rspSize = CY_DFU_RSP_SIZE_0;

    if (GetPacketDSize(packet) != DATA_LENGTH)
    {
        status = CY_DFU_ERROR_LENGTH;
    }
    
    if (status == CY_DFU_SUCCESS)
    {
        /* Data offsets 0, 1, 5 are defined in the DFU Packet Structure */
        uint32_t app       =      *( GetPacketData(packet, PACKET_DATA_NO_OFFSET) );
        
    #if CY_DFU_METADATA_WRITABLE != 0
        uint32_t verifyAddress = GetU32( GetPacketData(packet, SET_APP_METADATA_OFFSET) );
        uint32_t verifySize   = GetU32( GetPacketData(packet, SET_APP_METADATA_LENGTH_OFFSET) );
        
        status = Cy_DFU_SetAppMetadata(app, verifyAddress, verifySize, params);
    #endif
        
        params->appId = app;
    }
    return (status);
}


#if CY_DFU_OPT_GET_METADATA != 0
/*******************************************************************************
* Function Name: CommandGetMetadata
****************************************************************************//**
* This is a helper function for Cy_DFU_Continue().
* This function handles the Get Metadata DFU command.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
*
*******************************************************************************/
static cy_en_dfu_status_t CommandGetMetadata(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;
    uint32_t locRspSize = CY_DFU_RSP_SIZE_0;
    
    if (packet == NULL)
    {
        status = CY_DFU_ERROR_UNKNOWN;
    }
    if (status == CY_DFU_SUCCESS)
    {
        if (GetPacketDSize(packet) != DATA_PACKET_SIZE_4BYTES)
        {
            status = CY_DFU_ERROR_LENGTH;
        }
    }
    if (status == CY_DFU_SUCCESS)
    {
        uint32_t fromAddr = GetU16( GetPacketData(packet, PACKET_DATA_NO_OFFSET) );
        uint32_t toAddr   = GetU16( GetPacketData(packet, GET_METADATA_TO_OFFSET) );
        if ( (toAddr < fromAddr)
          || ( ( (toAddr - fromAddr) + CY_DFU_PACKET_MIN_SIZE) > CY_DFU_SIZEOF_CMD_BUFFER) )
        {
            status  = CY_DFU_ERROR_DATA;
        }
        else
        {
            uint32_t metadataAddr    = ElfSymbolToAddr(&__cy_boot_metadata_addr  );
            uint32_t metadataLength  = ElfSymbolToAddr(&__cy_boot_metadata_length);

            status = Cy_DFU_ValidateMetadata(metadataAddr, params);
            if (status == CY_DFU_SUCCESS)
            {
                status = Cy_DFU_ReadData(metadataAddr, metadataLength, CY_DFU_IOCTL_READ, params);
            }
            if (status == CY_DFU_SUCCESS)
            {
                locRspSize = (toAddr - fromAddr);
                (void) memmove( GetPacketData(packet, PACKET_DATA_NO_OFFSET), 
                                &(params->dataBuffer[fromAddr]), 
                                locRspSize);
                status = CY_DFU_SUCCESS;          
            }
        }
    }
    *rspSize = locRspSize; 
    return (status);
}
#endif /* CY_DFU_OPT_GET_METADATA != 0 */


#if CY_DFU_OPT_SET_EIVECTOR != 0
/*******************************************************************************
* Function Name: CommandSetEIVector
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function copies the Encryption Initial Vector value from the DFU
* command buffer to the buffer pointed by 
* \code params->encryptionVector \endcode.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t.
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandSetEIVector(uint8_t *packet, uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_SUCCESS;
    *rspSize = CY_DFU_RSP_SIZE_0;
    
    uint32_t size = GetPacketDSize(packet);
    
    if (( (size == 0u) ||  (size == DATA_PACKET_SIZE_8BYTES) 
        || (size == DATA_PACKET_SIZE_16BYTES) ) && (params->encryptionVector != NULL))
    {
        (void) memcpy(params->encryptionVector, GetPacketData(packet, PACKET_DATA_NO_OFFSET), size);
    }
    else
    {
        status = CY_DFU_ERROR_DATA;
    }
    
    return (status);
}
#endif /* CY_DFU_OPT_SET_EIVECTOR != 0 */


/*******************************************************************************
* Function Name: CommandUnsupported
****************************************************************************//**
*
* This is a helper function for Cy_DFU_Continue().
* This function responses to an unsupported command.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t CommandUnsupported(uint8_t packet[], uint32_t *rspSize, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_CMD;

    /* To remove the compiler warnings */
    (void)packet;
    params->dataOffset = 0u;

    *rspSize = CY_DFU_RSP_SIZE_0;
    return (status);
}


/*******************************************************************************
* Function Name: ContinueHelper
****************************************************************************//**
* This is a helper function for Cy_DFU_Continue(). This function parses
*  most of the DFU commands.
*
* \param packet     The pointer to the DFU packet buffer.
* \param rspSize    The pointer to a response packet size.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t .
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
static cy_en_dfu_status_t ContinueHelper(uint32_t command, uint8_t *packet, uint32_t *rspSize,
                                              cy_stc_dfu_params_t *params, bool *noResponse)
{
    /* Set to a close warning (status may be used uninitialized)*/
    cy_en_dfu_status_t status = CY_DFU_ERROR_UNKNOWN; 
    switch (command)
    {
    case CY_DFU_CMD_PROGRAM_DATA:
        status = CommandProgramData(packet, rspSize, params);
        break;

#if CY_DFU_OPT_VERIFY_DATA != 0
    case CY_DFU_CMD_VERIFY_DATA:
        status = CommandVerifyData(packet, rspSize, params);
        break;
#endif /* CY_DFU_OPT_VERIFY_DATA != 0 */

#if CY_DFU_OPT_ERASE_DATA != 0
    case CY_DFU_CMD_ERASE_DATA:
        status = CommandEraseData(packet, rspSize, params);
        break;
#endif /* CY_DFU_OPT_ERASE_DATA != 0 */

#if CY_DFU_OPT_VERIFY_APP != 0
    case CY_DFU_CMD_VERIFY_APP:
        status = CommandVerifyApp(packet, rspSize, params);
        break;
#endif /* CY_DFU_OPT_VERIFY_APP != 0 */

#if CY_DFU_OPT_SEND_DATA != 0
    case CY_DFU_CMD_SEND_DATA_WR:
        *noResponse = true;
        status = CommandSendData(packet, rspSize, params);
        break;
        
    case CY_DFU_CMD_SEND_DATA:
        status = CommandSendData(packet, rspSize, params);
        break;
#endif /* CY_DFU_NO_CMD_SEND_DATA == 0 */

    case CY_DFU_CMD_SYNC: /* If something fails, then the Host sends this command to reset the DFU */
        params->dataOffset = 0u;
        *noResponse = true;
        break;

    case CY_DFU_CMD_SET_APP_META:
        status = CommandSetAppMetadata(packet, rspSize, params);
        break;

#if CY_DFU_OPT_GET_METADATA != 0
    case CY_DFU_CMD_GET_METADATA:
        status = CommandGetMetadata(packet, rspSize, params);
        break;
#endif /* CY_DFU_OPT_GET_METADATA != 0 */

#if CY_DFU_OPT_SET_EIVECTOR != 0
    case CY_DFU_CMD_SET_EIVECTOR:
        status = CommandSetEIVector(packet, rspSize, params);
        break;
#endif /* CY_DFU_OPT_SET_EIVECTOR != 0 */

    default:
        status = CommandUnsupported(packet, rspSize, params);
        break;
    } /* switch (command) */
    return (status);
}


/*******************************************************************************
* Function Name: Cy_DFU_Continue
****************************************************************************//**
*
* This function causes the DFU to attempt to read data being transmitted
* by the Host application.  If data is sent from the Host, this establishes the
* communication interface to process all requests.
*
* \param state      The pointer to a updating state variable.
*                   See \ref group_dfu_macro_state.
* \param params     The pointer to a DFU parameters structure.
*                   See \ref cy_stc_dfu_params_t.
*
* \return See \ref cy_en_dfu_status_t
* 
*******************************************************************************/
cy_en_dfu_status_t Cy_DFU_Continue(uint32_t *state, cy_stc_dfu_params_t *params)
{
    cy_en_dfu_status_t status = CY_DFU_ERROR_UNKNOWN; /* Give a value to a close warning */
    uint8_t *packet = params->packetBuffer; /* Receive/Transmit buffer */  

    uint32_t rspSize = CY_DFU_RSP_SIZE_0;
    bool noResponse = false;        /* Indicates whether to send a response packet back to the Host */

    CY_ASSERT(params->timeout != 0u);
    CY_ASSERT(params->dataBuffer != NULL);
    CY_ASSERT(params->packetBuffer != 0);

    
    if ( (*state == CY_DFU_STATE_NONE) || (*state == CY_DFU_STATE_UPDATING) )
    {
        status = ReadVerifyPacket(packet, &noResponse, params->timeout);
        if (status == CY_DFU_SUCCESS)
        {
            uint32_t command = GetPacketCommand(packet);

            if      (command == CY_DFU_CMD_ENTER)
            {
                status = CommandEnter(packet, &rspSize, state, params);
            }
            else if (command == CY_DFU_CMD_EXIT)
            {
                *state = CY_DFU_STATE_FINISHED;
                noResponse = true;
            }
            else if (*state != CY_DFU_STATE_UPDATING)
            {
                status = CY_DFU_ERROR_CMD;
            }
            else
            {
                status = ContinueHelper(command, packet, &rspSize, params, &noResponse);
            }
        }

        if (!noResponse)
        {
            (void) WritePacket(status, packet, rspSize);
        }
    }
    else
    {
        /* empty */
    }
    return (status);
}


/* [] END OF FILE */
