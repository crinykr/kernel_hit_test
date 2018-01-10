/****************************************************************************************
*****************************************************************************************
***                                                                                   ***
***                                 Copyright (c) 2011                                ***
***                                                                                   ***
***                                Conexant Systems, Inc.                             ***
***                                                                                   ***
***                                 All Rights Reserved                               ***
***                                                                                   ***
***                                    CONFIDENTIAL                                   ***
***                                                                                   ***
***               NO DISSEMINATION OR USE WITHOUT PRIOR WRITTEN PERMISSION            ***
***                                                                                   ***
*****************************************************************************************
**
**  File Name:
**      CxCodecAPI.h
**
**  Abstract:
**      structure and APIs for Conexant I2S-based Audio Codec.. 
**      
**  Product Name:
**      Conexant i2s-based Audio codec.
**
**  Version : 1.0.0.10
**
**  Remark:
**      
** 
********************************************************************************
**  Revision History
**      Date        Description                                 Author
**      06/29/11    Created.                                    Simon Ho
**      06/30/11    Added SPI support and basic audio control.  Simon Ho
**      07/13/11    Added new function for changing PCM2        Simon Ho
**                  sampling rate.
**      08/24/11    Added support for PCM1.                     Simon Ho
**      10/05/11    Added new routing PCM1<=>PCM2               Simon Ho
**      11/07/11    Added support for swtiching between PCM1 
**                  and PCM2 on the fly                         Simon Ho
**      12/19/11    Added support for 21z codec.                Simon Ho
**      02/02/12    Fixed unaligned memory access problem       Simon Ho
**      06/05/12    Added support for both diff in and out      Simon Ho
********************************************************************************
*****************************************************************************************/
// Vendor version : 351423.111119

//#define  __BYTE_ORDER       __LITTLE_ENDIAN
#define  __LITTLE_ENDIAN    4321
#define  __BYTE_ORDER       4321
#define  MAX_BUF_LENGTH	    64

#ifdef __cplusplus
extern "C"{
#endif 

typedef int (*fun_I2CSPIWriteThenRead)(  void * pCallbackContext,
                                      unsigned char ChipAddr, 
                                      unsigned long cbBuf,
                                      unsigned char* pBuf,
                                      unsigned long cbReadBuf, 
                                      unsigned char*pReadBuf);

typedef int (*fun_I2CSPIWrite)(  void * pCallbackContext,
                              unsigned char ChipAddr,
                              unsigned long cbBuf, 
                              unsigned char* pBuf);
////////////////////////////////////////////////////////////////////////////////////////
//
// Initialization routines.
//
////////////////////////////////////////////////////////////////////////////////////////

/*
 * Setting up the I2C/SPI Write callback function.
 * 
 * PARAMETERS
 *  
 *    pCallbackContext [in] - A pointer to a caller-defined structure of data items
 *                            to be passed as the context parameter of the callback
 *                            routine each time it is called. 
 *
 *    pfnI2CSPIWrite   [in] - A pointer to a fun_I2CSPIWrite callback routine, which is to 
 *                            write I2C data. The callback routine must conform to 
 *                            the following prototype:
 * 
 *                        int (*fun_I2CSPIWrite)(  
 *                                void * pCallbackContext,
 *                                unsigned char ChipAddr,
 *                                unsigned long cbBuf, 
 *                                unsigned char* pBuf
 *                             );
 *
 *                        The callback routine parameters are as follows:
 *
 *                        pCallbackContext [in] - A pointer to a caller-supplied 
 *                                                context area as specified in the
 *                                                first parameter of SetupI2cWriteCallback. 
 *                        ChipAddr         [in] - The I2C chip address, this will be ignord 
 *                                                 when SPI mode.
 *                        cbBuf            [in] - The size of the input buffer, in bytes.
 *                        pBuf             [in] - A pointer to the input buffer that contains 
 *                                                the data required to perform the operation.
 *
 *
 *    cbMaxWriteBufSize [in] - Specify the maximux transfer size for a I2C/SPI continue 
 *                            writing without 'STOP'. This is limited in I2C/SPI bus Master
 *                            device. The size can not less then 3 since Channel 
 *                            requires 2 address bytes plus a data byte.
 *                              
 *
 *
 * RETURN
 *      None
 *
 */
void CxSetupI2cSpiWriteCallback( void * pCallbackContext,
                            fun_I2CSPIWrite      pfnI2CSPIWrite,
                            unsigned long        cbMaxWriteBufSize);


/*
 * Setting up the I2C/SPI WriteThenRead callback function.
 * 
 * PARAMETERS
 *  
 *    pCallbackContext    [in] - A pointer to a caller-defined structure of data items
 *                               to be passed as the context parameter of the callback
 *                               routine each time it is called. 
 *
 *    pfnI2CSPIWriteThenRead [in] - A pointer to a fun_I2CSPIWriteThenRead callback 
 *                               routine, which is to read data from codec. The callback 
 *                               routine must conform to the following prototype:
 *
 *                               int (*fun_I2cWriteThenRead)(  
 *                                    void * pCallbackContext,
 *                                    unsigned char ChipAddr,
 *                                    unsigned long cbBuf, 
 *                                    unsigned char* pBuf,
 *                                    unsigned long cbReadBuf,
 *                                    unsigned char* pReadBuf,
 *                                 );
 *
 *                               The callback routine parameters are as follows:
 *
 *                               pCallbackContext [in] - A pointer to a caller-supplied 
 *                                                       context area as specified in the
 *                                                       CallbackContext parameter of 
 *                                                       SetupI2cWriteCallback. 
 *                               ChipAddr         [in] - The i2c chip address.
 *                               cbBuf            [in] - The size of the input buffer, in bytes.
 *                               pBuf             [in] - A pointer to the input buffer that contains 
 *                                                       the data required to perform the operation.
 *                               cbReadBuf        [in] - The size of the output buffer, in bytes.
 *                               pReadBuf         [in] - A pointer to the buffer that contains 
 *                                                       the return data .
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 *
 */
void CxSetupI2cSpiWriteThenReadCallback( void * pCallbackContext,
                fun_I2CSPIWriteThenRead I2cWriteThenReadPtr);

/*
 * Initializes codec and sets serial communication type, sets the codec type, and download
 * firmware if it is required. 
 *
 * PARAMETERS
 *  
 *    bI2cSpi            [in] - Specifies desired serial communication type.
 *                              0 : I2C bus
 *                              1 : SPI bus
 *
 *    nCodecType        [in] - Specifies desired codec type.
 *                              1 : cx2070x
 *                              2 : cx2074x
 *
 *    bDownloadFW        [in] - Specifies whether to download firwmare. If this parameter is non-zero, 
 *                              then it will download firmware to codec. 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
 *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
 *    this function.
 */
int CxInitialize( int bI2cSpi, int nCodecType, int bDownloadFW);

//#define CX_CODEC_TYPE_AUTO_DECT 0 not implement yet
#define CX_CODEC_TYPE_CX2070X   1
#define CX_CODEC_TYPE_CX2074X   2

#define CX_SERIAL_I2C           0 
#define CX_SERIAL_SPI           1



/*
 * returns the version of the firmware that codec is running on.
 * 
 * PARAMETERS
 *  
 *    None.
 *
 * RETURN
 *  
 *    Returns the version of the firmware.

 * 
 * REMARKS
 *    *For cx2070x codec only*
 *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling 
 *    SetupI2cWriteCallback and SetupI2cWriteThenReadCallback before you call this function.
 */
unsigned int CxGetFirmwareVersion(void);


/*
 * returns the patch version of the firmware that codec is running on.
 * 
 * PARAMETERS
 *  
 *    None.
 *
 * RETURN
 *  
 *    Returns the version of the firmware.

 * 
 * REMARKS
 *    *For cx2070x -21z only*
 *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling 
 *    SetupI2cWriteCallback and SetupI2cWriteThenReadCallback before you call this function.
 */
unsigned int CxGetFirmwarePatchVersion(void);


/*
 * Sets the PCM1 format
 *
 * PARAMETERS
 *  
 *    nSampleRate         [in] - Specifies desired sampling rate. The value can be one of the following values.
 *                              8 : 8  KHz.
 *                              16: 16 KHz.
 *                              24: 24 KHz
 *                              32: 32 KHz
 *                              44: 44 KHz
 *                              48: 48 KHz
 *
 *    nSampleWidth        [in] - Specifies desired sample width. The value can be one of the following values.
 *                              8  : 8 bits per sample
 *                              16 : 16 bits per sample.
 *
 *    nChannelMask        [in] - Specifies desired channels to be used. The value can be one of the following values.
 *                              1: Left channel only(Mono)
 *                              2: Rigth channel only(Mono)
 *                              3: Stereo.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
 *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
 *    this function.
 */
int CxSetsPCM1Format( int nSampleRate, int nSampleWidth, int nChannelMask);

/*
 * Sets the PCM2 format
 *
 * PARAMETERS
 *  
 *    nSampleRate         [in] - Specifies desired sampling rate. The value can be one of the following values.
 *                              8 : 8  KHz.
 *                              16: 16 KHz.
 *                              24: 24 KHz
 *                              32: 32 KHz
 *                              44: 44 KHz
 *                              48: 48 KHz
 *
 *    nSampleWidth        [in] - Specifies desired sample width. The value can be one of the following values.
 *                              8  : 8 bits per sample
 *                              16 : 16 bits per sample.
 *
 *    nChannelMask        [in] - Specifies desired channels to be used. The value can be one of the following values.
 *                              1: Left channel only(Mono)
 *                              2: Rigth channel only(Mono)
 *                              3: Stereo.
 *    bSlaveToPCM1        [in] - Specifies whether the PCM2 slave to PCM1.The value can be one of the following values.
 *                              0 : non-slave mode. (default)
 *                              1 : slave to PCM 1 port.
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
 *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
 *    this function.
 */
int CxSetsPCM2Format( int nSampleRate, int nSampleWidth, int nChannelMask, int bSlaveToPCM1);



////////////////////////////////////////////////////////////////////////////////////////
//
// Volume Control routines.
//
////////////////////////////////////////////////////////////////////////////////////////

/*
 * Sets Headphone volume level. 
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetHeadphoneVolume(int nLevel);

/*
 * Sets Speaker volume level. (Class-D)
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetSpeakerVolume(int nLevel);

/*
 * Sets Mono Out volume level. 
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetMonoOutVolume(int nLevel);


/*
 * Sets Microphone volume level. 
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *    ChannelMask       - Specifies which channel to be controled.
 *                          1 means Left only.
 *                          2 means Right only. 
 *                          3 means both L & R.  
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_CX_ERRNO_NOERR.
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxSetMicrophoneVolume(int nLevel , int ChannelMask);

/*
 * Sets LineIn volume level. 
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetLineInVolume(int nLevel);

/*
 * Sets Headphone Mute state. 
 * 
 * PARAMETERS
 *  
 *    nMute - Specifiese the mute state 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetHeadphoneMute(int nMute);

/*
 * Sets Speaker Mute state. 
 * 
 * PARAMETERS
 *  
 *    nMute - Specifiese the mute state 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetSpeakerMute(int nMute);

/*
 * Sets Headphone Mute state. 
 * 
 * PARAMETERS
 *  
 *    nMute - Specifiese the mute state 
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetMonoOutMute(int nMute);


/* Sets Micorphone Mute state. 
 * 
 * PARAMETERS
 *  
 *    nMute			 - Specifiese the mute state.
 *    ChannelMask     - Specifies the channel to be set.
 *                     1: Left Channel.
 *                     2: Rigth Channel
 *                     3: Both L& R Channel.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetMicrophoneMute(int nMute, int ChannelMask);

/*
 * Sets Line In Mute state. 
 * 
 * PARAMETERS
 *  
 *    nMute - Specifiese the mute state.
 *    ChannelMask     - Specifies the channel to be set.
 *                     1: Left Channel.
 *                     2: Rigth Channel
 *                     3: Both L& R Channel.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetLineInMute(int nMute, int ChannelMask);



////////////////////////////////////////////////////////////////////////////////////////
//
// Routing Control routines.
//
////////////////////////////////////////////////////////////////////////////////////////


/*
 * Sets Input Srouce of PCM 1
 * 
 * PARAMETERS
 *  
 *    nInputSource - this parameter can be one of the following values.
 *                   0x00 means Turn off
 *                   0x01 means Microphone L only
 *                   0x02 means Microhphn  R only
 *                   0x03 means Microhphn L + R
 *                   0x08 means Line 1
 *                   0x10 means USB
 *                   0x20 means PCM2
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 */
int CxEnablePCM1Input(int nInput);

/*
 * Sets Output Destination of PCM 1
 * 
 * PARAMETERS
 *             
*    nOutput     - this parameter can be one or more of the following values.
 *                   0x00 means None, turn off
 *                   0x01 means Class D
 *                   0x02 means Headphone
 *                   0x08 means Mono Out
 *                   0x10 means PCM2 port
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxEnablePCM1Output(int nOutput);

/*
 * Sets Input Srouce of PCM 2
 * 
 * PARAMETERS
 *  
 *    nInputSource - this parameter can be one of the following values.
 *                   0x00 means Turn off
 *                   0x01 means Microphone L only
 *                   0x02 means Microhphn  R only
 *                   0x03 means Microhphn L + R
 *                   0x08 means Line 1 (Signal End)
 *                   0x10 means Line 2
 *                   0x20 means Line 3
 *                   0x88 means Line 1 (Differential)
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 */
int CxEnablePCM2Input(int nInput);

/*
 * Sets Output Destination of PCM 2
 * 
 * PARAMETERS
 *             
 *    nOutput     - this parameter can be one or more of the following values.
 *                   0x00 means None, turn off
 *                   0x01 means Class D mono
 *                   0x02 means Headphone
 *                   0x04 means LineOut (Single End)
 *                   0x08 means Mono Out
 *                   0x10 means PCM1 port
 *                   0x81 means Class D stereo
 *                   0x24 means LineOut (Differential)
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxEnablePCM2Output(int nOutput);

/*
 * Sets Analog to Analog loopback path.
 * 
 * PARAMETERS
 *  
 *    nPath        - this parameter can be one or more of the following values.
 *                   0X0038 means Microphone to Mono Out
 *                   0X0032 means Microphone to Headphone
 *                   0X0031 means Microphone to Class D
 *                   0X8800 means LineIn-1 to Mono
 *                   0X8200 means LineIn-1 to Heaphone 
 *                   0X8100 means LineIn-1 to Class D
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxEnableAnalogToAnalogLoopback(int nPath);

////////////////////////////////////////////////////////////////////////////////////////
//
// DSP Control routines.
//
////////////////////////////////////////////////////////////////////////////////////////

/*
 * Enables or disables AGC
 * 
 * PARAMETERS
 *  
 *    bAGCOn       - Specifies whether the AGC are enabled or disabled. If this parameter 
 *                   is non-zero, the AGC will be enabled. Otherwise, the AGC will be 
 *                   disabled.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxEnableAGC(int bAGCOn);

/*
 * Enables or disables DRC
 * 
 * PARAMETERS
 *  
 *    bDRCOn       - Specifies whether the DRC are enabled or disabled. If this parameter 
 *                   is non-zero, the DRC will be enabled. Otherwise, the DRC will be 
 *                   disabled.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxEnableDRC(int bDRCOn);

/*
 * Sets the AEC ProEnables or disables DRC
 * 
 * PARAMETERS
 *  
 *    nProfile     - Specifies the profile number. 0 means diable AEC.
                             
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetAECProfile( int nProfileNum);

/*
 * Sets the PCM1 clock
 * 
 * PARAMETERS
 *
 *    bEnable    - Specifies whether the clock are enabled or disabled. 0: disable. 1:enable.
 *  
 *    nClockRate -Specifies the clock rate, this parameter can be one of the following values.
 *                0x0    6.144 MHz
 *                0x1    4.096 MHz
 *                0x2    3.072 MHz
 *                0x3    2.048 MHz
 *                0x4    1.536 MHz
 *                0x5    1.024 MHz
 *                0x6    768 Khz
 *                0x7    512 Khz
 *                0x8    384 Khz
 *                0x9    256 Khz
 *                0xa    5.644 MHz
 *                0xb    2.822 MHz
 *                0xc    1.411 MHz
 *                0xd    705 Khz
 *                0xe    352 Khz
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetPCM1Clock(int bEnable, int nClockRate );

/*
 * Sets the PCM2 clock
 * 
 * PARAMETERS
 *
 *    bEnable    - Specifies whether the clock are enabled or disabled. 0: disable. 1:enable.
 *  
 *    nClockRate -Specifies the clock rate, this parameter can be one of the following values.
 *                0x0    6.144 MHz
 *                0x1    4.096 MHz
 *                0x2    3.072 MHz
 *                0x3    2.048 MHz
 *                0x4    1.536 MHz
 *                0x5    1.024 MHz
 *                0x6    768 Khz
 *                0x7    512 Khz
 *                0x8    384 Khz
 *                0x9    256 Khz
 *                0xa    5.644 MHz
 *                0xb    2.822 MHz
 *                0xc    1.411 MHz
 *                0xd    705 Khz
 *                0xe    352 Khz
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 */
int CxSetPCM2Clock(int bEnable, int nClockRate );

/*Error codes*/
#define CX_ERRNO_NOERR                 0
#define CX_ERRNO_SRC_FILE_NOT_EXIST    101
#define CX_ERRNO_WRITE_FILE_FAILED     102
#define CX_ERRNO_INVALID_DATA          103
#define CX_ERRNO_CHECKSUM_FAILED       104
#define CX_ERRNO_FAILED                105
#define CX_ERRNO_INVALID_PARAMETER     106
#define CX_ERRNO_NOMEM                 107
#define CX_ERRNO_I2CFUN_NOT_SET        108
#define CX_ERRNO_UPDATE_MEMORY_FAILED  109
#define CX_ERRNO_DEVICE_NOT_RESET      110
#define CX_ERRNO_UNSUPPORT_CODEC       111
#define CX_ERRNO_NOT_IMPLEMENT         112
#define CX_ERRNO_NOT_RESPONSE          113
#define CX_ERRNO_DEVICE_OUT_OF_CONTROL 114
#define CX_ERRNO_DSP_LOCKUP            115




//#define CX2070X_AVOID_USE_STREAM4 

#ifdef __cplusplus
}
#endif 


