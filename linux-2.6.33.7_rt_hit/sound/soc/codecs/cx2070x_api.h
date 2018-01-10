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
**      Cx2070x_api.h
**
**  Abstract:
**      structure and APIs for Conexant cx2070xCodec.
**      
**
**  Product Name:
**      Conexant i2s-based Audio codec.
**
**  Version : 1.0.0.10 
**
**  Remark:
**      This is for internal use, please don't include this file or call the following 
**      functions directly. 
**
** 
********************************************************************************
**  Revision History
**      Date        Description                                 Author
**      06/29/11    Created                                     Simon Ho
**      07/13/11    Added 16KHz sampling rate support.          Simon Ho
**      08/24/11    Added support for PCM1.                     Simon Ho
**      10/05/11    Added new routing PCM1<=>PCM2               Simon Ho
**      11/07/11    Added support for swtiching between PCM1 
**                  and PCM2 on the fly                         Simon Ho
**      02/02/12    Fixed unaligned memory access problem       Simon Ho 
**      06/05/12    Added support for both diff in and out      Simon Ho
********************************************************************************
*****************************************************************************************/
// Vendor version : 351423.111119
#ifdef __cplusplus
extern "C"{
#endif 


/*
 * Download Firmware to Channel.
 * 
 * PARAMETERS
 *  
 *    pRomData            [in] - A pointer fo the input buffer that contains rom data.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
 *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
 *    this function.
 */
int DownloadFW(const unsigned char *const pRomData);

/*
 * Initializes the register value from a array 
 * 
 * PARAMETERS
 *  
 *    pRegTable            [in] - A pointer to the input buffer that contains register data.
 *    nLen                 [in] - Number of register to program.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
 *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
 *    this function.
 */
int InitRegisterFromTable(short *pRegTable, int nLen);


#ifdef __cplusplus
}
#endif 

