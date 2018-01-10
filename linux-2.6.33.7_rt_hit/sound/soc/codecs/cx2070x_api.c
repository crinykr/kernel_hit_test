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
**      Cx2070x_API.c
**
**  Abstract:
**      functions for Conexant cx2070x codec API. 
**      
**
**  Product Name:
**      Conexant cx2070x audio codec.
**
**  Version : 1.0.0.10 
**  Remark:
****  For wide band phone turning, increases the sampling rate to 16 KHz. 
**
** 
********************************************************************************
**  Revision History
**      Date        Description                                 Author
**      06/30/11    Created.                                    Simon Ho
**      07/13/11    Added 16KHz sampling rate support.          Simon Ho
**      08/24/11    Added support for PCM1.                     Simon Ho
**      10/05/11    Added new routing PCM1<=>PCM2               Simon Ho
**      11/07/11    Added support for swtiching between PCM1 
**                  and PCM2 on the fly                         Simon Ho
**      12/19/11    Added support for 21z codec.                Simon Ho
**      02/02/12    Fixed unaligned memory access problem       Simon Ho
**      06/05/12    Added support for both diff in and out      Simon Ho
********************************************************************************
*****************************************************************************************/

#if defined(_MSC_VER) 
// microsoft windows environment.
#define  __BYTE_ORDER       __LITTLE_ENDIAN
#define  __LITTLE_ENDIAN    1234

#include <stdlib.h>   // For _MAX_PATH definition
#include <stdio.h>
#include <string.h>
#include <malloc.h>


void * __cdecl malloc( size_t _Size);
#define msleep(_x_) 
void __cdecl msleep(unsigned long dwMilliseconds );
void delay_m(unsigned int m);

int printk(const char *s, ...);
#define KERN_ERR "<3>"
#define true 1
#define false 0
#elif defined(__KERNEL__)  
// linux kernel environment.
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#else
//
// linux user mode environment.
//
#include <stdlib.h>   // For _MAX_PATH definition
#include <stdio.h>
#include <string.h> 	
//#include <unistd.h>    
#endif

#include "cxcodecapi.h"
#include "cx2070x_api.h"
#include "cx2070x_fw.h"
#include "cx2070x_init.h"

#define ENABLE_I2C_BURST_READ 1

#if defined( __BIG_ENDIAN) && !defined(__BYTE_ORDER)
#define __BYTE_ORDER __BIG_ENDIAN
#elif defined( __LITTLE_ENDIAN ) && !defined(__BYTE_ORDER)
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif 

#ifndef __BYTE_ORDER
#error __BYTE_ORDER is undefined.
#endif 



typedef enum _I2C_STATE{I2C_OK,I2C_ERR,I2C_RETRY} I2C_STATE;

#if defined(ENABLE_SHOW_PROGRESS) 
void ShowProgress(int curPos,int bForceRedraw, I2C_STATE eState, const int MaxPos);
void InitShowProgress(const int MaxPos);
#else 
#define InitShowProgress(MaxPos)
#define ShowProgress(curPos,bForceRedraw,eState, axPos)
#endif


#ifndef NULL
#define NULL 0
#endif //#ifndef NULL

#define S_DESC          "Cnxt Channel Firmware"  /*Specify the string that will show on head of rom file*/
#define S_ROM_FILE_NAME "cx2070x.fw"            /*Specify the file name of rom file*/
#define CHIP_ADDR        0x14                    /*Specify the i2c chip address*/
#define MEMORY_UPDATE_TIMEOUT  300
#define MAX_ROM_SIZE (1024*1024)
//#define DBG_ERROR  "ERROR  : "
//#define DBG_ERROR  KERN_ERR 		// LG-E
#define DBG_ERROR			// LG-E
#define DBG_INFO			// LG-E	
#define LOG( _msg_ )  printk  _msg_ 
//#define LOG( _msg_ )  printf _msg_ // LG-E

typedef struct CX_CODEC_ROM_DATA
{
#ifdef USE_TYPE_DEFINE
    unsigned long      Type;
#endif //#ifdef USE_TYPE_DEFINE
    unsigned long      Length;
    unsigned long      Address;
    unsigned char      data[1];
}CX_CODEC_ROM_DATA;

#define ROM_DATA_TYPE_S37           0xAA55CC01 // S37 format.
#define ROM_DATA_TYPE_CNXT          0xAA55CC04 // Conexant SPI format.
#define ROM_DATA_SEPARATED_LINE     0x23232323 //()()()()

typedef struct CX_CODEC_ROM{
    char                        sDesc[24]; 
    char                        cOpenBracket;
    char                        sVersion[5];
    char                        cCloseBracket;
    char                        cEOF;
    unsigned long               FileSize;
    unsigned long               LoaderAddr;
    unsigned long               LoaderLen;
    unsigned long               CtlAddr;
    unsigned long               CtlLen;
    unsigned long               SpxAddr;
    unsigned long               SpxLen;
    struct CX_CODEC_ROM_DATA    Data[1];
}CX_CODEC_ROM;

// To convert two digital ASCII into one BYTE.
unsigned char ASCII_2_BYTE( char ah, char al) ;

#define BUF_SIZE 0x1000
#define BIBF(_x_) if(!(_x_)) break;
#define BIF(_x_) if((ErrNo=(_x_)) !=0) break;


#ifndef BIBF
#define BIBF( _x_ ) if(!(_x_)) break;
#endif 

enum { 
    MEM_TYPE_RAM     = 1 /* CTL*/, 
    MEM_TYPE_SPX     = 2,
    MEM_TYPE_EEPROM  = 3
}; 


typedef struct PORT_FORMAT{
    int nSamplingRate; //sampling rate in Hz.
    int nSampleWidth;  //the size of sample in bytes.
    int nChannelMask;  
}PORT_FORMAT,*PPORT_FORMAT;

//Digital to digital loopback mode.
typedef enum DD_LOOPBACK{
    DD_LOOPBACK_OFF                = 0,
    DD_LOOPBACK_PCM2_SLAVE_TO_PCM1 = 1,
    DD_LOOPBACK_PCM1_SLAVE_TO_PCM2 = 2,
};

typedef struct _CONFIG{
    PORT_FORMAT   oPCM1;
    PORT_FORMAT   oPCM2;
}CONFIG,PCONFIG;

#undef current

typedef struct _CX2070X{
    int cbSize;   //the size of this structure in BYTES.
    CONFIG   current;
    CONFIG   desired;
}CX2070X,*PCX20709;

CX2070X                 g_cx2070x;
fun_I2CSPIWriteThenRead g_I2cWriteThenReadPtr      = NULL;
fun_I2CSPIWrite         g_I2cWritePtr              = NULL;
unsigned char *      g_AllocatedBuffer          = NULL;
unsigned char *      g_Buffer                   = NULL;
unsigned long        g_cbMaxWriteBufSize        = 0;
void *               g_pContextI2cWrite         = NULL;
void *               g_pContextI2cWriteThenRead = NULL;
int                  g_nSerialMode              = 0; //0:i2c, 1: SPI
int                  g_nCodecType               = 0; //auto


unsigned char gBuf[MAX_BUF_LENGTH] = {0};         

void delay_m(unsigned int m)
{
  volatile unsigned int i = 0;
  m = 9000*m;
  for(i = 0; i<m; i++);
}


int SetsPCM1Format(void);
int SetsPCM2Format( int bEnableDDLoopback);

/*
* The SetupI2cWriteCallback sets the I2cWrite callback function.
* 
* PARAMETERS
*  
*    pCallbackContext [in] - A pointer to a caller-defined structure of data items
*                            to be passed as the context parameter of the callback
*                            routine each time it is called. 
*
*    I2cWritePtr      [in] - A pointer to a i2cwirte callback routine, which is to 
*                            write I2C data. The callback routine must conform to 
*                            the following prototype:
* 
*                        int (*fun_I2cWrite)(  
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
*                                                CallbackContext parameter of 
*                                                SetupI2cWriteCallback. 
*                        ChipAddr         [in] - The i2c chip address.
*                        cbBuf            [in] - The size of the input buffer, in bytes.
*                        pBuf             [in] - A pointer to the input buffer that contains 
*                                                the data required to perform the operation.
*
*
*    cbMaxWriteBuf    [in] - Specify the maximux transfer size for a I2c continue 
*                            writing with 'STOP'. This is limited in I2C bus Master
*                            device. The size can not less then 3 since Channel 
*                            requires 2 address bytes plus a data byte.
*                              
*
*
* RETURN
*  
*    None
*
*/
void CxSetupI2cSpiWriteCallback( void * pCallbackContext,
    fun_I2CSPIWrite         I2cWritePtr,
    unsigned long        cbMaxWriteBufSize)
{
    g_pContextI2cWrite  = pCallbackContext;
    g_I2cWritePtr       = I2cWritePtr;
    g_cbMaxWriteBufSize = cbMaxWriteBufSize;
}

/*
* The SetupI2cWriteThenReadCallback sets the SetupI2cWriteThenRead callback function.
* 
* PARAMETERS
*  
*    pCallbackContext    [in] - A pointer to a caller-defined structure of data items
*                               to be passed as the context parameter of the callback
*                               routine each time it is called. 
*
*    I2cWriteThenReadPtr [in] - A pointer to a i2cwirte callback routine, which is to 
*                               write I2C data. The callback routine must conform to 
*                               the following prototype:
*
*                        int (*fun_I2cWriteThenRead)(  
*                                void * pCallbackContext,
*                                unsigned char ChipAddr,
*                                unsigned long cbBuf, 
*                                unsigned char* pBuf,
*                                unsigned long cbReadBuf,
*                                unsigned char* pReadBuf,
*                             );
*
*                        The callback routine parameters are as follows:
*
*                         pCallbackContext [in] - A pointer to a caller-supplied 
*                                                 context area as specified in the
*                                                 CallbackContext parameter of 
*                                                 SetupI2cWriteCallback. 
*                         ChipAddr         [in] - The i2c chip address.
*                         cbBuf            [in] - The size of the input buffer, in bytes.
*                         pBuf             [in] - A pointer to the input buffer that contains 
*                                                 the data required to perform the operation.
*                         cbReadBuf        [in] - The size of the output buffer, in bytes.
*                         pReadBuf         [in] - A pointer to the buffer that contains 
*                                                 the return data .
* RETURN
*      None
* 
*/
void CxSetupI2cSpiWriteThenReadCallback( void * pCallbackContext,
                fun_I2CSPIWriteThenRead I2cWriteThenReadPtr)
{
    g_pContextI2cWriteThenRead  = pCallbackContext;
    g_I2cWriteThenReadPtr       = I2cWriteThenReadPtr;
}


/*
* Convert a 4-byte number from a ByteOrder into another ByteOrder.
*/
unsigned long ByteOrderSwapULONG(unsigned long i)
{
    return((i&0xff)<<24)+((i&0xff00)<<8)+((i&0xff0000)>>8)+((i>>24)&0xff);
}

/*
* Convert a 2-byte number from a ByteOrder into another ByteOrder.
*/
unsigned short ByteOrderSwapWORD(unsigned short i)
{
    return ((i>>8)&0xff)+((i << 8)&0xff00);
}

/*
* Convert a 4-byte number from generic byte order into Big Endia
*/
unsigned long ToBigEndiaULONG(unsigned long i)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return ByteOrderSwapULONG(i);
#else
    return i;
#endif
}


/*
* Convert a 2-byte number from generic byte order into Big Endia
*/
unsigned short ToBigEndiaWORD(unsigned short i)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return ByteOrderSwapWORD(i);
#else
    return i;
#endif
}

/*
* Convert a 4-byte number from Big Endia into generic byte order.
* 
* For some cpu that doesn't support unaligned memory access, we access the memory one by one and one byte at a time.
*/
unsigned long FromBigEndiaULONGP(unsigned char *p)
{
    unsigned char *by;
    unsigned long i;
#if __BYTE_ORDER == __LITTLE_ENDIAN
    by = (unsigned char*)p;
    i = by[0];
    i<<=8;
    i += by[1];
    i<<=8;
    i += by[2];
    i<<=8;
    i += by[3];
#else
    by = (unsigned char*)p;
    i =  by[3];
    i<<=8;
    i += by[2];
    i<<=8;
    i += by[1];
    i<<=8;
    i += by[0];
#endif
    return i;
}


/*
* Convert a 2-byte number from Big Endia into generic byte order.
*/
unsigned short FromBigEndiaWORDP(unsigned char *p)
{
    unsigned char *by;
    unsigned short i;
#if __BYTE_ORDER == __LITTLE_ENDIAN
    by = (unsigned char*)p;
    i = by[0];
    i<<=8;
    i += by[1];
#else
    by = (unsigned char*)p;
    i =  by[1];
    i<<=8;
    i += by[0];
#endif
    return i;
}


/*
* To convert two digital ASCII into one BYTE.
*/
unsigned char ASCII_2_BYTE( char ah, char al) 
{
    unsigned char ret = '\0';
    int i =2;

    for(;i>0;i--)
    {
        if( 'a' <= ah && 'f' >= ah)
        {
            ret += ah - 'a'+10;
        }
        else if( 'A' <= ah && 'F' >= ah)
        {
            ret += ah -'A'+10;
        }

        else if( '0' <= ah && '9' >= ah)
        {
            ret += ah - '0';
        }
        else
        {
            LOG((DBG_ERROR "Invalid txt data.\n"));

            // ErrNo = CX_ERRNO_INVALID_DATA;
            break;
        }
        ah =al;
        if(i==2)
            ret = (unsigned short)ret << 4;
    }
    return ret;
}

/*
* Read a byte from the specified  register address.
* 
* PARAMETERS
*  
*    RegAddr             [in] - Specifies the register address.
*
* RETURN
*  
*    Returns the byte that is read from the specified register address.
*
*/
unsigned char ReadReg(unsigned short RegAddr)
{

    unsigned char RegData;
    unsigned char spibuf[3]; // the register is 2 bytes-wide. 

    if(!g_I2cWriteThenReadPtr)
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        return 0;
    }

    
    if(g_nSerialMode == CX_SERIAL_SPI)
    {
        spibuf[0] =(unsigned char) (RegAddr >>8);
        spibuf[1] =(unsigned char) (RegAddr );
        spibuf[2] = 0;           //must be zero.
        g_I2cWriteThenReadPtr(g_pContextI2cWriteThenRead,CHIP_ADDR,
            3,spibuf,1,&RegData);
    }
    else
    {
        RegAddr = ToBigEndiaWORD(RegAddr);
        g_I2cWriteThenReadPtr(g_pContextI2cWriteThenRead,CHIP_ADDR,
            2,(unsigned char*) &RegAddr,1,&RegData);
    }

    return RegData;
}


/*
* Write a byte from the specified register address.
* 
* PARAMETERS
*  
*    RegAddr             [in] - Specifies the register address.
*
* RETURN
*  
*    Returns the byte that is read from the specified register address.
*
* REMARK
* 
*    The g_I2cWriteThenReadPtr must be set before calling this function.
*/
int WriteReg(unsigned short RegAddr, unsigned char RegData)
{
    unsigned char WrBuf[3];
    int timeout = 10;
    unsigned short NEWC =0x117d;

    if(!g_I2cWritePtr)
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        return CX_ERRNO_I2CFUN_NOT_SET;
    }

    // Workaround !
    // prevent CPU from accessing unaligned memory, we can't use the following 
    // *((unsigned short*) WrBuf) = ToBigEndiaWORD(RegAddr);
    // 
    WrBuf[0] = (unsigned char) (RegAddr >>8);
    WrBuf[1] = (unsigned char) (RegAddr);
    WrBuf[2] = RegData;

    if(g_nSerialMode == CX_SERIAL_SPI)
    {
        WrBuf[0] |= 0x80; // SPI writing flag.
    }
    g_I2cWritePtr(g_pContextI2cWrite,CHIP_ADDR,sizeof(WrBuf),WrBuf);

    if( RegAddr == NEWC)
    {
        for(;timeout;timeout--)
        {
//            msleep(1);
			delay_m(1);
            RegData = ReadReg( NEWC);
            if( (RegData & 1) == 0) 
            {
                break;
            }
        }
        if( timeout == 0 )
        {
            return CX_ERRNO_NOT_RESPONSE;
        }
    }

    return CX_ERRNO_NOERR;
}

/*
*  Writes a number of bytes from a buffer to Channel via I2C bus.
*  
* PARAMETERS
*  
*    NumOfBytes         [in] - Specifies the number of bytes to be written
*                              to the memory address.
*    pData              [in] - Pointer to a buffer from an array of I2C data
*                              are to be written.
*  
* RETURN
*  
*    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
*    Otherwise, return ERRON_* error code. 
*/
int ChannelI2cBurstWrite( unsigned long NumOfBytes, unsigned char *pData)
{
    int ErrNo           = CX_ERRNO_NOERR;
    unsigned short CurAddr;

    //unsigned char  *pDataEnd            = pData + NumOfBytes;
    unsigned char  *pCurData            = pData;
    unsigned short *pCurAddrByte        = NULL;
    unsigned long  BytesToProcess       = 0;
    unsigned short backup               = 0;
    const unsigned long cbAddressBytes  = 2;
    const unsigned long cbMaxDataLen    = g_cbMaxWriteBufSize-cbAddressBytes;


    if(!g_I2cWritePtr )
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        return CX_ERRNO_I2CFUN_NOT_SET;
    }

    //assert(NumOfBytes < 3);
    CurAddr = FromBigEndiaWORDP( pData);

    //skip first 2 bytes data (address).
    NumOfBytes -= cbAddressBytes;
    pCurData   += cbAddressBytes;

    for(;NumOfBytes;)
    {
        BytesToProcess = NumOfBytes > cbMaxDataLen? cbMaxDataLen : NumOfBytes;
        NumOfBytes-= BytesToProcess;
        // save the pervious 2 bytes for later use.
        pCurAddrByte = (unsigned short*) (pCurData -cbAddressBytes);
        //backup       = *pCurAddrByte;
        memcpy(&backup,pCurAddrByte,2);

//        *pCurAddrByte=  ToBigEndiaWORD(CurAddr);
        ((unsigned char*) pCurAddrByte)[0] = (unsigned char)( CurAddr>>8);
        ((unsigned char*) pCurAddrByte)[1] = (unsigned char)( CurAddr);
        if(g_nSerialMode == CX_SERIAL_SPI)
        {
            pCurAddrByte[0] |= 0x80; // SPI writing flag.
        }
        BIBF(g_I2cWritePtr(g_pContextI2cWrite,CHIP_ADDR, BytesToProcess + cbAddressBytes,(unsigned char*)pCurAddrByte));
        
//        delay_m(25);  // 2012.11.12 mail from Ryu
        
        //restore the data 
        memcpy(pCurAddrByte,&backup,2);

        pCurData += BytesToProcess;
        CurAddr  += (unsigned short)BytesToProcess;
    }
    
    
    return ErrNo;
}

/*
*  Writes a number of bytes from a buffer to the specified memory address.
*
* PARAMETERS
*
*    dwAddr             [in] - Specifies the memory address.
*    NumOfBytes         [in] - Specifies the number of bytes to be written
*                              to the memory address.
*    pData              [in] - Pointer to a buffer from an struct of 
*                              CX_CODEC_ROM_DATA is to be written.
*    MemType            [in] - Specifies the requested memory type, the value must be from 
*                              the following table.
*
*                              MEM_TYPE_RAM     = 1
*                              MEM_TYPE_SPX     = 2
*                              MEM_TYPE_EEPROM  = 3
*
* RETURN
*  
*    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
*    Otherwise, return ERRON_* error code. 
*/
int CxWriteMemory(unsigned long dwAddr, unsigned long NumOfBytes, unsigned char * pData, int MemType )
{
    int ErrNo           = CX_ERRNO_NOERR;
    unsigned char      Address[4];
    unsigned char      WrData[8];
    unsigned char      offset = 0;
    const unsigned long MAX_BUF_LEN = 0x100;
    unsigned char      cr = 0;
    int                 bNeedToContinue = 0;
    int                 i=0;
    unsigned long       ulTemp;


    const unsigned long cbAddressBytes  = 2;

    unsigned short *    pAddressByte;
    unsigned char *pEndData  = pData + NumOfBytes;
    unsigned short      RegMemMapAddr = ToBigEndiaWORD(0x300);
    unsigned long       BytesToProcess = 0;

    while(NumOfBytes)
    {

        BytesToProcess = NumOfBytes <= MAX_BUF_LEN ? NumOfBytes : MAX_BUF_LEN;
        NumOfBytes -= BytesToProcess;
        pEndData  = pData + BytesToProcess;

        // Workaround !
        // prevent CPU from accessing unaligned memory, we can't use the following 
        // *((unsigned long*)&Address) = ToBigEndiaULONG(dwAddr);
        // 
        Address[0] = (unsigned char) (dwAddr>>24);
        Address[1] = (unsigned char) (dwAddr>>16);
        Address[2] = (unsigned char) (dwAddr>>8);
        Address[3] = (unsigned char) (dwAddr);

        //        dwAddr += offset;
        offset = 0;

        if( !bNeedToContinue )
        {
            //
            //  Update the memory target address and buffer length.
            //
            WrData[0] = 0x02;    //update target address Low 0x02FC 
            WrData[1] = 0xFC;
            WrData[2] = Address[3];
            WrData[3] = Address[2];
            WrData[4] = Address[1];
            WrData[5] = (unsigned char)BytesToProcess -1 ;  // X bytes - 1

            if(g_nSerialMode == CX_SERIAL_SPI)
            {
                WrData[0] |= 0x80; // SPI writing flag.
            }
            BIBF(g_I2cWritePtr(g_pContextI2cWrite,CHIP_ADDR, 6 , WrData));
        }

        //
        //  Update buffer.
        //
        pAddressByte = (unsigned short*) (pData - cbAddressBytes);
        memcpy(gBuf, pAddressByte, BytesToProcess+cbAddressBytes);
        *((unsigned short*)gBuf) = RegMemMapAddr;
        ChannelI2cBurstWrite(BytesToProcess+cbAddressBytes, (unsigned char*)gBuf);
        pData = pEndData;

        //
        // Commit the changes and start to transfer buffer to memory.
        //
        if( MemType == MEM_TYPE_RAM)
        {
            cr = 0x81;
        }
        else if( MemType == MEM_TYPE_EEPROM)
        {
            cr = 0x83;
        }
        else if( MemType == MEM_TYPE_SPX)
        {
            cr = 0x85;
            if( bNeedToContinue )
            {
                cr |= 0x08;
            }
        }

        WrData[0] = 0x04;   // UpdateCtl [0x400]
        WrData[1] = 0x00;
        WrData[2] = cr;   // start to transfer  
        if(g_nSerialMode == CX_SERIAL_SPI)
        {
            WrData[0] |= 0x80; // SPI writing flag.
        }
        BIBF(g_I2cWritePtr(g_pContextI2cWrite,CHIP_ADDR, 3 , WrData));

        for(i = 0;i<MEMORY_UPDATE_TIMEOUT;i++)
        {
            // loop until the writing is done.
            WrData[0] = ReadReg(0x0400);
            if(!( WrData[0] & 0x80 ))
            {
                //done
                break;
            }
            else
            {
                //pending
                //msleep(1);
                delay_m(1);
                continue;
            }
        }

        if( i == MEMORY_UPDATE_TIMEOUT)
        {
            //Failed to update memory.
            LOG( (DBG_ERROR "memory update timeout.\n"));
            ErrNo = CX_ERRNO_UPDATE_MEMORY_FAILED;

            break;
        }
        //if ( i >= 1) 
        //{
        //    //printk( KERN_ERR "write pending loop =%d\n", i);
        //}	

        bNeedToContinue = 1; 
    }while(0);

    return ErrNo ;
}



#define WAIT_UNTIL_DEVICE_READY(_x_,_err_msg_) \
for (timeout=0;timeout<dev_ready_time_out;timeout++) \
{                                                    \
    Ready = ReadReg(0x1000);                         \
    if (Ready _x_) break;                            \
    delay_m(10);                                      \
};                                                   \
if( timeout == dev_ready_time_out)                   \
{                                                    \
    LOG((DBG_ERROR _err_msg_)); \
    ErrNo = CX_ERRNO_DEVICE_OUT_OF_CONTROL;          \
    break;                                           \
}            

/*
 * Download Firmware to Channel.
 * 
 * PARAMETERS
 *  
 *    pRomBin            [in] - A pointer fo the input buffer that contains rom data.
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling 
 *    SetupI2cWriteCallback and SetupI2cWriteThenReadCallback before you call this function.
 */
int DownloadFW(const unsigned char * const pRomBin)
{
    int ErrNo = CX_ERRNO_NOERR;
    struct CX_CODEC_ROM      *pRom  = (struct CX_CODEC_ROM  *)pRomBin;
    struct CX_CODEC_ROM_DATA *pRomData;
    struct CX_CODEC_ROM_DATA *pRomDataEnd;
    unsigned char            *pData;
    unsigned char            *pDataEnd;
    unsigned long            CurAddr = 0;
    unsigned long            cbDataLen = 0;
    unsigned char            Ready;
    unsigned long            curProgress = 0;
    unsigned long            TotalLen    = 0;
    unsigned long            i = 0;
    const unsigned long      dev_ready_time_out = 100;
    int                      bIsRomVersion  = 0;           
    const char               CHAN_PATH[]="CNXT CHANNEL PATCH";    
    unsigned long            timeout;
    unsigned long            fwVer;
    unsigned long            fwPatchVer;

    do{
        if(pRom == NULL ||g_Buffer == NULL)
        {
            ErrNo = CX_ERRNO_INVALID_PARAMETER;
            LOG( (DBG_ERROR "Invalid parameter.\n"));
            break;
        }

        if( NULL == g_I2cWriteThenReadPtr||
            NULL == g_I2cWritePtr)
        {
            ErrNo = CX_ERRNO_I2CFUN_NOT_SET;
            LOG( (DBG_ERROR "i2C function is not set.\n"));
            break;
        }


        //check if codec is ROM version
        if(0==memcmp(CHAN_PATH,pRom->sDesc,sizeof(CHAN_PATH)-1))
        {
            bIsRomVersion = 1;
        }

        if(bIsRomVersion)
        {
            WAIT_UNTIL_DEVICE_READY(== 0X01,"cx2070x: Timed out waiting for codecto be ready!\n");
        }
        else
        {
            //Check if there is a FIRMWARE present. the Channel should get
            // a clear reset signal before we download firmware to it.
            if( (ReadReg(0x009) & 0x04) == 0)
            {
                LOG((DBG_ERROR "cx2070x: did not get a clear reset..!"));
                ErrNo = CX_ERRNO_DEVICE_NOT_RESET;
                break;
            }
        }

    
        TotalLen = FromBigEndiaULONGP((unsigned char*)&pRom->LoaderLen) + FromBigEndiaULONGP((unsigned char*)&pRom->CtlLen) + FromBigEndiaULONGP((unsigned char*)&pRom->SpxLen);
       // InitShowProgress(TotalLen);

        //Download the loader.
        pRomData    = (struct CX_CODEC_ROM_DATA *) ( (char*)pRom + FromBigEndiaULONGP((unsigned char*)&pRom->LoaderAddr));
        pRomDataEnd = (struct CX_CODEC_ROM_DATA *) ((char*)pRomData +FromBigEndiaULONGP((unsigned char*)&pRom->LoaderLen));

        for( ;pRomData!=pRomDataEnd;)
        {

            pData   = &pRomData->data[0];
            pDataEnd= pData + FromBigEndiaULONGP((unsigned char*)&pRomData->Length) - sizeof(unsigned long); 
            memcpy(gBuf, pData-2, FromBigEndiaULONGP((unsigned char*)&pRomData->Length) - sizeof(unsigned short));
            BIF(ChannelI2cBurstWrite( FromBigEndiaULONGP((unsigned char*)&pRomData->Length) - sizeof(unsigned short), gBuf));
            curProgress +=  FromBigEndiaULONGP((unsigned char*)&pRomData->Length) ;
            ShowProgress(curProgress,false, I2C_OK,TotalLen);
            pRomData = (struct CX_CODEC_ROM_DATA *)pDataEnd;

        }

        
        //* check if the device is ready.
        if(bIsRomVersion)
        {
            WAIT_UNTIL_DEVICE_READY(== 0X01,"cx2070x: Timed out waiting for cx2070x to be ready after loader downloaded!\n");
        }
        else
        {
            WAIT_UNTIL_DEVICE_READY(!= 0xFF,"cx2070x: Timed out waiting for cx2070x to be ready after loader downloaded!\n");
        }

       
        //Download the CTL
        pRomData    = (struct CX_CODEC_ROM_DATA *) ( (char*)pRom + FromBigEndiaULONGP((unsigned char*)&pRom->CtlAddr ));
        pRomDataEnd = (struct CX_CODEC_ROM_DATA *) ((char*)pRomData +FromBigEndiaULONGP((unsigned char*)&pRom->CtlLen));

        for( ;pRomData!=pRomDataEnd;)
        {
            CurAddr = FromBigEndiaULONGP((unsigned char*)&pRomData->Address);
            pData       = &pRomData->data[0];
            cbDataLen   = FromBigEndiaULONGP((unsigned char*)&pRomData->Length) ;
            BIF(CxWriteMemory(CurAddr,cbDataLen -sizeof(unsigned long) , pData, MEM_TYPE_RAM ));
            // The next RoMData position = current romData position + cbDataLen + sizeof( data len bytes)
            pRomData  =   (struct CX_CODEC_ROM_DATA *)((char*) pRomData + cbDataLen + sizeof(unsigned long));  

            curProgress +=  cbDataLen ;
            ShowProgress(curProgress,false, I2C_OK,TotalLen);
        }

        //
        // Download SPX code.
        // 
        pRomData    = (struct CX_CODEC_ROM_DATA *) ( (char*)pRom + FromBigEndiaULONGP((unsigned char*)&pRom->SpxAddr ));
        pRomDataEnd = (struct CX_CODEC_ROM_DATA *) ((char*)pRomData +FromBigEndiaULONGP((unsigned char*)&pRom->SpxLen));

        for( ;pRomData!=pRomDataEnd;)
        {
            CurAddr = FromBigEndiaULONGP((unsigned char*)&pRomData->Address);
            pData       = &pRomData->data[0];
            cbDataLen   = FromBigEndiaULONGP((unsigned char*)&pRomData->Length) ;
            BIF(CxWriteMemory(CurAddr,cbDataLen -sizeof(unsigned long) , pData, MEM_TYPE_SPX ));
            // The next RoMData position = current romData position + cbDataLen + sizeof( data len bytes)
            pRomData  =   (struct CX_CODEC_ROM_DATA *)((char*) pRomData + cbDataLen + sizeof(unsigned long));  

            curProgress +=  cbDataLen ;
            ShowProgress(curProgress,false, I2C_OK,TotalLen);
        }

        if(ErrNo != 0) break;

        ShowProgress(TotalLen,false, I2C_OK,TotalLen);

        //
        // Reset
        //
        if(bIsRomVersion)
        {
            WriteReg(0x1000,0x00);
         //   msleep(400); //delay 400 ms
        }
        else
        {
            WriteReg(0x400,0x40);
            //msleep(400); //delay 400 ms
            delay_m(400);
        }
       
       WAIT_UNTIL_DEVICE_READY(== 0x01,"cx2070x: Timed out waiting for cx2070x to be ready after firmware downloaded!\n");

        //check if XPS code is working or not.

        WriteReg(0x117d,0x01);
        for (timeout=0;timeout<dev_ready_time_out;timeout++) 
        {                                                    
            Ready = ReadReg(0x117d);                         
            if (Ready == 0x00) break;   
            printk("ReadReg(0x117d) == %d, retry=%d\n", Ready, timeout);                         
            //msleep(1); 
            delay_m(50);
        };                          
        if( timeout == dev_ready_time_out)                   
        {                                                    
            LOG((DBG_ERROR "cx2070x: DSP lockup! download firmware failed!")); 
            ErrNo = CX_ERRNO_DSP_LOCKUP;          
            break;                                           
        } 

        fwVer = CxGetFirmwareVersion();
        if(bIsRomVersion)
        {
            fwPatchVer = CxGetFirmwarePatchVersion();
            LOG((DBG_INFO "cx2070x: firmware download successfully! FW: %u,%u,%u, FW Patch: %u,%u,%u\n",
                (unsigned char)(fwVer>>16),  
                (unsigned char)(fwVer>>8),  
                (unsigned char)fwVer,
                (unsigned char)(fwPatchVer>>16),  
                (unsigned char)(fwPatchVer>>8),  
                (unsigned char)fwPatchVer));
        }
        else
        {
            LOG((DBG_INFO "cx2070x: firmware download successfully! FW: %u,%u,%u\n",
                (unsigned char)(fwVer>>16),  
                (unsigned char)(fwVer>>8),  
                (unsigned char)fwVer));
        }

    }while(0);

    return ErrNo;
}

//
///*
// * Initializes codec and sets serial communication type, sets the codec type, and download
// * firmware if it is required. 
// *
// * PARAMETERS
// *  
// *    bI2cSpi            [in] - Specifies desired serial communication type.
// *                              0 : I2C bus
// *                              1 : SPI bus
// *
// *    nCodecType        [in] - Specifies desired codec type.
// *                              0 : auto dect.
// *                              1 : cx2070x
// *                              2 : cx2074x
// *
// *    bDownloadFW        [in] - Specifies whether to download firwmare. If this parameter is non-zero, 
// *                              then it will download firmware to codec. 
// *                              0 : none
// *                              1 : update the firmware.
// * RETURN
// *  
// *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
// *    Otherwise, return ERRON_* error code. 
// * 
// * REMARKS
// *  
// *    You need to set up both I2C/SPI Write and I2C/SPI WriteThenRead callback function 
// *    by calling SetupI2cSpiWriteCallback and SetupI2cSpiWriteThenReadCallback before you call 
// *    this function.
//*



int CxInitialize( int bI2cSpi, int nCodecType, int bDownloadFW)
{
    int ErrNo = CX_ERRNO_NOERR;
    int bNeedFW = 0;

    do{

        if( NULL == g_I2cWriteThenReadPtr||
            NULL == g_I2cWritePtr)
        {
            ErrNo = CX_ERRNO_I2CFUN_NOT_SET;
            LOG( (DBG_ERROR "i2C function is not set.\n"));
            break;
        }

        g_nSerialMode = bI2cSpi;

        switch(nCodecType)
        {
        case 0:
            ErrNo = CX_ERRNO_UNSUPPORT_CODEC;
            LOG( (DBG_ERROR "auto dect function is not implement yet.\n"));
            break;

        case CX_CODEC_TYPE_CX2070X:
            bNeedFW = bDownloadFW;
            g_nCodecType = nCodecType;
            LOG((DBG_INFO "Initialize cx2070x audio codec..\n"));
            break;

        // not implement yet.
        //case CX_CODEC_TYPE_CX2074X:
        //    g_nCodecType = nCodecType;
        //    LOG((DBG_INFO "Initialize cx2074x audio codec..\n"));
        //    break;

        default:
            ErrNo = CX_ERRNO_UNSUPPORT_CODEC;
            LOG( (DBG_ERROR "Unsupport codec type %d.\n", nCodecType));
            break;
        }
        if( ErrNo != CX_ERRNO_NOERR) break;
            
        //download firmware if needed.
        if(bNeedFW)
        {
#if defined(_MSC_VER) 
			g_AllocatedBuffer = (unsigned char*) malloc(0x200);
#elif defined(__KERNEL__)  
#else
			g_AllocatedBuffer = malloc(0x200);
#endif

//g_Buffer
//			g_Buffer = g_AllocatedBuffer +2;
			g_Buffer = &gBuf[0];
			g_Buffer = g_AllocatedBuffer +2;
			
            ErrNo = DownloadFW( ChannelFW ); // load firmware from c head file.

#if defined(_MSC_VER) 

            free(g_AllocatedBuffer);
#elif defined(__KERNEL__)  
#else
			free(g_AllocatedBuffer);
#endif
        }
        if( ErrNo != CX_ERRNO_NOERR) break;

        // Sets initial value to codec.
        ErrNo = InitRegisterFromTable(InitialRegTable,(sizeof(InitialRegTable)/sizeof(InitialRegTable[0]))/2);
        
        memset(&g_cx2070x,0,sizeof(g_cx2070x));
        g_cx2070x.cbSize = sizeof(g_cx2070x);
            
            
    }while(0);
    return ErrNo;
}

///*
// * returns the version of the firmware that codec is running on.
// * 
// * PARAMETERS
// *  
// *    None.
// *
// * RETURN
// *  
// *    Returns the version of the firmware.
// * 
// * REMARKS
// *    *For cx2070x codec only*
// *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling 
// *    SetupI2cWriteCallback and SetupI2cWriteThenReadCallback before you call this function.
// */***
unsigned int CxGetFirmwareVersion(void)
{
    unsigned int FwVersion = 0;
    int ErrNo;


    if( NULL == g_I2cWriteThenReadPtr||
        NULL == g_I2cWritePtr)
    {
        ErrNo = CX_ERRNO_I2CFUN_NOT_SET;
        LOG( (DBG_ERROR "i2C function is not set.\n"));
        return 0;
    }

    FwVersion = ReadReg(0x1002);
    FwVersion <<= 8;
    FwVersion |= ReadReg(0x1001);
    FwVersion <<= 8;
    FwVersion |= ReadReg(0x1006);

    return FwVersion;

}


///*
// * returns the version of the firmware that codec is running on.
// * 
// * PARAMETERS
// *  
// *    None.
// *
// * RETURN
// *  
// *    Returns the version of the firmware.
// * 
// * REMARKS
// *    *For cx2070x -21z  only*
// *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling 
// *    SetupI2cWriteCallback and SetupI2cWriteThenReadCallback before you call this function.
// */***
unsigned int CxGetFirmwarePatchVersion(void)
{
    unsigned int FwPatchVersion = 0;
    int ErrNo;


    if( NULL == g_I2cWriteThenReadPtr||
        NULL == g_I2cWritePtr)
    {
        ErrNo = CX_ERRNO_I2CFUN_NOT_SET;
        LOG( (DBG_ERROR "i2C function is not set.\n"));
        return 0;
    }

    FwPatchVersion = ReadReg(0x1584);
    FwPatchVersion <<= 8;
    FwPatchVersion |= ReadReg(0x1585);
    FwPatchVersion <<= 8;
    FwPatchVersion |= ReadReg(0x1586);

    return FwPatchVersion;

}

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
 *    Otherwise, return CX_CX_ERRNO_ error code. 
 */
int CxSetHeadphoneVolume(int nLevel)
{
    int ErrNo = CX_ERRNO_NOERR;
    const char DBSteps  = 75;
    const char DBOffset = -74;
    char db = (nLevel*DBSteps)/101 + DBOffset;
    ErrNo = WriteReg( 0x100d , db);
    ErrNo = ErrNo?ErrNo: WriteReg( 0x100e , db);
    return ErrNo;
}

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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxSetSpeakerVolume(int nLevel)
{
    int ErrNo = CX_ERRNO_NOERR;
    const char DBSteps  = 75;
    const char DBOffset = -74;
    char db = (nLevel*DBSteps)/101 + DBOffset;
    ErrNo = WriteReg( 0x100d , db);
    ErrNo = ErrNo?ErrNo: WriteReg( 0x100e , db);
    return ErrNo;
}


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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxSetMonoOutVolume(int nLevel)
{
    int ErrNo = CX_ERRNO_NOERR;
    const char DBSteps  = 75;
    const char DBOffset = -74;
    char db = (nLevel*DBSteps)/101 + DBOffset;
    ErrNo = WriteReg( 0x1012 , db);
    return ErrNo;
}

/*
 * Sets Microphone volume level. 
 * 
 * PARAMETERS
 *  
 *    nLevel - the output volume from 0 to 100. 
 *    ChannelMask      - 1 means Left only, 2 means Right only. 3 means both L & R.  
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxSetMicrophoneVolume(int nLevel , int ChannelMask)
{
    int ErrNo = CX_ERRNO_NOERR;
    const char DBSteps  = 75;
    const char DBOffset = -74;
    char db = (nLevel*DBSteps)/101 + DBOffset;
	
	if( ChannelMask & 1) 
    ErrNo = WriteReg( 0x1015 , db);
    if ( ChannelMask & 2) 
    ErrNo = ErrNo?ErrNo: WriteReg( 0x1016 , db);
    return ErrNo;
}


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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxSetLineInVolume(int nLevel)
{
    int ErrNo = CX_ERRNO_NOERR;
    const char DBSteps  = 75;
    const char DBOffset = -74;
    char db = (nLevel*DBSteps)/101 + DBOffset;
    ErrNo = WriteReg( 0x1013 , db);
    ErrNo = ErrNo?ErrNo: WriteReg( 0x1014 , db);
    return ErrNo;
}



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
int CxSetHeadphoneMute(int nMute)
{
    int ErrNo = CX_ERRNO_NOERR;
	unsigned char MuteReg  =  ReadReg(0x1018);
	unsigned char DAC1MuteMask = 0x03;

	MuteReg &= ~DAC1MuteMask;
	
	if( nMute ) MuteReg |= DAC1MuteMask;

    ErrNo = WriteReg( 0x1018 , MuteReg);
    return ErrNo;
}
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
int CxSetSpeakerMute(int nMute)
{
    int ErrNo = CX_ERRNO_NOERR;
	unsigned char MuteReg  =  ReadReg(0x1018);
	unsigned char DAC1MuteMask = 0x03;

	MuteReg &= ~DAC1MuteMask;
	
	if( nMute ) MuteReg |= DAC1MuteMask;

    ErrNo = WriteReg( 0x1018 , MuteReg);
    return ErrNo;
}

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
int CxSetMonoOutMute(int nMute)
{
	int ErrNo = CX_ERRNO_NOERR;
	unsigned char MuteReg  =  ReadReg(0x1018);
	unsigned char DAC1MuteMask = 0x04;

	MuteReg &= ~DAC1MuteMask;

	if( nMute ) MuteReg |= DAC1MuteMask;

	ErrNo = WriteReg( 0x1018 , MuteReg);
	return ErrNo;
}


/*
 * Sets Micorphone Mute state. 
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
int CxSetMicrophoneMute(int nMute, int ChannelMask)
{
	int ErrNo = CX_ERRNO_NOERR;
	unsigned char MuteReg  =  ReadReg(0x1018);
	unsigned char MuteMask = 0x18;

	MuteReg &= ~MuteMask;

	
	if( nMute ) 
	{
		if( ChannelMask & 0x1 )
		{
			MuteReg |= 0x10;
		}
		
		if( ChannelMask & 0x2 )
		{
			MuteReg |= 0x08;
		}

	}
	ErrNo = WriteReg( 0x1018 , MuteReg);
	return ErrNo;
}
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
int CxSetLineInMute(int nMute, int ChannelMask)
{

	int ErrNo = CX_ERRNO_NOERR;
	unsigned char MuteReg  =  ReadReg(0x1018);
	unsigned char MuteMask = 0x60;

	MuteReg &= ~MuteMask;

	
	if( nMute ) 
	{
		if( ChannelMask & 0x1 )
		{
			MuteReg |= 0x40;
		}
		
		if( ChannelMask & 0x2 )
		{
			MuteReg |= 0x20;
		}

	}
	ErrNo = WriteReg( 0x1018 , MuteReg);
	return ErrNo;
}




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
int CxEnablePCM1Input(int nInput)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char Mask    = (1<<5) | //stream 5
                            (1<<1) | //stream 1 ;
                            (1<<2) | //stream 2 ;
                            (1<<4) ; //stream 4 ;
    
	unsigned char DSPEnReg = ReadReg(0x117A);
	unsigned char MicDSPLRMask    = (1<<6) ; // a switch to switch MIC L/R input when DSP mode is enabled.
    unsigned char DSPVoiceIn = 0;

                                   
    InitReg &= ~Mask;

	DSPEnReg &= ~MicDSPLRMask;

    if( nInput  )
    {
        InitReg |= (1<<5) ; //stream 5 ;
        WriteReg( 0x1181, 0x06); // sets the PCM1 source to voice0
    }
    else
    {   // 0 means disable input,
        WriteReg( 0x1181, 0x00); // no connect.
    }

    if( nInput & 0x3 )
    {
        if((nInput & 0x3) == 0x3)
        {
            //both L & R
            WriteReg( 0x116B, 0xa2);
        }
        else if ( (nInput & 0x3) ==1 )
        {
            //Left channel only
            WriteReg( 0x116B, 0x22);

        }
        else if (( nInput & 0x3) == 2)
        {
            // Right channel only
            WriteReg( 0x116B, 0x62);
            DSPEnReg |= MicDSPLRMask;
        }
        InitReg |= (1<<2) ; //stream 2 ;
        ErrNo = ErrNo? ErrNo: WriteReg( 0x117A , DSPEnReg);
        DSPVoiceIn = 2 ; //stream 2 ;
    } 
    else if( nInput & 0x8 )
    {
        InitReg |= (1<<1)  ;//stream 1 ;
        DSPVoiceIn = 1 ;    //stream 1 ;
    }
    else if( nInput & 0x10)
    {
        InitReg |= (4<<2) ; //stream 4 ;
        WriteReg( 0x1170 , 0x15); //set steam 4 input sourc to USB.
        WriteReg( 0x1186 , 0x00); //break the connection between DSP and stream 4.
        DSPVoiceIn = 4 ;    //stream 4 ;
    }
    else if( nInput & 0x20)
    {
        InitReg |= (4<<2) ; //stream 4 ;
        WriteReg( 0x1170 , 0x09); //set steam 4 input sourc to PCM2.
        WriteReg( 0x1186 , 0x00); //break the connection between DSP and stream 4.
        DSPVoiceIn = 4 ;    //stream 4 ;
        SetsPCM2Format(1);
    }

    SetsPCM1Format();

    ErrNo = ErrNo? ErrNo: WriteReg( 0x118C , DSPVoiceIn);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);
    return ErrNo;
}

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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxEnablePCM1Output(int nOutput)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char Mask    = (1<<3) | //stream 3
                            (1<<6) | //stream 6 ;
                            (1<<7) ; //stream 7 ;

    unsigned char Misc2 = ReadReg(0x1176);
    unsigned char DSP16KOutMask = 0x2;

    Misc2 &=~DSP16KOutMask;
    
    InitReg &= ~Mask;
    if( nOutput  )
    {
        InitReg |= (1<<3) ; //stream 3 ;
    }
    if( nOutput & 0x8 )
    {
        WriteReg( 0x117C , 0x01);
        WriteReg( 0x1180 , 0x05); //set Mono output source to OP.
    }
    else
    {
        WriteReg( 0x117C , 0x00);
    }
    if( nOutput & 0x3 )
    {
        InitReg |= (1<<7)  ;//stream 7 ;

        if(( nOutput & 0x3 ) == 1 )
        {   //class d
            WriteReg( 0x1019 , 0x0c); //mono output.
        }
        else if( (nOutput & 0x3) == 2 )
        {   //headphone
            WriteReg( 0x1019 , 0x01);
        }
        else if( (nOutput & 0x3) == 3 )
        {   //headphone + class D
            WriteReg( 0x1019 , 0x0d);
        }
        WriteReg( 0x117f, 0X05);
    }
    if(  nOutput & 0x10 )
    {
        WriteReg( 0x1182 , 0X05);
        Misc2 |= 0x2;  //enable 16 bit output.
        InitReg |= (1<<6) ; //stream 6 ;
        WriteReg( 0x117f, 0X00);
        SetsPCM2Format(1);
    }

    SetsPCM1Format();

    ErrNo = ErrNo? ErrNo: WriteReg( 0x116e , 0x02); //set steam 3 input sourc to PCM 1.
    ErrNo = ErrNo? ErrNo: WriteReg( 0x1176 , Misc2);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);
    if(nOutput == 0) WriteReg( 0x117E , 0XE0);
  //  ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);
    return ErrNo;
}

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
int CxEnablePCM2Input(int nInput)
{
    int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char Mask    = (1<<6) | //stream 6
                            (1<<1) | //stream 1 ;
                            (1<<2) ; //stream 2 ;

    unsigned char DSPEnReg = ReadReg(0x117A);
    unsigned char InCtrlReg ;
    unsigned char MicDSPLRMask    = (1<<6) ; // a switch to switch MIC L/R input when DSP mode is enabled.

    unsigned char DSPVoiceIn = 0;

    InitReg &= ~Mask;

    DSPEnReg &= ~MicDSPLRMask;

    if( nInput  )
    {
        InitReg |= (1<<6) ; //stream 6 ;
        WriteReg( 0x1182, 0x06); // sets the PCM2 source to voice0
    }
    else
    {
        //0 means disalbe.
        WriteReg( 0x1182, 0x00); // no connect
    }

    if( nInput & 0x3 )
    {
        if((nInput & 0x3) == 0x3)
        {
            //both L & R
            WriteReg( 0x116B, 0xa2);

        }
        else if ( (nInput & 0x3) ==1 )
        {
            //Left channel only
            WriteReg( 0x116B, 0x22);

        }
        else if (( nInput & 0x3) == 2)
        {
            // Right channel only
            WriteReg( 0x116B, 0x62);
            DSPEnReg |= MicDSPLRMask;
        }
        InitReg |= (1<<2) ; //stream 2 ;
        ErrNo = ErrNo? ErrNo: WriteReg( 0x117A , DSPEnReg);
        DSPVoiceIn = 2;
    }
    else if( nInput & 0x38 )
    {
        if( nInput & 0x08) 
        {  //Line 1
            InCtrlReg = 0x01;
            if( nInput & 0x80) //test if it is differential.
            {
                InCtrlReg |= 0x08;
            }
        }
        if( nInput & 0x10)
        {  //Line 2
            InCtrlReg = 0x02;
        }
        if( nInput & 0x20)
        { // Line 3
            InCtrlReg = 0x03;
        }
        ErrNo = ErrNo? ErrNo: WriteReg( 0x101A , InCtrlReg);
        InitReg |= (1<<1)  ;//stream 1 ;
        DSPVoiceIn = 1; //redirect voice In to steam 1.
    }
    SetsPCM2Format(0);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x118C , DSPVoiceIn);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);
    return ErrNo;
}

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
int CxEnablePCM2Output(int nOutput)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char Mask    = (1<<4) | //stream 4
                            (1<<7) ; //stream 7 ;
    unsigned char OutputCrtl ;

#ifdef CX2070X_AVOID_USE_STREAM4
    Mask    = (1<<3) | //stream 3
              (1<<7) ; //stream 7 ;
#endif 

    InitReg &= ~Mask;
    if( nOutput  )
    {
#ifdef CX2070X_AVOID_USE_STREAM4
        InitReg |= (1<<3) ; //stream 3 ;
#else
        InitReg |= (1<<4) ; //stream 4 ;
        WriteReg( 0x1170 , 0x09); //set steam 4 input sourc to PCM2.
#endif 
    }
    if( nOutput & 0x8 )
    {
        WriteReg( 0x117C , 0x01);
        WriteReg( 0x1180 , 0x05); //set Mono output source to OP.
    }
    else
    {
        WriteReg( 0x117C , 0x00);
    }
    if( nOutput & 0x7 )
    {
        InitReg |= (1<<7)  ;//stream 7 ;
        OutputCrtl = ReadReg(0x1019);
        OutputCrtl &=~0xAf;
        if(( nOutput == 0x24) )
        {   //Line Out (Differential)
            OutputCrtl |= 0x22;
            WriteReg( 0x1019 , OutputCrtl); //Line out.
        }
        if(( nOutput & 0x87 ) == 0x81 )
        {
            OutputCrtl |= 0x04; //enable class-d output.
            WriteReg( 0x1019 , OutputCrtl) ; 
        }
        else if(( nOutput & 0x7 ) == 1 )
        {   //class d
            OutputCrtl |= 0x04; //enable class-d output.
            OutputCrtl |= 0x08; //mono output.
            WriteReg( 0x1019 , OutputCrtl) ; 
        }
        else if( (nOutput & 0x7) == 2 )
        {   //headphone
            OutputCrtl |= 0x01; //enable headphone output.
            WriteReg( 0x1019 , OutputCrtl) ; 
        }
        else if( (nOutput & 0x7) == 3 )
        {   //headphone + class D
            OutputCrtl |= 0x0d; //mono output.
            WriteReg( 0x1019 , OutputCrtl) ; 
        }
        else if(( nOutput & 0x7 ) == 4 )
        {   //Line Out (single end).
            OutputCrtl |= 0x02;
            WriteReg( 0x1019 , OutputCrtl); //Line out.
        }
    }

    SetsPCM2Format(0);
#ifdef CX2070X_AVOID_USE_STREAM4
    WriteReg( 0x116e , 0x0a); //set steam 3 input sourc to PCM 2.
#endif 
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

	return ErrNo;

}

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
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxEnableAnalogToAnalogLoopback(int nPath)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char Mask    = (1<<1) | //stream 1
                            (1<<2) | //stream 2
                            (1<<7) | //stream 7
                            (1<<8) ; //stream 8

    InitReg &= ~Mask;
    if( nPath & 0x808 )
    {
        WriteReg( 0x117C , 0x01);
    }
    else
    {
        WriteReg( 0x117C , 0x00);
    }
    if( nPath & 0x303 )
    {
        InitReg |= (1<<7)  ;//stream 7 ;

        if( (nPath & 0x300) == 0x100 )
        {   //class d
            WriteReg( 0x1019 , 0x0c); //mono 
        }
        else if( (nPath & 0x300) == 0x200 )
        {   //headphone
            WriteReg( 0x1019 , 0x01);
        }
        else if( (nPath & 0x300) == 0x300 )
        {   //headphone + class D
            WriteReg( 0x1019 , 0x0d);
        }
    }

	if( nPath == 0x8138)
	{
		WriteReg(0x118c,0X2);
		WriteReg(0x1184,0X1);
		WriteReg(0x116A,0X7);
		WriteReg(0x117F,0X1);
	}
	

    if( nPath & 0x30 ) //mic
    {
        InitReg |= (1<<2) ; //stream 2 ;
    }
    if( nPath & 0x80 ) //line in
    {
        InitReg |= (1<<1)  ;//stream 1 ;
    }

    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

	return ErrNo;
}

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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxEnableAGC(int bAGCOn)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char DspReg  =  ReadReg(0x117A);
    unsigned char AGCMask =  0x10;

    DspReg &= ~AGCMask;

    if( bAGCOn)
    {
        DspReg |= AGCMask;
    }

    ErrNo = WriteReg( 0x117a , DspReg);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

	return ErrNo;
}

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
 *    Otherwise, return CX_CX_ERRNO_* error code. 
 */
int CxEnableDRC(int bDRCOn)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char DspReg  =  ReadReg(0x117B);
    unsigned char DRCMask =  0x04;

    DspReg &= ~DRCMask;

    if( bDRCOn)
    {
        DspReg |= DRCMask;
    }

    ErrNo = WriteReg( 0x117B , DspReg);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

	return ErrNo;
}


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
int InitRegisterFromTable(short *pRegTable, int nLen)
{
    int ErrNo = CX_ERRNO_NOERR;
    int i=0;
    short *pReg = pRegTable ;
    short *pRegEnd = pReg + nLen*2;
    short addr, value;
    for( ;pReg != pRegEnd; pReg)
    {
        addr  = *pReg++;
        value = *pReg++;
        ErrNo = WriteReg(addr, value);
        if(ErrNo != 0) break;
    }
    return ErrNo;
}


/*
 * Resotres the AEC Parameters to defautl.
 *
 * PARAMETERS
 *  
 *    None.
                             
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return CX_ERRNO_* error code. 
 *
 * REMARK:
 *    For internal use only, don't call it from outside.
 *    The settings won't take effect until the DSP_INIT has been set.
 */
int CxAECResetToDefault(void)
{
     int ErrNo = CX_ERRNO_NOERR;

	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10ed , 0xff);   // AEC Adaption Speed High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10ee , 0x02);   // AEC ENDLP Gain Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10ef , 0x00);   // AEC ENDLP Gain High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f0 , 0xfe);   // AEC Double Talk Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f1 , 0xff);   // AEC Double Talk High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f2 , 0x00);   // AEC ENDLP Gain Converged Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f3 , 0x00);   // AEC ENDLP Gain Converged High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f4 , 0x00);   // AEC Far-End Voice Low Threshold Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f5 , 0x09);   // AEC Far-End Voice Low Threshold high 
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f6 , 0x01);   // AEC Far-End Voice High Threshold Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f7 , 0x00);   // AEC Far-End Voice High Threshold high
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f8 , 0x00);   // AEC ERLE threshold Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10f9 , 0x20);   // AEC ERLE threshold High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10fa , 0x00);   // AEC Bulk delay Gain Low
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10fb , 0x04);   // AEC Bulk delay Gain High
     ErrNo = ErrNo? ErrNo:WriteReg( 0x10fC , 0x00);   // AEC Bulk delay when LEC is OFF
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x10fd , 0x07);   // AEC Bulk delay when LEC is on

	 // DRC 
     ErrNo = ErrNo? ErrNo:WriteReg( 0x10fe , 0x00);   // AEC DRC ratio low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x10ff , 0x01);   // AEC DRC ratio high
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1100 , 0x00);   // AEC DRC max Ampl Low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1101 , 0x12);   // AEC DRC max ampl high
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1102 , 0x00);   // AEC DRC threshold Low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1103 , 0x12);   // AEC DRC threshold high high
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1104 , 0x04);   // AEC DRC Attack Time Low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1105 , 0x00);   // AEC DRC Attack Time High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x1106 , 0x00);   // AEC DRC Release Time Low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1107 , 0x08);   // AEC DRC Release Time High
	 ErrNo = ErrNo? ErrNo:WriteReg( 0x1108 , 0x00);   // AEC DRC Boost Low
     ErrNo = ErrNo? ErrNo:WriteReg( 0x1109 , 0x00);   // AEC DRC Boost High
    return ErrNo;
}




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
int CxSetAECProfile( int nProfileNum)
{
	int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);
    unsigned char DspReg  =  ReadReg(0x117A);
    unsigned char AECMask =  0x01;

	//restores all AEC settings to default.
	ErrNo = CxAECResetToDefault();
	if(ErrNo) return ErrNo;


	// sets the AEC settings according to Profile Number;
	switch(nProfileNum)
	{
	case 0:
		// Disables AEC.
		break;
	case 1:
		{
			 //WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low
			 //WriteReg( 0x10ed , 0xff);   // AEC Adaption Speed High
			 //WriteReg( 0x10ee , 0x02);   // AEC ENDLP Gain Low

		}
		break;
	case 2:
		{
			//WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low
			//WriteReg( 0x10ed , 0xff);   // AEC Adaption Speed High
			//WriteReg( 0x10ee , 0x02);   // AEC ENDLP Gain Low

		}
		break;
	case 3:
		{
			//WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low

		}
		break;
	case 4:
		{
			//WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low
		}
		break;
	case 5:
		{
			//WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low
		}
		break;
	case 6:
		{
			//WriteReg( 0x10ec , 0xea);   // AEC Adaption Speed Low

		}
		break;
	default:
		ErrNo = CX_ERRNO_NOT_IMPLEMENT;
		return ErrNo;
	}

	// enables or disables the AEC.
    DspReg &= ~AECMask;
    if( nProfileNum )
    {
        DspReg |= AECMask;
    }
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117a , DspReg);
    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

    return ErrNo;
}


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
int CxSetsPCM1Format( int nSampleRate, int nSampleWidth, int nChannelMask)
{
    int ErrNo = CX_ERRNO_NOERR;
    switch(nSampleRate)
    {
    case 8:
    case 16:
    case 24:
    case 32:
    case 44:
    case 48:
        break;
    default:
        return CX_ERRNO_NOT_IMPLEMENT;
    }
    g_cx2070x.desired.oPCM1.nSamplingRate = nSampleRate * 1000;
    g_cx2070x.desired.oPCM1.nSampleWidth = nSampleWidth / 8;
    g_cx2070x.desired.oPCM1.nChannelMask = nChannelMask;
    return ErrNo;
}

/*
 * Commit the format change to PCM1 port
 *
 * PARAMETERS
 *     none
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    This function will be called when calling CxEnablePCMxInput/Output.
 *    You should avoid call this function directly. 
 */
int SetsPCM1Format(void)
{
    int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);

    if(g_cx2070x.current.oPCM1.nSamplingRate || 
        (g_cx2070x.desired.oPCM1.nSamplingRate != g_cx2070x.current.oPCM1.nSamplingRate))
    {
        switch(g_cx2070x.desired.oPCM1.nSamplingRate)
        {
        case 8000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x20);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x20);
            break;
        case 16000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x22);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x22);
            break;
        case 24000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x24);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x24);
            break;
        case 32000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x25);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x25);
            break;
        case 44000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x26);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x26);
            break;
        case 48000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0x116d , 0x27);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1171 , 0x27);
            break;
        default:
            return CX_ERRNO_NOT_IMPLEMENT;
        }
    }

    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);
    if(ErrNo == 0)
    {
        memcpy(&g_cx2070x.current.oPCM1,&g_cx2070x.desired.oPCM1 ,sizeof(g_cx2070x.desired.oPCM1 ));
    }
    return ErrNo;
}

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
 *                              44: 44.1 KHz
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
int CxSetsPCM2Format( int nSampleRate, int nSampleWidth, int nChannelMask, int bSlaveToPCM1)
{
    int ErrNo = CX_ERRNO_NOERR;

    if( bSlaveToPCM1 )
    {
        switch(nSampleRate)
        {
        case 8:
        case 16:
            break;
        default:
            return CX_ERRNO_NOT_IMPLEMENT;
        }
    }
    else
    {
        int streamin   = 0x116F;
        switch(nSampleRate)
        {
        case 8:
        case 16:
        case 24:
        case 32:
        case 44:
        case 48:
            break;
        default:
            return CX_ERRNO_NOT_IMPLEMENT;
        }
    }
    g_cx2070x.desired.oPCM2.nSamplingRate = nSampleRate * 1000;
    g_cx2070x.desired.oPCM2.nSampleWidth = nSampleWidth / 8;
    g_cx2070x.desired.oPCM2.nChannelMask = nChannelMask;
    return ErrNo;
}

/*
 * Commit the format change to PCM1 port
 *
 * PARAMETERS
 *     none
 *
 * RETURN
 *  
 *    If the operation completes successfully, the return value is CX_ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code. 
 * 
 * REMARKS
 *  
 *    This function will be called when calling CxEnablePCMxInput/Output.
 *    You should avoid call this function directly. 
 */
int SetsPCM2Format( int bEnableDDLoopback)
{
    int ErrNo = CX_ERRNO_NOERR;
    unsigned char InitReg = ReadReg(0x117D);

    if( bEnableDDLoopback )
    {
        switch(g_cx2070x.desired.oPCM2.nSamplingRate)
        {
        case 8000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0X116F , 0x2A);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x20);
            break;
        case 16000:
            ErrNo = ErrNo? ErrNo: WriteReg( 0X116F , 0x2B);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x22);
            break;
        default:
            return CX_ERRNO_NOT_IMPLEMENT;
        }
    }
    else
    {
        int streamin   = 0x116F;
        switch(g_cx2070x.desired.oPCM2.nSamplingRate)
        {
        case 8000:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x20);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x20);
            break;
        case 16000:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x22);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x22);
            break;
        case 24000:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x24);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x24);
            break;
        case 32:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x25);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x25);
            break;
        case 44000:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x26);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x26);
            break;
        case 48000:
            ErrNo = ErrNo? ErrNo: WriteReg( streamin , 0x27);
            ErrNo = ErrNo? ErrNo: WriteReg( 0x1172 , 0x27);
            break;
        default:
            return CX_ERRNO_NOT_IMPLEMENT;
        }
    }


    ErrNo = ErrNo? ErrNo: WriteReg( 0x117D , InitReg|01);

    if(ErrNo == 0)
    {
        memcpy(&g_cx2070x.current.oPCM2,&g_cx2070x.desired.oPCM2,sizeof(g_cx2070x.desired.oPCM2));
    }

    return ErrNo;
}

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
int CxSetPCM2Clock(int bEnable, int nClockRate )
{
    int ErrNo = CX_ERRNO_NOERR;
    unsigned short reg = 0xf50;
    unsigned char rate = ReadReg(reg);
    rate &= 0x0F;

    if( bEnable )
    {
        rate |= ((unsigned char )nClockRate&0xf) <<4;
    }
    else
    {
        rate |= (unsigned char ) 0xF0;
    }
    ErrNo = WriteReg(reg,rate);
    msleep(450); //need 350 ms delay.
  
    return ErrNo;
}

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
int CxSetPCM1Clock(int bEnable, int nClockRate )
{
    int ErrNo = CX_ERRNO_NOERR;
    unsigned short reg = 0xf50;
    unsigned char rate = ReadReg(reg);
    rate &= 0xF0;

    if( bEnable )
    {
        rate |= ((unsigned char )nClockRate&0xf);
    }
    else
    {
        rate |= (unsigned char ) 0x0F;
    }
    ErrNo = WriteReg(reg,rate);
    msleep(350); //need 350 ms delay.
    return ErrNo;
}