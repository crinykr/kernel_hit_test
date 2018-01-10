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
**      Cx2070x_init.h
**
**  Abstract:
**      Defining Initial register value for Conexant cx2070x Codec. 
**      
**
**  Product Name:
**      Conexant i2s-based Audio codec.
**
**  Version : 1.0.0.9
**  Remark:
**      This is for internal use, please don't include this file or call the following 
**      functions directly. 
**
**      For wide band phone turning, increases the sampling rate to 16 KHz. 
** 
********************************************************************************
**  Revision History
**      Date        Description                                 Author
**      06/30/11    Created                                     Simon Ho
**      07/13/11    Change the default sampling rate to 16KHz   Simon Ho
**                  Change the PCM2 render path from Stream 4
**                  to Stream 3.
**                  Change DSP settings.
**      08/24/11    Added support for PCM1.                     Simon Ho
**      10/05/11    Added new routing PCM1<=>PCM2               Simon Ho
**      11/07/11    Added support for swtiching between PCM1 
**                  and PCM2 on the fly                         Simon Ho
**      02/02/12    Fixed unaligned memory access problem       Simon Ho
********************************************************************************
*****************************************************************************************/
// Vendor version : 351423.111119
short InitialRegTable[]=
{
	///////////////////////////////
	//Register Address,  Value
	
	// Init PCM 2.
	//0x0F50,     0xff,   // [clock divider] set PCM 2 to slave mode.
	//0x0F5E,     0xB1,   //- Delay option for frame sync: 0 
	//									//- MSB or LSB first :MSB 
	//								   	//- timing option: TX rising send 
	//									//<== 0XB5,   // [PORT2_CONTROL] Delay 1 bit, RX/TX en, mode =PCM , inverted.
	//0x0F5F,     0X03,   // 32 clocks per frame.
	//									// [PORT2_CLOCK_PER_FRAME] 256-bits per frame.
	//0x0F60,     0X00,   //The same value[PORT2_SYNC_WIDTH] Short frame sync
	//0x0F61,     0X01,   //The same value[PORT2_SAMPLE_WIDTH] 16 bits
	//0x0F62,     0X20,   // [PORT2_RX_STREAM1] RX 1 <- Slot 0
	//0x0F63,     0X22,   // [PORT2_RX_STREAM2] RX 2 <- Slot 2
	//0x0F65,     0X20,   // [PORT2_TX_STREAM1] TX 1 -> Slot 0
	//0x0F66,     0X22,   // [PORT2_TX_STREAM2] TX 2 -> Slot 2
    //0x117d,     0x01,

    // Init PCM 1.
    //0x0F51,     0xB5,   // using PCM emulate the I2S.
    //0x0F52,     0x07,   
    //0x0F53,     0x07,
    //0x0F54,     0x1F,
    //0x0F55,     0X1F,
    //0x0F56,     0X05,
    //0x0F57,     0X20,
    //0x0F58,     0X24,
    //0x0F5A,     0X20,
    //0x0F5B,     0X24,

	// Class D
	//0x1011,     0X07,   // [ClassDGain] 2.0 Watt
	
	//0x116F,     0X20, 
	
	//0x1181,     0x00,   // PCM 1 no connect by default.
	//0x1182,     0x00,   // PCM 2 no connect by default.

	//0x1172,	    0x22,   //sets the sampling rate of PCM to 16k
	//0X101A,	    0x09,   //disables Jack sense, and enables the LINE 1 IN diff mode.
	//0x116a,     0x22,   // sets mic sampling rate to 16KHz. Left mono.
	//0x1175,     0x02,   // sets mic sampling rate to 16KHz. Left mono.

	//0x117a,     0x35,   //Enable In-Bound NR, NR, AEC and AGC.

	//0x101e,     0x04,   //sets auto power on, and sets boost to 24 db

	//0X11B1,     0x00,    //sets voice output rate to 16K
	

    //record output gain
    //0x1136,     0x00,      //Record output gain low.
    //0x1137,     0x14,      //Record output gain high.
    0x117d,       0x01,      //Disable all streams by default.

};
/*
   - Delay option for frame sync: 0 or 1 clock : 0 
   - MSB or LSB first :MSB 
   - timing option: TX rising send/RX&FS falling strob or TX  falling send/RX&FS rising strob   : TX RISING 
   - channel numbers per frame: 2 ~32 channels  :16 
   - sync width: 1 ~ 128 clocks  :1 
   - sample size: 8/16 bits  :16 
   - RX/TX start channel NO: 0 ~ 31                                                                
*/
