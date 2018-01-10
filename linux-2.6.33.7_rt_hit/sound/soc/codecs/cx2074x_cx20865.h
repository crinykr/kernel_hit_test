/*
 * ALSA SoC CX2074x codec driver
 *
 * Copyright:   (C) 2010/2011 Conexant Systems
 *
 * Based on sound/soc/codecs/tlv320aic2x.c by Vladimir Barinov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The software is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY, without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along with 
 * this software.  If not, see <http//www.gnu.org/licenses/>
 *
 * All copies of the software must include all unaltered copyright notices, 
 * disclaimers of warranty, all notices that refer to the General Public License 
 * and the absence of any warranty.
 *
 *  History
 *  Added support for CX2074x codec [www.conexant.com]
 */

#ifndef _CX2074x_CX20865_H
#define _CX2074x_CX20865_H

#define CX2074X_I2C_DRIVER_NAME	"cx2074x"
//#define CX2074X_JACK_SENSE_INTERRUPT_MODE

typedef u8 cx2074x_reg_t;

#ifndef _PASS_1_COMPLETE
// DAC 1:2 sample rate/size			0x0F
#define DAC_LEFT_ENABLE				0x01
#define DAC_RIGHT_ENABLE			0x02
#define DAC_RATE_8000				0x00
#define DAC_RATE_11025				0x90
#define DAC_RATE_16000				0x20
#define DAC_RATE_22050				0xb0
#define DAC_RATE_32000				0x40
#define DAC_RATE_44100				0xd0
#define DAC_RATE_48000				0x50
#define DAC_RATE_88200				0xe0
#define DAC_RATE_96000				0x60

// ADC sample rate/size				0x13
#define ADC_ENABLE					0x01
#define ADC_RATE_8000				0x00
#define ADC_RATE_12000				0x10
#define ADC_RATE_16000				0x20
#define ADC_RATE_24000				0x30
#define ADC_RATE_32000				0x40
#define ADC_RATE_48000				0x50
#define ADC_RATE_8018				0x80
#define ADC_RATE_11025				0x90
#define ADC_RATE_16036				0xA0
#define ADC_RATE_22050				0xB0
#define ADC_RATE_44100				0xD0

// ADC analog left control			0x19
#define ADC_LEFT_ENABLE				0x01

// ADC analog right control			0x1A
#define ADC_RIGHT_ENABLE			0x01

// I2S/PCM Control 1				0x20
#define PCM_ENABLE					0x01
#define I2S_PCM_DAC_8_BIT			0x00
#define I2S_PCM_DAC_16_BIT			0x02
#define I2S_PCM_DAC_24_BIT			0x04
#define I2S_PCM_ADC_8_BIT			0x00
#define I2S_PCM_ADC_16_BIT			0x08
#define I2S_PCM_ADC_24_BIT			0x10

// DAC1 Left Channel
#define DAC_LEFT_MUTE				0x80

// DAC2 Right Channel
#define DAC_RIGHT_MUTE				0x80

// Jack Sense Status
#define JACK_SENSE_HEADPHONE		0x01
#define JACK_SENSE_LINEIN_L			0x02
#define	JACK_SENSE_LINEIN_R			0x04
#define	JACK_SENSE_LINEOUT			0x08

// Interrupt Status
#define INT_STS_CLASS_D_ERROR		0x02
#define INT_STS_JACK_SENSE			0x04
#define INT_STS_HEADSET_VOL			0x08
#define INT_STS_ROTARY_VOL			0x10

#endif // #ifndef _PASS_1_COMPLETE

// Don't define bit macros more than once.
#ifndef _PASS_1_COMPLETE
#define _PASS_1_COMPLETE
#endif

#define CX20865_I2C_DRIVER_NAME	"cx20865"
#define AUDDRV_VERSION(major0, major1, minor, build) ((major0) << 24 | (major1) << 16 | (minor) << 8 |(build))

#define COMMAND_OF_SIZE(n)   \
struct {                     \
  int32_t   num_32b_words:16;\
  uint32_t  command_id:15;   \
  uint32_t  reply:1;         \
  uint32_t  app_module_id;   \
  uint32_t  data[n] ;        \
}

// The maximum number of 32-bit data elements that a command can contain
#define MAX_COMMAND_SIZE 13

#define CMD_SET(item)   ((item) & ~0x0100)
#define CMD_GET(item)   ((item) |  0x0100)
#define CMD_MASK        (~(CMD_SET(0)|CMD_GET(0)))
#define CMD_ITEM(cmd)   ((cmd) & CMD_MASK)

#define CMD_REPLY 1
#define APP_ID(a,b,c,d) ((((a)-0x20)<<8)|(((b)-0x20)<<14)|(((c)-0x20)<<20)|(((d)-0x20)<<26))
#define ID(a,b,c,d)  	((((a)-0x20)<<8)|(((b)-0x20)<<14)|(((c)-0x20)<<20)|(((d)-0x20)<<26))

// Retrieve the app and module id from an app_module_id
#define GET_APP_ID(app_module_id)    ((app_module_id)&~0xFF)
#define GET_MODULE_ID(app_module_id) ((app_module_id)& 0xFF)

// Reserved App IDs
#define APP_ID_BROADCAST     0xFFFFFF00 // to broadcast commands to all apps

// Reserved module IDs
#define MODULE_ID_APP        0    // to send commands to the app
#define MODULE_ID_BROADCAST  0xFF // to broadcast commands to all modules

// The Command type may be used to point to commands of arbitrary
// sizes, for example:
// COMMAND_OF_SIZE(5) cmd
// Command *ptr = (Command *)&cmd;
typedef COMMAND_OF_SIZE(MAX_COMMAND_SIZE) Command ;

#if 0
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('C','O','N','F'));// CONF
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('A','S','R','2'));// ASR2
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('A','S','R','4'));// ASR4
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('V','O','I','2'));// VOI2
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('V','O','I','4'));// VOI4
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('D','C','1','6'));// DC16
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('D','C','4','8'));// DC48
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('M','P','1','6'));// MP16
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('M','P','T','S'));// MPTS
SENDCMD(cmd,APP_ID_CTRL,CONTROL_APP_EXEC_FILE,ID('R','6','C','H'));// R6CH
#endif

#define APP_ID_CTRL		APP_ID('C','T','R','L')
#define APP_ID_STRM		APP_ID('S','T','R','M')
#define APP_ID_SOS		APP_ID('S','O','S',' ')
#define APP_ID_COUT		APP_ID('C','O','U','T')
#define APP_ID_MAUI		APP_ID('M','A','U','I')

#define ID_CTRL			ID('C','T','R','L')

#define ID_CONF			ID('C','O','N','F'));// CONF
#define ID_ASR2			ID('A','S','R','2'));// ASR2
#define ID_ASR4			ID('A','S','R','4'));// ASR4
#define ID_VOI2			ID('V','O','I','2'));// VOI2
#define ID_VOI4			ID('V','O','I','4'));// VOI4
#define ID_DC16			ID('D','C','1','6'));// DC16
#define ID_DC48			ID('D','C','4','8'));// DC48
#define ID_MP16			ID('M','P','1','6'));// MP16
#define ID_MPTS			ID('M','P','T','S'));// MPTS
#define ID_R6CH			ID('R','6','C','H'));// R6CH


#define STREAMER_APP_SET_CONFIG_IBIZA 0x0030
#define IBIZA_ADC0_BOOST 14
#define IBIZA_ADC1_BOOST 15

typedef enum {

  CONTROL_APP_CMD_RESERVED      =  0,
//  CONTROL_APP_SUSPEND         =  1,
//  CONTROL_APP_RESUME          =  2,
  CONTROL_APP_VERSION           =  3,
  CONTROL_APP_EXEC_FILE         =  4,
  CONTROL_APP_OCLA_ENABLE       =  5,
  CONTROL_APP_MEMORY            =  6,
  CONTROL_APP_CLOCK	        =  7,

  CONTROL_APP_I2C_TUNNEL_CONFIG =  8,
  CONTROL_APP_I2C_TUNNEL_DATA   =  9,
  CONTROL_APP_I2C_TUNNEL_APPLY  = 10,

  CONTROL_APP_LOGGING_INIT      = 20,
  CONTROL_APP_LOGGING_ENABLE    = 21,

  SOS_TASK_CREATE               = 35,
  SOS_TASK_PRIORITY             = 36,
  SOS_EXEC_DUP                  = 37,
  SOS_EXEC_FREE                 = 38,
  SOS_EXEC_PARM                 = 39,
  SOS_JIFFIES                   = 41,
  SOS_SIGNAL                    = 42,
  SOS_SIGNAL_ALL                = 43,
  SOS_ENABLE_IRQ_NR             = 44,
  SOS_DISABLE_IRQ_NR            = 45,
  SOS_RESOURCE                  = 47,
  SOS_TIME                      = 50,
  SOS_TASK_SLEEP_JIFFIES        = 51,

  CONTROL_APP_USB_START         = 60,
  CONTROL_APP_USB_STOP	        = 61,

} ControlAppCommandCode;

#define CONTROL_APP_GET_VERSION           CMD_GET(CONTROL_APP_VERSION)
#define CONTROL_APP_GET_MEMORY            CMD_GET(CONTROL_APP_MEMORY )
#define CONTROL_APP_SET_MEMORY            CMD_SET(CONTROL_APP_MEMORY )
#define CONTROL_APP_GET_I2C_TUNNEL_CONFIG CMD_GET(CONTROL_APP_I2C_TUNNEL_CONFIG)
#define CONTROL_APP_SET_I2C_TUNNEL_CONFIG CMD_SET(CONTROL_APP_I2C_TUNNEL_CONFIG)
#define CONTROL_APP_GET_I2C_TUNNEL_DATA   CMD_GET(CONTROL_APP_I2C_TUNNEL_DATA)
#define CONTROL_APP_SET_I2C_TUNNEL_DATA   CMD_SET(CONTROL_APP_I2C_TUNNEL_DATA)

// Data structures used by the control app's commands
typedef enum {
  MEM_TYPE_X = 0,
  MEM_TYPE_Y = 1,
} MemType;

extern  struct snd_soc_dai soc_codec_cx2074x_cx20865_dai;
extern  struct snd_soc_codec_device soc_codec_dev_cx2074x_cx20865;

struct cx2074x_data {
	int dres;
};

#endif // _CX2074x_H
