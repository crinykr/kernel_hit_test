/*
 * ALSA SoC CX20865 codec driver
 *
 * Copyright:   (C) 2013 Conexant Systems
 *
 * Based on wm9081 by Mark Brown
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _CX20865_H
#define _CX20865_H


//#define GPIO_HP_JACKSENSE 178 //Tegra 250
//.#define JACK_SENSE_GPIO_PIN    178 // Tegra
//#define CODEC_RESET_GPIO_PIN   184 //  Tegra

#define JACK_SENSE_GPIO_PIN   151 //s5pc110 GPH2_5
#define CODEC_RESET_GPIO_PIN  157 //s5pc110 reset pin.

#if (defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE) ) && !defined(CONFIG_CNXT_USING_SPI_BUS)
#define USING_I2C 1
#endif 

#if defined(CONFIG_SPI_MASTER) && defined(CONFIG_CNXT_USING_SPI_BUS)
#define USING_SPI 1
#endif 


enum Cx_INPUT_SEL{
    Cx_INPUT_SEL_BY_GPIO = 0,
    Cx_INPUT_SEL_MIC,
    Cx_INPUT_SEL_LINE,
    Cx_INPUT_SEL_DPORT2,
};

enum Cx_OUTPUT_SEL{
    Cx_OUTPUT_SEL_BY_GPIO = 0,
    Cx_OUTPUT_SEL_SPK,
    Cx_OUTPUT_SEL_LINE,
    Cx_OUTPUT_SEL_HP,
    Cx_OUTPUT_SEL_DPORT2,
};

#define CX20865_I2C_DRIVER_NAME	"cx20865"
#define AUDDRV_VERSION(major0,major1, minor, build ) ((major0)<<24|(major1)<<16| (minor)<<8 |(build))

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

extern struct snd_soc_dai cx20865_dai;
extern struct snd_soc_codec_device soc_codec_dev_cx20865;

struct cx20865_setup_data
{
    unsigned short i2c_address;
    unsigned short reg_addr_len; 
    unsigned short gpio_reset_pin;
};


#endif // _CX20865_H
