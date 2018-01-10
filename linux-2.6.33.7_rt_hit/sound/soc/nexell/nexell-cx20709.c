/*
 * (C) Copyright 2012
 * Dong-gweon Oh, Flowdas Inc.,, <prospero@flowdas.com>
 *
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include <mach/platform.h>

#include "nexell-pcm.h"
#include "nexell-i2s.h"
#include "../codecs/cx20709.h"

#define MOST2120_CX20709_DEBUG 1

#if (1)
#define DBGOUT(msg...)      { printk(KERN_INFO "codec: " msg); }
#else
#define DBGOUT(msg...)      do {} while (0)
#endif

/*--------------------------------------------------------------------------------
 * sound soc ops
 */

#define CX20709_POBCTRL   0x80
#define CX20709_BUFDCOPEN 0x10
#define CX20709_BUFIOEN   0x08
#define CX20709_SOFT_ST   0x04
#define CX20709_HPSTBY    0x01

static int most2120_cx20709_init(struct snd_soc_codec *codec)
{
    u16 iface;
    DBGOUT("%s\n", __func__);

    /* Startup codec would need initialization...? */

//    /* Enable anti-pop features */
//    snd_soc_write(codec, CX20709_APOP1,
//                     CX20709_POBCTRL | CX20709_SOFT_ST |
//                     CX20709_BUFDCOPEN | CX20709_BUFIOEN);
//    /* Now we should enable CX20709 and set CX20709 as correct status as we wanted. */
//    
//    snd_soc_write(codec, CX20709_POWER1, 0x1F2); //0x1F2
//    /* Speaker Power Enable */
//    snd_soc_write(codec, CX20709_POWER2, 0x1FF);
//#if 1
//    snd_soc_write(codec, CX20709_IFACE2, 0x040);  /* ADCLRC is not GPIO1 */
//    snd_soc_write(codec, CX20709_ADDCTL2, 0x024); /* LRCM On/ADCRC input/HPSWEN off/HPSWPOL=1 */
//    /* Set GPIO to GPIO JD */
//    snd_soc_write(codec, CX20709_ADDCTL4, 0x000); /* GPIO1 is JD */
//    /* Set LINVOL/RINVOL to set 0dB */
//    snd_soc_write(codec, CX20709_LINVOL, 0x140 | 27); /* Set to 0dB */
//    snd_soc_write(codec, CX20709_RINVOL, 0x140 | 27); /* Set to 0dB */
//    /* Set ADCVOL to 0dB */
//    snd_soc_write(codec, CX20709_LADC, 0x100 | 195); /* Set to 0dB */
//    snd_soc_write(codec, CX20709_RADC, 0x100 | 195); /* Set to 0dB */
//    /* Set MICBOOST to 20dB/no Mute */
//    snd_soc_write(codec, CX20709_LINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
//    snd_soc_write(codec, CX20709_RINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
//    /* Set LOUT1/ROUT1 volume to set 0dB */
//    snd_soc_write(codec, CX20709_LOUT1, 0x1F9);
//    snd_soc_write(codec, CX20709_ROUT1, 0x1F9);
//    /* Set output mixer DAC mute to off */
//    snd_soc_write(codec, CX20709_LOUTMIX, 0x100);
//    snd_soc_write(codec, 0x23, 0x100);
//    snd_soc_write(codec, 0x24, 0x100);
//    snd_soc_write(codec, CX20709_ROUTMIX, 0x100);
//    /* Set LOUT2/ROUT2 volume to set 0dB */
//    snd_soc_write(codec, CX20709_LOUT2, 0x1F9);
//    snd_soc_write(codec, CX20709_ROUT2, 0x1F9);
//    /* Power on Mixer */
//    snd_soc_write(codec, CX20709_POWER3, 0x03C); // 0x03C for L,R micamp on
//#endif
//#if 1
//    iface = snd_soc_read(codec, CX20709_IFACE1);
//    snd_soc_write(codec, CX20709_IFACE1, iface | 0x100); // ADC L,R channel would be reversed
//#endif
//    /* Speaker DC/AC gain to 1.5 (3.3V->5V) translate... */
//    snd_soc_write(codec, CX20709_CLASSD3, 0x1B); /* DC gain to 1.52, AC gain to 1.52 */
//    /* Speaker Output Driver enable */
//    snd_soc_write(codec, CX20709_CLASSD1, 0xC0); /* Disables all speaker*/

    return 0;
}

static int most2120_codec_startup(struct snd_pcm_substream *substream)
{
    DBGOUT("%s\n", __func__);
    
    return 0;
}

static void most2120_codec_shutdown(struct snd_pcm_substream *substream)
{
    DBGOUT("%s\n", __func__);
}

extern int cpu_get_clock_hz(int clk); /* From Nexell's core freq module */

static int most2120_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
    unsigned int pll_out = 0, ddiv = 0, adiv = 0, bdiv;
    int ret = 0;
    int iis_clk;
#if MOST2120_CX20709_DEBUG
    int i;
#endif

    DBGOUT("Entered %s\n", __func__);

    iis_clk = cpu_get_clock_hz(1); /* Get PLL1 freq */
    
    //printk("iis_clk = %d params_rate = %d\n", iis_clk, params_rate(params));
//    switch (params_rate(params)) {
//      case 8000:
//        ddiv = CX20709_DAC_DIV_6;
//        adiv = CX20709_ADC_DIV_6;
//        bdiv = CX20709_BCLK_DIV_24;
//        pll_out = 12288000;
//        break;
//      case 16000:
//        ddiv = CX20709_DAC_DIV_3;
//        adiv = CX20709_ADC_DIV_3;
//        bdiv = CX20709_BCLK_DIV_12;
//        pll_out = 12288000;
//        break;
//      case 32000:
//        ddiv = CX20709_DAC_DIV_1_5;
//        adiv = CX20709_ADC_DIV_1_5;
//        bdiv = CX20709_BCLK_DIV_6;
//        pll_out = 12288000;
//        break;
//      case 48000:
//        ddiv = CX20709_DAC_DIV_1;
//        adiv = CX20709_ADC_DIV_1;
//        bdiv = CX20709_BCLK_DIV_4;
//        pll_out = 12288000;
//        break;
//      case 11025:
//        ddiv = CX20709_DAC_DIV_4;
//        adiv = CX20709_ADC_DIV_4;
//        bdiv = CX20709_BCLK_DIV_16;
//        pll_out = 11289600;
//        break;
//      case 22050:
//        ddiv = CX20709_DAC_DIV_2;
//        adiv = CX20709_ADC_DIV_2;
//        bdiv = CX20709_BCLK_DIV_8;
//        pll_out = 11289600;
//        break;
//      case 44100:
//        ddiv = CX20709_DAC_DIV_1;
//        adiv = CX20709_ADC_DIV_1;
//        bdiv = CX20709_BCLK_DIV_4;
//        pll_out = 11289600;
//        break;
//      default :
//        ddiv = CX20709_DAC_DIV_1;
//        adiv = CX20709_ADC_DIV_1;
//        bdiv = CX20709_BCLK_DIV_4;
//        pll_out = 11289600;
//    }
//
//    /* NEXELL's nxp2120 has limited clock generation function, so it would be better
//       Nxp2120 only generate reference clock from PLL1, 192MHz / 16 = 12 MHz...
//       And then let CX20709 generate required clock for itself. */
//
//    /* set codec DAI configuration */
//    ret = snd_soc_dai_set_fmt(codec_dai,
//        SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
//        SND_SOC_DAIFMT_CBM_CFM); /* Set master mode for generate bitclk and fs(lrclk) */
//    if (ret < 0)
//        return ret;

    /* set cpu DAI configuration */
    ret = snd_soc_dai_set_fmt(cpu_dai,
        SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
        SND_SOC_DAIFMT_CBS_CFS); /* Set slave mode for bitclk, and fs(lrclk) */
    if (ret < 0)
        return ret;

    /* set CPU generate reference clock for Codec's PLL. So Set PLL1 as clock source.  */
    ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_0, NX_I2S_CLKSRC0_PLL1, SND_SOC_CLOCK_OUT);
    if (ret < 0)
        return ret;
    
    /* Our I2S is Slave, So Set to EXT_BIT.  */
    ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_1, NX_I2S_CLKSRC1_EXT_BIT, SND_SOC_CLOCK_IN);
    if (ret < 0)
        return ret;
    
    /* Set prescaler set to 16 to generate 12MHz around clock */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_0, 16 - 1); /* PLL1 192MHz -> 12MHz with 1/16 */
    if (ret < 0)
        return ret;
    
    /* Use Bitclk as Clock Source and no division. */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_1, 1 - 1); /* No div as BITCLK */
    if (ret < 0)
        return ret;

    /* Set fs as 64fs(256fs). */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_SYNC_PERIOD, NX_I2S_PERDIV_64);
    if (ret < 0)
        return ret;
   
//    /* set codec sysclock from PLL source */
//    ret = snd_soc_dai_set_clkdiv(codec_dai, CX20709_SYSCLKSEL, CX20709_SYSCLK_PLL);
//    if (ret < 0)
//        return ret;
//
//    /* Now All the bit rate and rootclock should be set. */
//    /* set codec sysclock from PLL source */
//    ret = snd_soc_dai_set_pll(codec_dai, 0, 0, iis_clk / 16, pll_out);
//    if (ret < 0)
//        return ret;
//   
//    /* set codec speaker DCLK for SYSCLK/2. -> SYSCLK/16 */
//    ret = snd_soc_dai_set_clkdiv(codec_dai, CX20709_DCLKDIV, CX20709_DCLK_DIV_16);
//    if (ret < 0)
//        return ret;
//
//    /* set codec DAC div factor to ddiv. */
//    ret = snd_soc_dai_set_clkdiv(codec_dai, CX20709_DACDIV, ddiv);
//    if (ret < 0)
//        return ret;
//    
//    /* set codec ADC div factor to adiv. */
//    ret = snd_soc_dai_set_clkdiv(codec_dai, CX20709_ADCDIV, adiv);
//    if (ret < 0)
//        return ret;
//    
//    /* Bit Clock should be devided for NEXELL 256fs machine  */
//    ret = snd_soc_dai_set_clkdiv(codec_dai, CX20709_BCLKDIV, bdiv);
//    if (ret < 0)
//        return ret;
//
//    
//#if MOST2120_CX20709_DEBUG
//    for (i=0; i<56;i++) {
//        printk("codec addr = 0x%x(%d) val= %x\n", i, i, snd_soc_read(codec_dai->codec, i));
//    }
//#endif
    return 0;
}

static int (*cpu_dai_resume)(struct snd_soc_dai *dai) = NULL;

static int most2120_codec_resume_pre(struct platform_device *pdev)
{
    struct snd_soc_device *socdev  = platform_get_drvdata(pdev);
    struct snd_soc_card   *card    = socdev->card;
    struct snd_soc_dai    *cpu_dai = card->dai_link->cpu_dai;
    int ret = 0;

    PM_DBGOUT("+%s\n", __func__);

    /*
     * first execute cpu(i2s) resume and execute codec resume.
     */
    if (cpu_dai->resume && ! cpu_dai_resume) {
        cpu_dai_resume  = cpu_dai->resume;
        cpu_dai->resume = NULL;
    }

    if (cpu_dai_resume)
        ret = cpu_dai_resume(cpu_dai);

    PM_DBGOUT("-%s\n", __func__);
    return ret;
}

/*--------------------------------------------------------------------------------
 * sound soc struct
 */
static struct snd_soc_ops most2120_codec_ops = {
    .startup    = most2120_codec_startup,
    .shutdown   = most2120_codec_shutdown,
    .hw_params  = most2120_codec_hw_params,
};

static struct snd_soc_dai_link most2120_dai_link = {
    .name           = "most-cx20709",
    .stream_name    = "most-cx20709 HiFi",
    .codec_dai      = &cx20709_dai,          /* codecs cx20709 */
    .cpu_dai        = &nx_snd_i2s_dai,      /* nexell-i2s */
    .init           = most2120_cx20709_init,
    .ops            = &most2120_codec_ops,
};

static struct snd_soc_card most2120_cx20709_card = {
    .name       = "most-cx20709",
    .platform   = &nx_snd_pcm_platform,         /* nexell-pcm */
    .dai_link   = &most2120_dai_link,
    .num_links  = 1,
    .resume_pre = &most2120_codec_resume_pre,
};

static struct cx20709_data cx20709_codec_data = {
    .dres = 0,
};

static struct snd_soc_device cx20709_soc_device = {
    .card       = &most2120_cx20709_card,
    .codec_dev  = &soc_codec_dev_cx20709,        /* codecs/cx20709 */
    .codec_data = &cx20709_codec_data,
};

/*--------------------------------------------------------------------------------
 * sound pcm platform
 ---------------------------------------------------------------------------------*/
static struct platform_device * cx20709_plat_device;

static int __init most2120_init(void)
{
    struct snd_soc_device *socdev  = &cx20709_soc_device;
    int ret = 0;

    DBGOUT("%s\n", __func__);

    cx20709_plat_device = platform_device_alloc("soc-audio", -1);
    if (! cx20709_plat_device) {
        printk(KERN_ERR "%s: fail platform_device_alloc for codec ...\n", __func__);
        return -ENOMEM;
    }

    platform_set_drvdata(cx20709_plat_device, socdev);
    socdev->dev = &cx20709_plat_device->dev;

    ret = platform_device_add(cx20709_plat_device);
    if (ret) {
        platform_device_put(cx20709_plat_device);
        return ret;
    }

    return ret;
}

static void __exit most2120_exit(void)
{
    DBGOUT("%s\n", __func__);
    platform_device_unregister(cx20709_plat_device);
}

module_init(most2120_init);
module_exit(most2120_exit);

MODULE_AUTHOR("Dong-gweon Oh <prospero@flowdas.com>");
MODULE_DESCRIPTION("ALSA SoC CX20709 MOST2120");
MODULE_LICENSE("GPL");
