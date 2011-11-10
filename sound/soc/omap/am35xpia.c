/*
 * am35xpia.c  -- ALSA SoC support for piAx-AM3517
 * by pironex GmbH -- http://www.pironex.de
 *
 * Copyright (C) 2011 pironex GmbH <info@pironex.de>
 * Author: Bjoern Krombholz <b.krombholz@pironex.de>
 *
 * Based on ALSA SoC for AM3517 EVM by Anuj Aggarwal <anuj.aggarwal@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic32x4.h"

#define CODEC_CLOCK 	12000000

//static struct clk *sys_clkout1;

static int am35xpia_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned int fmt;
	u8 reg;
#if 1
	//fmt = (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS; // omap master

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
//	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
//			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	/* use internal McBSP CLKS 96MHz */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
			96000000, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_CLKR_SRC_CLKX\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 125);
	if (ret < 0) {
		printk(KERN_ERR "can't set SRG clock divider\n");
		return ret;
	}
#else
	fmt = (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
//	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
//			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

//	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT, 0,
//				SND_SOC_CLOCK_IN);
//	if (ret < 0) {
//		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_CLKR_SRC_CLKX\n");
//		return ret;
//	}
//	snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, 0,
//			SND_SOC_CLOCK_IN);
//	if (ret < 0) {
//		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_FSR_SRC_FSX\n");
//		return ret;
//	}
	/* use internal McBSP CLKS 96MHz */
//	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
//			96000000, SND_SOC_CLOCK_IN);
//	if (ret < 0) {
//		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_SYSCLK_CLKS_FCLK\n");
//		return ret;
//	}

	reg = snd_soc_read(codec_dai->codec, 27);
	pr_info("aic3204: reg27: %x", reg);
	reg = snd_soc_read(codec_dai->codec, 28);
	pr_info("aic3204: reg28: %x", reg);
	reg = snd_soc_read(codec_dai->codec, 29);
	pr_info("aic3204: reg29: %x", reg);
	reg = snd_soc_read(codec_dai->codec, 31);
	pr_info("aic3204: reg31: %x", reg);
	reg = snd_soc_read(codec_dai->codec, 32);
	pr_info("aic3204: reg32: %x", reg);
	reg = snd_soc_read(codec_dai->codec, 33);
	pr_info("aic3204: reg33: %x", reg);


#endif

	return 0;
}

static struct snd_soc_ops am35xpia_ops = {
	.hw_params = am35xpia_hw_params,
};

/* am3517evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic32x4_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	/*SND_SOC_DAPM_MIC("DMic", NULL),*/
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},

	{"Ext Spk", NULL, "LLOUT"},
	{"Ext Spk", NULL, "RLOUT"},

	/*{"DMic Rate 64", NULL, "Mic Bias"},*/
	/*{"Mic Bias 2V", NULL, "DMic"},*/
};

static int am35xpia_aic32x4_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;

	/* not connected or unused */
	snd_soc_dapm_nc_pin(codec, "IN1_L"); /* IN1_L */
	snd_soc_dapm_nc_pin(codec, "IN1_R"); /* IN1_R */
	snd_soc_dapm_nc_pin(codec, "IN2_L"); /* IN2_L */
	snd_soc_dapm_nc_pin(codec, "IN2_R"); /* IN2_R */
	snd_soc_dapm_nc_pin(codec, "IN3_L");  /* IN3_L */
	//snd_soc_dapm_nc_pin(codec, "LOL");  /* just tp */
	//snd_soc_dapm_nc_pin(codec, "LOR");  /* just tp */

	/* Add piAx AM3517 specific widgets */
//	snd_soc_dapm_new_controls(codec, aic32x4_dapm_widgets,
//				  ARRAY_SIZE(aic32x4_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
//	snd_soc_dapm_enable_pin(codec, "HPL");
//	snd_soc_dapm_enable_pin(codec, "HPR");
//	snd_soc_dapm_enable_pin(codec, "LOL");
//	snd_soc_dapm_enable_pin(codec, "LOR");
	//snd_soc_dapm_enable_pin(codec, "Mic In");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link am35xpia_dai = {
	.name = "TLV320AIC32X4",
	.stream_name = "AIC32X4",
	.cpu_dai_name ="omap-mcbsp-dai.1",
	.codec_dai_name = "tlv320aic32x4-hifi",
	.platform_name = "omap-pcm-audio",
	.codec_name = "tlv320aic32x4.2-0018",
	.init = am35xpia_aic32x4_init,
	.ops = &am35xpia_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_am35xpia = {
	.name = "am35xpia",
	.dai_link = &am35xpia_dai,
	.num_links = 1,
};

static struct platform_device *am35xpia_snd_device;

static int __init am35xpia_soc_init(void)
{
	int ret;
//	struct device *dev;

	if (!machine_is_pia_am35x())
		return -ENODEV;

	pr_info("piA AM3517 SoC init\n");

	am35xpia_snd_device = platform_device_alloc("soc-audio", -1);
	if (!am35xpia_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(am35xpia_snd_device, &snd_soc_am35xpia);

	ret = platform_device_add(am35xpia_snd_device);
	if (ret)
		goto err1;

//	dev = &am35xpia_snd_device->dev;
//	pr_info("piA AM3517 SoC: Initializing SYS_CLKOUT1");
//	sys_clkout1 = clk_get(dev, "sys_clkout1");
//	if (IS_ERR(sys_clkout1)) {
//		pr_err("pia35x: Could not get sys_clkout1");
//		return -2;
//	}
//
//	pr_info("pia35x: CLK - enabling SYS_CLKOUT1 with %lu MHz",
//			clk_get_rate(sys_clkout1));
//	clk_enable(sys_clkout1);


	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(am35xpia_snd_device);

	return ret;
}

static void __exit am35xpia_soc_exit(void)
{
	platform_device_unregister(am35xpia_snd_device);
}

module_init(am35xpia_soc_init);
module_exit(am35xpia_soc_exit);

MODULE_AUTHOR("Bjoern Krombholz <b.krombholz@pironex.de>");
MODULE_DESCRIPTION("ALSA SoC piAx-AM3517");
MODULE_LICENSE("GPL v2");
