/*
 * Allwinner sun8i I2S sound card for NanoPi NEO/NEO2 series
 *
 * Copyright (C) 2016 Jean-Francois Moine <moinejf at free.fr>
 * Copyright (C) 2017 Anthony Lee <don.anthony.lee at gmail.com>
 * Copyright (C) 2017-2022 __tkz__ <tkz at lrclk.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>

#define MAX_CODECS_NUM (4)

/* --- hardware --- */

#define I2S_CTL 	  	0x00
	#define I2S_CTL_SDOEN_MSK	(0x0f00)
	#define I2S_CTL_SDO3EN		BIT(11)
	#define I2S_CTL_SDO2EN		BIT(10)
	#define I2S_CTL_SDO1EN		BIT(9)
	#define I2S_CTL_SDO0EN		BIT(8)
	#define I2S_CTL_OUTMUTE		BIT(6)
	#define I2S_CTL_TXEN		BIT(2)
	#define I2S_CTL_RXEN		BIT(1)
	#define I2S_CTL_GEN		BIT(0)
	#define I2S_CTL_H3_BCLKOUT	BIT(18)
	#define I2S_CTL_H3_LRCKOUT	BIT(17)
	#define I2S_CTL_H3_MODE_MSK	(3 << 4)
	#define I2S_CTL_H3_MODE_I2S	(1 << 4)
	#define I2S_CTL_H3_MODE_RGT	(2 << 4)

#define I2S_FAT0 		0x04
	#define I2S_FAT0_H3_LRCKR_PERIOD(v) ((v) << 20)
	#define I2S_FAT0_H3_LRCKR_PERIOD_MSK (0x3ff << 20)
	#define I2S_FAT0_H3_LRCK_POLARITY	BIT(19)
	#define I2S_FAT0_H3_LRCK_PERIOD(v)	((v) << 8)
	#define I2S_FAT0_H3_LRCK_PERIOD_MSK (0x3ff << 8)
	#define I2S_FAT0_H3_BCLK_POLARITY	BIT(7)
	#define I2S_FAT0_H3_SR_16		(3 << 4)
	#define I2S_FAT0_H3_SR_24		(5 << 4)
	#define I2S_FAT0_H3_SR(v)		(((v>>2)-1) << 4)
	#define I2S_FAT0_H3_SR_MSK		(7 << 4)
	#define I2S_FAT0_H3_SW_16		(3 << 0)
	#define I2S_FAT0_H3_SW_32		(7 << 0)
	#define I2S_FAT0_H3_SW(v)		(((v>>2)-1) << 0)
	#define I2S_FAT0_H3_SW_MSK		(7 << 0)

#define I2S_FAT1		0x08

#define I2S_FCTL		0x14
	#define I2S_FCTL_FTX		BIT(25)
	#define I2S_FCTL_FRX		BIT(24)
	#define I2S_FCTL_TXTL(v)	((v) << 12)
	#define I2S_FCTL_TXIM		BIT(2)

#define I2S_INT    		0x1c
	#define I2S_INT_TXDRQEN		BIT(7)

#define I2S_RXFIFO		0x10
#define I2S_TXFIFO		0x20
#define I2S_FSTA		0x18
	
#define I2S_CLKD   		0x24
	/* common */
	#define I2S_CLKD_BCLKDIV(v)	((v) << 4)
	#define I2S_CLKD_MCLKDIV(v)	((v) << 0)
	#define I2S_CLKD_H3_MCLKOEN	BIT(8)

#define I2S_TXCNT  		0x28

#define I2S_RXCNT  		0x2c


/* --- H3 --- */
#define I2S_TXCHCFG_H3		0x30
	#define I2S_TXCHCFG_H3_TX_SLOT_NUM_MSK (7 << 0)
	#define I2S_TXCHCFG_H3_TX_SLOT_NUM(v) ((v) << 0)

#define I2S_TX0CHSEL_H3		0x34		/* 0..3 */
	#define I2S_TXn_H3_OFFSET_MSK	(3 << 12)
	#define I2S_TXn_H3_OFFSET(v)	((v) << 12)
	#define I2S_TXn_H3_CHEN_MSK	(0xff << 4)
	#define I2S_TXn_H3_CHEN(v)	((v) << 4)
	#define I2S_TXn_H3_CHSEL_MSK	(7 << 0)
	#define I2S_TXn_H3_CHSEL(v)	((v) << 0)

#define I2S_TX0CHMAP_H3		0x44		/* 0..3 */

#define I2S_RXCHSEL_H3		0x54
#define I2S_RXCHMAP_H3		0x58

/* --- driver --- */

#define I2S_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | \
	 SNDRV_PCM_FMTBIT_S20_3LE | \
	 SNDRV_PCM_FMTBIT_S24_LE | \
	 SNDRV_PCM_FMTBIT_S32_LE)

#define PCM_LRCK_PERIOD 32
#define PCM_LRCKR_PERIOD 1

#define PCM_LRCK_PERIOD_MAP_SHIFT 1 // period resolution 2bit
#define PCM_LRCK_PERIOD_MAP_RESOLUTION (1 << PCM_LRCK_PERIOD_MAP_SHIFT)
#define PCM_LRCK_PERIOD_MAP_MIN        PCM_LRCK_PERIOD_MAP_RESOLUTION
#define PERIOD_TO_MAP(period)          (1 << (((unsigned int)period) >> PCM_LRCK_PERIOD_MAP_SHIFT))

struct priv {
	struct device *dev;
	struct clk *mod_clk;
	struct clk *bus_clk;
	struct regmap *regmap;
	struct reset_control *rstc;

	int type;
	int nchan;

	struct snd_dmaengine_dai_dma_data playback_dma_data;

	int clk_always_on;
	
	int fs;
	int div;

	unsigned int lrclk_period_map;
	unsigned int mclk_mode;
	unsigned int mclk_max_freq;
	unsigned int dai_fmt;
	unsigned int rj_slotwidth;
};

static const struct of_device_id sun8i_i2s_of_match[] = {
	{ .compatible = "allwinner,sun8i-h3-i2s_kai" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun8i_i2s_of_match);

/* --- CPU DAI --- */

static void sun8i_i2s_init(struct priv *priv)
{
	/* disable global */
	regmap_update_bits(priv->regmap, I2S_CTL,
			   I2S_CTL_GEN | I2S_CTL_RXEN | I2S_CTL_TXEN,
			   0);

	priv->nchan = 2;

	regmap_update_bits(priv->regmap, I2S_FCTL,
				I2S_FCTL_FRX | I2S_FCTL_FTX,	/* clear the FIFOs */
				0);

	regmap_write(priv->regmap, I2S_TXCNT, 0); /* FIFO counters */
	regmap_write(priv->regmap, I2S_RXCNT, 0);

	regmap_update_bits(priv->regmap, I2S_CTL,
				I2S_CTL_H3_LRCKOUT | I2S_CTL_H3_BCLKOUT,
				I2S_CTL_H3_LRCKOUT | I2S_CTL_H3_BCLKOUT);

	regmap_update_bits(priv->regmap, I2S_CTL,
				I2S_CTL_H3_MODE_MSK,
				I2S_CTL_H3_MODE_I2S);

	regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
				I2S_TXn_H3_OFFSET_MSK,
				I2S_TXn_H3_OFFSET(1));

	regmap_update_bits(priv->regmap, I2S_FAT0,
				I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY, /* normal bclk & frame */
				0);
}

static int sun8i_i2s_lrclk_period_minimum_match(struct priv *priv, int sample_resolution)
{
	int min_match  = sample_resolution;
	int max_period = PCM_LRCK_PERIOD;

	if (priv->fs) {
		return priv->fs >> 1;
	}

	// check max_period
	while ( (max_period > PCM_LRCK_PERIOD_MAP_RESOLUTION) && !(priv->lrclk_period_map & PERIOD_TO_MAP(max_period)) )
		max_period -= PCM_LRCK_PERIOD_MAP_RESOLUTION ;
	if ( sample_resolution > max_period ) {
		dev_dbg(priv->dev, "%s: sample resolution round down %dbit to %dbit", __func__, sample_resolution, max_period);
		return max_period;
	}

	// scan matched fs
	while((min_match < PCM_LRCK_PERIOD) && !(priv->lrclk_period_map & PERIOD_TO_MAP(min_match))) 
	{
		min_match += PCM_LRCK_PERIOD_MAP_RESOLUTION;
	}

	return min_match;
}

static int sun8i_i2s_set_clock(struct priv *priv, unsigned long rate, int sample_resolution)
{
	unsigned long freq;
	int ret, i, div, period;
	static const u8 div_tb[] = {
		1, 2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 176, 192
	};

	/* calculate period */
	period = sun8i_i2s_lrclk_period_minimum_match(priv, sample_resolution);
	dev_dbg(priv->dev, "%s: rate = %lu. sample_resolution = %d period = %d", __func__, rate, sample_resolution, period);

	if ( (priv->dai_fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBS_CFS) {
		/* compute the sys clock rate and divide values */
		if (priv->div) {
			div = priv->div;
			freq = period * 2 * rate * div;
		}else
		if (priv->mclk_max_freq) {
			// search mclk less than mclk_max_freq
			for (i = 0; i < ARRAY_SIZE(div_tb) - 1; i++) {
				if ((period * rate * 2 * div_tb[i]) > priv->mclk_max_freq ) {
					break;
				}
			}
			// error check
			if ( i == 0 ) {
				pr_info("Setting sysclk rate %lu is not supported. (search mclk)\n", rate );
				return -EINVAL;
			}
			// calcurate mclk & bclk div
			freq = period * rate * 2 * div_tb[i-1];
			div  = div_tb[i-1];

		}else{
			if ((rate % 11025) == 0)
				freq	= 22579200;
			else if ((rate % 8000) == 0)
				freq	= 24576000;
			else {
				pr_info("Setting sysclk rate %lu is not supported.\n", rate );
				return -EINVAL;
			}

			while( freq < (rate * 2 * period) ) {
				freq	*= 2;
			}

			div = freq / 2 / period / rate;
		}

		for (i = 0; i < ARRAY_SIZE(div_tb) - 1; i++)
			if (div_tb[i] >= div)
				break;

		dev_dbg(priv->dev, "%s: mclk freq = %lu. bclk div = %u. CLKD_BCLKDIV = %u", __func__, freq, div, i+1);

		if (100000000 < freq) {
			pr_info("Setting sysclk rate %lu is not supported.\n", rate );
			return -EINVAL;
 		}

		ret = clk_set_rate(priv->mod_clk, freq);
		if (ret) {
			pr_info("Setting sysclk rate failed %d\n", ret);
			return ret;
		}
		{
			unsigned long actual_rate = clk_get_rate(priv->mod_clk);
			long error_rate  = 1000000 - ((long)freq * 1000000 / (long)actual_rate);
			dev_dbg(priv->dev, "%s: mclk actual freq = %lu. error = %ld ppm", __func__, actual_rate, error_rate);
		}

		/* set the mclk and bclk dividor register */
		regmap_write(priv->regmap, I2S_CLKD,
					I2S_CLKD_H3_MCLKOEN | I2S_CLKD_MCLKDIV(1) | I2S_CLKD_BCLKDIV(i + 1));
	}else{
		dev_dbg(priv->dev, "%s: skip pll clk_set_rate. (not in CBS_CFS mode)", __func__);
	}

	/* format */
	regmap_update_bits(priv->regmap, I2S_FAT0,
				I2S_FAT0_H3_LRCKR_PERIOD_MSK | I2S_FAT0_H3_LRCK_PERIOD_MSK,
				I2S_FAT0_H3_LRCK_PERIOD(period - 1) | I2S_FAT0_H3_LRCKR_PERIOD(PCM_LRCKR_PERIOD - 1));

	regmap_update_bits(priv->regmap, I2S_FAT0,
				I2S_FAT0_H3_SW_MSK | I2S_FAT0_H3_SR_MSK,
				I2S_FAT0_H3_SW_16 | I2S_FAT0_H3_SR_16);
	regmap_write(priv->regmap, I2S_FAT1, 0);

	return 0;
}

static void sun8i_i2s_tx_set_channels(struct priv *priv, int nchan)
{
	u32 reg = 0;
	int n;

	dev_dbg(priv->dev, "%s: nchan = %d", __func__, nchan);

	priv->nchan = nchan;
	regmap_update_bits(priv->regmap, I2S_TXCHCFG_H3,
				I2S_TXCHCFG_H3_TX_SLOT_NUM_MSK,
				I2S_TXCHCFG_H3_TX_SLOT_NUM(nchan - 1));

	regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
				I2S_TXn_H3_CHEN_MSK,
				I2S_TXn_H3_CHEN((0x01 << nchan) - 1));
	regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
				I2S_TXn_H3_CHSEL_MSK,
				I2S_TXn_H3_CHSEL(nchan - 1));

	reg = 0;
	for(n = 0; n < nchan; n++) {
		reg |= (((0x00000001 << n) - 0x00000001) << (n * 4));
	}
	regmap_write(priv->regmap, I2S_TX0CHMAP_H3, reg);

	reg = 0;
	if (nchan >= 7)
		reg |= I2S_CTL_SDO3EN;
	if (nchan >= 5)
		reg |= I2S_CTL_SDO2EN;
	if (nchan >= 3)
		reg |= I2S_CTL_SDO1EN;
	reg |= I2S_CTL_SDO0EN;
	regmap_update_bits(priv->regmap, I2S_CTL,
			   I2S_CTL_SDOEN_MSK,
			   reg);
}

static int sun8i_i2s_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);
	int nchan = priv->nchan;

	dev_dbg(priv->dev, "%s: reached.", __func__);

	/* Enable the whole hardware block */
	regmap_update_bits(priv->regmap, I2S_CTL,
			   I2S_CTL_GEN,
			   I2S_CTL_GEN);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sun8i_i2s_tx_set_channels(priv, nchan);
		regmap_write(priv->regmap, I2S_TXCNT, 0);
	} else { // TODO
		return -EINVAL;
	}

	if (priv->clk_always_on == 1) {
		dev_dbg(priv->dev, "%s: clk always on.", __func__);
		return 0;
	}else
	if (priv->clk_always_on == 0){
		priv->clk_always_on = 1; // set clk start state.
	}

	return clk_prepare_enable(priv->mod_clk);
}

static void sun8i_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	dev_dbg(priv->dev, "%s: reached.", __func__);

	if (priv->clk_always_on == 1) {
		dev_dbg(priv->dev, "%s: clk always on.", __func__);
		return;
	}
	clk_disable_unprepare(priv->mod_clk);

	regmap_update_bits(priv->regmap, I2S_CTL,
			   I2S_CTL_GEN,
			   0);
	
	priv->fs = 0;
	priv->div = 0;
}

static int sun8i_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);
	int nchan = params_channels(params);
	int sample_resolution;
	int ret;

	dev_dbg(priv->dev, "%s: reached line %d, rate = %u, format = %d, nchan = %d.",
	       __func__, __LINE__,
	       params_rate(params), params_format(params), nchan);

	if (nchan < 1 || nchan > 8) return -EINVAL;
	sun8i_i2s_tx_set_channels(priv, nchan);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_resolution = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sample_resolution = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_resolution = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_resolution = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = sun8i_i2s_set_clock(priv, params_rate(params), sample_resolution);
	if (ret) {
		return ret;
	}

	if (sample_resolution == 16) {
		priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	}else{
		priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}

	dev_dbg(priv->dev, "%s: sample_resolution = %d\n", __func__, sample_resolution);

	if ((priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_RIGHT_J) { // RJ support
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_SR_MSK | I2S_FAT0_H3_SW_MSK,
					I2S_FAT0_H3_SR(sample_resolution) | I2S_FAT0_H3_SW(priv->rj_slotwidth));
		regmap_update_bits(priv->regmap, I2S_FCTL,
					I2S_FCTL_TXIM,
					I2S_FCTL_TXIM);
	}else{
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_SR_MSK | I2S_FAT0_H3_SW_MSK,
					I2S_FAT0_H3_SR(sample_resolution) | I2S_FAT0_H3_SW(sample_resolution));
		regmap_update_bits(priv->regmap, I2S_FCTL,
					I2S_FCTL_TXIM,
					I2S_FCTL_TXIM);
	}

	/* flush TX FIFO */
	regmap_update_bits(priv->regmap, I2S_FCTL,
			   I2S_FCTL_FTX,
			   I2S_FCTL_FTX);

	return 0;
}

static int sun8i_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	dev_dbg(priv->dev, "%s: fmt = 0x%x.", __func__, fmt);
	priv->dai_fmt = fmt;

	/* DAI Mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_MODE_MSK,
					I2S_CTL_H3_MODE_I2S);
		regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
					I2S_TXn_H3_OFFSET_MSK,
					I2S_TXn_H3_OFFSET(1));
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_MODE_MSK,
					I2S_CTL_H3_MODE_I2S);
		regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
					I2S_TXn_H3_OFFSET_MSK,
					I2S_TXn_H3_OFFSET(0));
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_MODE_MSK,
					I2S_CTL_H3_MODE_RGT);
		regmap_update_bits(priv->regmap, I2S_TX0CHSEL_H3,
					I2S_TXn_H3_OFFSET_MSK,
					I2S_TXn_H3_OFFSET(0));
		break;
	default:
		return -EINVAL;
	}

	/* DAI clock polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		/* Invert both clocks */
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY,
					I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY,
					I2S_FAT0_H3_BCLK_POLARITY);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* Invert frame clock */
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY,
					I2S_FAT0_H3_LRCK_POLARITY);
		break;
	case SND_SOC_DAIFMT_NB_NF:
		/* Nothing to do for both normal cases */
		regmap_update_bits(priv->regmap, I2S_FAT0,
					I2S_FAT0_H3_BCLK_POLARITY | I2S_FAT0_H3_LRCK_POLARITY,
					0);
		break;
	default:
		return -EINVAL;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* DAI Master */
		regmap_update_bits(priv->regmap, I2S_CLKD,
					I2S_CLKD_H3_MCLKOEN,
					I2S_CLKD_H3_MCLKOEN);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_BCLKOUT,
					I2S_CTL_H3_BCLKOUT);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_LRCKOUT,
					I2S_CTL_H3_LRCKOUT);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* Codec Master */
		regmap_update_bits(priv->regmap, I2S_CLKD,
					I2S_CLKD_H3_MCLKOEN,
					0);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_BCLKOUT,
					0);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_LRCKOUT,
					0);
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/* Bclk Slave / LRclk master */
		regmap_update_bits(priv->regmap, I2S_CLKD,
					I2S_CLKD_H3_MCLKOEN,
					0);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_BCLKOUT,
					0);
		regmap_update_bits(priv->regmap, I2S_CTL,
					I2S_CTL_H3_LRCKOUT,
					I2S_CTL_H3_LRCKOUT);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sun8i_i2s_set_bclk_ratio(struct snd_soc_dai *dai,
				      unsigned int ratio)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	priv->fs = ratio;

	return 0;
}

static int sun8i_i2s_set_bclk_clkdiv(struct snd_soc_dai *dai,
				      int div_id, int div)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	priv->div = div;

	return 0;
}

static void sun8i_i2s_start_playback(struct priv *priv)
{
	/* Clear TX counter */
	regmap_write(priv->regmap, I2S_TXCNT, 0);

	/* Flush TX FIFO */
	regmap_update_bits(priv->regmap, I2S_FCTL,
			   I2S_FCTL_FTX,
			   I2S_FCTL_FTX);

	/* Enable TX Block */
	regmap_update_bits(priv->regmap, I2S_CTL,
			   I2S_CTL_TXEN,
			   I2S_CTL_TXEN);

	/* Enable TX DRQ */
	regmap_update_bits(priv->regmap, I2S_INT,
			   I2S_INT_TXDRQEN,
			   I2S_INT_TXDRQEN);
}


static void sun8i_i2s_stop_playback(struct priv *priv)
{
	/* Clear TX counter */
	regmap_write(priv->regmap, I2S_TXCNT, 0);

	/* Flush TX FIFO */
	regmap_update_bits(priv->regmap, I2S_FCTL,
			   I2S_FCTL_FTX,
			   I2S_FCTL_FTX);

	if (priv->clk_always_on != 1) {
		/* Disable TX Block */
		regmap_update_bits(priv->regmap, I2S_CTL,
				   I2S_CTL_TXEN,
				   0);
	}else{
		dev_dbg(priv->dev, "%s: clk always on.", __func__);
	}

	/* Disable TX DRQ */
	regmap_update_bits(priv->regmap, I2S_INT,
			   I2S_INT_TXDRQEN,
			   0);
}

static int sun8i_i2s_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	dev_dbg(priv->dev, "%s: cmd = %d, substream->stream = %d\n",
	       __func__, cmd, substream->stream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sun8i_i2s_start_playback(priv);
		} else { // TODO
			return -EINVAL;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sun8i_i2s_stop_playback(priv);
		} else { // TODO
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sun8i_i2s_dai_ops = {
	.hw_params	= sun8i_i2s_hw_params,
	.set_fmt	= sun8i_i2s_set_fmt,
	.set_bclk_ratio = sun8i_i2s_set_bclk_ratio,
	.set_clkdiv = sun8i_i2s_set_bclk_clkdiv,
	.shutdown	= sun8i_i2s_shutdown,
	.startup	= sun8i_i2s_startup,
	.trigger	= sun8i_i2s_trigger,
};

static int sun8i_i2s_controls_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int shift = *((unsigned int*)&mc->shift);
	unsigned int ret = 0;

	if (regmap_read(priv->regmap, mc->reg, &ret) != 0)
		return -EINVAL;

	ret &= shift;
	ucontrol->value.integer.value[0] = *((int*)&ret);
	return 0;
}

static int sun8i_i2s_controls_set(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int value = *((unsigned int*)ucontrol->value.integer.value);
	unsigned int shift = *((unsigned int*)&mc->shift);
	unsigned int max = *((unsigned int*)&mc->max);

	if (value > max)
		return -EINVAL;

	regmap_update_bits(priv->regmap, mc->reg,
			   shift,
			   value);
	return 0;
}

static const struct snd_kcontrol_new sun8i_i2s_controls[] = {
	SOC_SINGLE_EXT("Mute Switch", I2S_CTL, I2S_CTL_OUTMUTE,
		       I2S_CTL_OUTMUTE, 0,
		       sun8i_i2s_controls_get, sun8i_i2s_controls_set),
};

static int sun8i_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct priv *priv = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &priv->playback_dma_data, NULL);
	snd_soc_add_dai_controls(dai, sun8i_i2s_controls, ARRAY_SIZE(sun8i_i2s_controls));

	return 0;
}

static struct snd_soc_dai_driver sun8i_i2s_dai = {
	.name = "sun8i-i2s",
	.probe = sun8i_i2s_dai_probe,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 8000,
		.rate_max = 1536000,
		.formats = I2S_FORMATS,
	},
	.ops = &sun8i_i2s_dai_ops,
	.symmetric_rate        = true,
};

static const struct snd_soc_component_driver sun8i_i2s_component = {
	.name			= "sun8i-i2s-comp",
};

/* --- dma --- */

static const struct snd_pcm_hardware sun8i_i2s_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = I2S_FORMATS,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min = 8000,
	.rate_max = 1536000,
	.channels_min = 1,
	.channels_max = 8,
	.buffer_bytes_max = 1024 * 1024,
	.period_bytes_min = 156,
	.period_bytes_max = 1024 * 1024,
	.periods_min = 1,
	.periods_max = 8,
	.fifo_size = 128,
};

static const struct snd_dmaengine_pcm_config sun8i_i2s_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &sun8i_i2s_pcm_hardware,
	.prealloc_buffer_size = 1024 * 1024,
};

static int snd_sun8i_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int snd_sun8i_dac_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops snd_sun8i_dac_ops = {
        .hw_params = snd_sun8i_dac_hw_params,
};

static struct snd_soc_dai_link snd_sun8i_dac_dai = {
	.name		= "h3/h5-i2s-dac",
	.stream_name	= "h3/h5-i2s-dac",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			  SND_SOC_DAIFMT_CBS_CFS,
	.ops		= &snd_sun8i_dac_ops,
	.init		= snd_sun8i_dac_init,
};

static const struct snd_soc_card snd_sun8i_dac = {
	.name		= "snd-sun8i-i2s-dac",
};

/* --- regmap --- */

static bool sun8i_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_TXFIFO:
		return false;

	default:
		return true;
	}
}

static bool sun8i_i2s_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_RXFIFO:
	case I2S_FSTA:
		return false;

	default:
		return true;
	}
}

static bool sun8i_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_RXFIFO:
	case I2S_FSTA:
	case I2S_TXCNT:
	case I2S_RXCNT:
		return true;

	default:
		return false;
	}
}

static const struct reg_default sun8i_i2s_reg_h3_defaults[] = {
	{ I2S_CTL, 0x00060000 }, // BCLK & LRCK for output
	{ I2S_FAT0, 0x00000033 }, 
	{ I2S_FAT1, 0x00000030 },
	{ I2S_FCTL, 0x000400f0 },
	{ I2S_INT, 0x00000000 },
	{ I2S_CLKD, 0x00000000 },
	{ I2S_TX0CHSEL_H3, 0x00000001 }, // 2 channels
	{ I2S_TX0CHMAP_H3, 0x76543210 },
	{ I2S_RXCHSEL_H3, 0x00000001 }, // 2 channels
	{ I2S_RXCHMAP_H3, 0x00000000 },
};

static const struct regmap_config sun8i_i2s_regmap_h3_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= I2S_RXCHMAP_H3,

	.cache_type		= REGCACHE_FLAT,
	.reg_defaults		= sun8i_i2s_reg_h3_defaults,
	.num_reg_defaults	= ARRAY_SIZE(sun8i_i2s_reg_h3_defaults),
	.writeable_reg		= sun8i_i2s_wr_reg,
	.readable_reg		= sun8i_i2s_rd_reg,
	.volatile_reg		= sun8i_i2s_volatile_reg,
};

/* --- pm --- */

static int sun8i_i2s_runtime_resume(struct device *dev)
{
	struct priv *priv = dev_get_drvdata(dev);
	int ret;

	if (!IS_ERR_OR_NULL(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret) {
			dev_err(dev, "Failed to enable bus clock\n");
			return ret;
		}
	}

	regcache_cache_only(priv->regmap, false);

	ret = regcache_sync(priv->regmap);
	if (ret) {
		dev_err(dev, "Failed to sync regmap cache\n");
		goto err_disable_clk;
	}

	return 0;

err_disable_clk:
	if (!IS_ERR_OR_NULL(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);
	return ret;
}

static int sun8i_i2s_runtime_suspend(struct device *dev)
{
	struct priv *priv = dev_get_drvdata(dev);

	regcache_cache_only(priv->regmap, true);
	regcache_mark_dirty(priv->regmap);

	if (!IS_ERR_OR_NULL(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);

	return 0;
}

static void sun8i_i2s_parse_device_tree_options(struct device *dev, struct priv *priv)
{
	int ret, cnt;
	struct snd_soc_dai_link *dai = &snd_sun8i_dac_dai;

	/* get lrclk period map */
	cnt = of_property_count_strings(dev->of_node, "lrclk_periods");
	if ( cnt > 0 ) {
		int i;
		for ( i = 0 ; i < cnt ; i++) {
			const char *output;
			char *endp;
			ret = of_property_read_string_index(dev->of_node, "lrclk_periods", i, &output);
			if (ret) break;
			priv->lrclk_period_map |= PERIOD_TO_MAP(simple_strtol(output, &endp, 10));
			dev_dbg(dev, "%s: lrclk_periods[%d] = %s\n", __func__, i,output);
		}
	}else{
		priv->lrclk_period_map = PERIOD_TO_MAP(32); // default 64fs only
	}
	dev_dbg(dev, "%s: priv->lrclk_period_map = %x\n", __func__, priv->lrclk_period_map);

	/* get mclk max frequency */
	{
		u32 output;
		priv->mclk_max_freq = 0;
		ret = of_property_read_u32(dev->of_node, "mclk_max_freq", &output);
		if (ret == 0){
			priv->mclk_max_freq = output;
			dev_dbg(dev, "%s: priv->mclk_max_freq = %d\n", __func__, priv->mclk_max_freq);
		}
	}

	/* get dai_fmt master override*/
	{
		const char *output;
		unsigned int dai_fmt = 0;
		ret = of_property_read_string(dev->of_node, "daifmt_master_override", &output);
		if (ret == 0){
			if (!strncmp(output, "CBM_CFM", sizeof("CBM_CFM"))) {
				dai_fmt = SND_SOC_DAIFMT_CBM_CFM;
			}else
			if (!strncmp(output, "CBS_CFS", sizeof("CBS_CFS"))) {
				dai_fmt = SND_SOC_DAIFMT_CBS_CFS;
			}else
			if (!strncmp(output, "CBM_CFS", sizeof("CBM_CFS"))) {
				dai_fmt = SND_SOC_DAIFMT_CBM_CFS;
			}
		}

		if (dai_fmt) {
			dai->dai_fmt = (dai->dai_fmt & (~SND_SOC_DAIFMT_MASTER_MASK)) | dai_fmt;
			dev_info(dev, "daifmt_master_override = %s\n", output);
		}
	}

	/* get dai_fmt audio override*/
	{
		const char *output;
		unsigned int dai_fmt = 0;
		ret = of_property_read_string(dev->of_node, "daifmt_audio_override", &output);
		if (ret == 0){
			if (!strncmp(output, "I2S", sizeof("I2S"))) {
				dai_fmt = SND_SOC_DAIFMT_I2S;
			}else
			if (!strncmp(output, "RJ", sizeof("RJ"))) {
				dai_fmt = SND_SOC_DAIFMT_RIGHT_J;
			}else
			if (!strncmp(output, "LJ", sizeof("LJ"))) {
				dai_fmt = SND_SOC_DAIFMT_LEFT_J;
			}
		}

		if (dai_fmt) {
			dai->dai_fmt = (dai->dai_fmt & (~SND_SOC_DAIFMT_FORMAT_MASK)) | dai_fmt;
			dev_info(dev, "daifmt_audio_override = %s\n", output);
		}
	}

	/* get dai_fmt polarity override*/
	{
		const char *output;
		unsigned int dai_fmt = 0;
		ret = of_property_read_string(dev->of_node, "daifmt_polarity_override", &output);
		if (ret == 0){
			if (!strncmp(output, "IB_IF", sizeof("IB_IF"))) {
				dai_fmt = SND_SOC_DAIFMT_IB_IF;
			}else
			if (!strncmp(output, "IB_NF", sizeof("IB_NF"))) {
				dai_fmt = SND_SOC_DAIFMT_IB_NF;
			}else
			if (!strncmp(output, "NB_IF", sizeof("NB_IF"))) {
				dai_fmt = SND_SOC_DAIFMT_NB_IF;
			}else
			if (!strncmp(output, "NB_NF", sizeof("NB_NF"))) {
				dai_fmt = SND_SOC_DAIFMT_NB_NF;
			}
		}

		if (dai_fmt) {
			dai->dai_fmt = (dai->dai_fmt & (~SND_SOC_DAIFMT_INV_MASK)) | dai_fmt;
			dev_info(dev, "daifmt_polarity_override = %s\n", output);
		}
	}

	/* get Right justified slot width */
	{
		u32 output;
		priv->rj_slotwidth = 32;
		ret = of_property_read_u32(dev->of_node, "rj_slotwidth", &output);
		if (ret == 0){
			priv->rj_slotwidth = output;
			dev_dbg(dev, "%s: priv->rj_slotwidth = %d\n", __func__, priv->rj_slotwidth);
		}
	}

	/* get clk always on */
	{
		u32 output;
		priv->clk_always_on = -1; // default off.
		ret = of_property_read_u32(dev->of_node, "clk_always_on", &output);
		if (ret == 0 && output == 1){
			priv->clk_always_on = 0; // clk alwasys on enable, set clk stop state.
			dev_dbg(dev, "%s: priv->clk_always_on = enabled\n", __func__);
		}
	}

	return;
}



/* --- module init --- */

static int sun8i_i2s_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct priv *priv;
	struct resource *res;
	void __iomem *mmio;
	int irq, ret;

	if (!dev->of_node) {
		dev_err(dev, "no DT!\n");
		return -EINVAL;
	}
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* get the resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(mmio)) {
		dev_err(dev, "Can't request IO region\n");
		return PTR_ERR(mmio);
	}
	dev_dbg(dev, "%s: resource=%pR, name=%s, mmio=%p\n", __func__, res, res->name, mmio);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Can't retrieve interrupt\n");
		return irq;
	}

	/* get SoC type */
	priv->type = (long int) of_match_device(sun8i_i2s_of_match, &pdev->dev)->data;
	dev_dbg(dev, "%s: priv->type = %d\n", __func__, priv->type);

	/* get and enable the clocks */
	priv->bus_clk = devm_clk_get(dev, "apb");	/* optional */
	priv->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(priv->mod_clk)) {
		dev_err(dev, "Can't get mod clock\n");
		return PTR_ERR(priv->mod_clk);
	}
	ret = clk_set_rate(priv->mod_clk, 24576000);
	if (ret) {
		dev_err(dev, "Can't set rate of i2s clock\n");
		return ret;
	}

	sun8i_i2s_parse_device_tree_options(dev, priv);

	priv->regmap = devm_regmap_init_mmio(dev, mmio, &sun8i_i2s_regmap_h3_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Regmap initialisation failed\n");
		return PTR_ERR(priv->regmap);
	}

	priv->rstc = devm_reset_control_get_optional(dev, NULL);
	if (!IS_ERR(priv->rstc)) {
		ret = reset_control_deassert(priv->rstc);
		if (ret < 0)
			return ret;
	}

	priv->playback_dma_data.maxburst = 8;
	priv->playback_dma_data.addr = res->start + I2S_TXFIFO;
	priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	pm_runtime_enable(dev);
	if (!pm_runtime_enabled(dev)) {
		ret = sun8i_i2s_runtime_resume(dev);
		if (ret)
			goto err_pm_disable;
	}

	priv->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, priv);

	/* activate the audio subsystem */
	sun8i_i2s_init(priv);

	ret = devm_snd_soc_register_component(dev,
					      &sun8i_i2s_component,
					      &sun8i_i2s_dai, 1);
	if (ret) {
		dev_err(dev, "Could not register DAI, error_code = %d\n", ret);
		goto err_suspend;
	}

	ret = devm_snd_dmaengine_pcm_register(dev, &sun8i_i2s_config, 0);
	if (ret) {
		dev_err(dev, "Could not register PCM, error_code = %d\n", ret);
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(dev))
		sun8i_i2s_runtime_suspend(dev);
err_pm_disable:
	pm_runtime_disable(dev);

	if (!IS_ERR_OR_NULL(priv->rstc))
		reset_control_assert(priv->rstc);

	return ret;
}

static int sun8i_i2s_dev_remove(struct platform_device *pdev)
{
	struct priv *priv = dev_get_drvdata(&pdev->dev);

	snd_dmaengine_pcm_unregister(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		sun8i_i2s_runtime_suspend(&pdev->dev);

	if (!IS_ERR_OR_NULL(priv->rstc))
		reset_control_assert(priv->rstc);

	return 0;
}

static const struct dev_pm_ops sun8i_i2s_pm_ops = {
	.runtime_resume		= sun8i_i2s_runtime_resume,
	.runtime_suspend	= sun8i_i2s_runtime_suspend,
};

static struct platform_driver sun8i_i2s_driver = {
	.probe  = sun8i_i2s_dev_probe,
	.remove = sun8i_i2s_dev_remove,
	.driver = {
		.name = "sun8i-i2s",
		.of_match_table = sun8i_i2s_of_match,
		.pm = &sun8i_i2s_pm_ops,
	},
};
module_platform_driver(sun8i_i2s_driver);

MODULE_AUTHOR("Jean-Francois Moine <moinejf at free.fr>");
MODULE_AUTHOR("Anthony Lee <don.anthony.lee at gmail.com>");
MODULE_AUTHOR("__tkz__ <tkz at lrclk.com>");
MODULE_DESCRIPTION("Allwinner sun8i I2S ASoC Interface for Nanopi NEO/NEO2 series");
MODULE_LICENSE("GPL v2");
