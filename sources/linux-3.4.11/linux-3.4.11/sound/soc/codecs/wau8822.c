
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "wau8822.h"

#define CODEC_NAME	"wau8822"

#define wau8822_reset(c) wau8822_write(c, WAU8822_RESET, 0)

struct wau8822_priv {
	enum snd_soc_control_type	control_type;
	void				*control_data;
	u16				suspended_cache[WAU8822_CACHEREG_NUM];
};

/* wau8822 default cfg values */
static const u16 wau8822_reg[WAU8822_CACHEREG_NUM] = {
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0050, 0x0000, 0x0140, 0x0000,
	0x0000, 0x0080, 0x0000, 0x00ff,
	0x00ff, 0x0000, 0x0100, 0x00ff,
	0x00ff, 0x0000, 0x012c, 0x002c,
	0x002c, 0x002c, 0x002c, 0x0000,
	0x0032, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0038, 0x000b, 0x0032, 0x0010,
	0x0008, 0x000c, 0x0093, 0x00e9,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0033, 0x0010, 0x0010, 0x0100,
	0x0100, 0x0002, 0x0001, 0x0001,
	0x0039, 0x0039, 0x0000, 0x0000,
	0x0001, 0x0001, 0x0000, 0x0000,
	0x0020, 0x000 , 0x0000, 0x001A,
};

/* wau8822 custom defaults values */
static const struct reg_default wau8822_reg_defaults[] = {
	/* starting values other than default ones */
	{WAU8822_POWER1,	0x001f}, /*
					  * VREF 3kohm nominal impedance
					  * tie-off buffer power on
					  * bias buffer power on
					  * micbias enabled
					  */
	{WAU8822_POWER2,	0x01bf}, /*
					  * left ADC on
					  * right ADC on
					  * left PGA on
					  * right PGA on
					  * left ADC mix / boost on
					  * right ADC mix / boost on
					  * left headphone on
					  * right headphone on
					  */
	{WAU8822_POWER3,	0x006f}, /*
					  * left DAC on
					  * right DAC on
					  * left main mixer on
					  * right main mixer on
					  * left spk on
					  * right spk on
					  */
	{WAU8822_IFACE,		0x0070}, /* standard i2s */
	{WAU8822_CLOCK1,	0x0000}, /* divider for fs set to 1 */
	{WAU8822_GPIO,          0x0004}, /* output divided PLL clock */
	{WAU8822_DAC,		0x0008}, /* 128x oversampling */
	{WAU8822_ADCVOLL,	0x01ff}, /* left adc set to 0dB */
	{WAU8822_ADCVOLR,	0x01ff}, /* right adc set to 0dB */
	{WAU8822_INPUT,		0x0000}, /* disconnect all input */
	{WAU8822_INPGAL,	0x0110}, /* left PGA 0dB */
	{WAU8822_INPGAR,	0x0110}, /* right PGA 0dB */
	{WAU8822_ADCBOOSTL,	0x0055}, /*
					  * left line-in 0dB
					  * left aux-in 0dB
					  */
	{WAU8822_ADCBOOSTR,	0x0055}, /*
					  * right line-in 0dB
					  * right aux-in 0dB
					  */
};

static inline unsigned int wau8822_read_reg_cache(struct snd_soc_codec *codec,
						  unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WAU8822_RESET)
		return 0;
	if (reg >= WAU8822_CACHEREG_NUM)
		return -1;
	return cache[reg];
}
static inline void wau8822_write_reg_cache(struct snd_soc_codec *codec,
					   u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WAU8822_CACHEREG_NUM)
		return;
	cache[reg] = value;
}

static int wau8822_write(struct snd_soc_codec *codec,
			 unsigned int reg,
			 unsigned int value)
{
	int ret = 0;
	wau8822_write_reg_cache(codec, reg, value);

	ret = snd_soc_write(codec, reg, value);
	if (!ret) {
		dev_dbg(codec->dev, "%s: reg = 0x%x ; value = 0x%x success\n",
			__func__, reg, value);
	} else {
		dev_err(codec->dev, "%s: reg = 0x%x ; value = 0x%x error\n",
			__func__, reg, value);
		ret = -EIO;
	}

	return ret;
}

static const char * const wau8822_pga_input_sel_text[] = {
	"None", "Line", "Mic"
};

static const struct soc_enum wau8822_pga_input_sel_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wau8822_pga_input_sel_text),
			    wau8822_pga_input_sel_text);

int wau8822_vol_update(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	bool type_2r = 0;
	unsigned int val2 = 0;
	unsigned int val, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (snd_soc_volsw_is_stereo(mc)) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		if (reg == reg2) {
			val_mask |= mask << rshift;
			val |= val2 << rshift;
		} else {
			val2 = val2 << shift;
			type_2r = 1;
		}
	}

	err = snd_soc_update_bits_locked(codec,
					 reg,
					 val_mask | 0x100,
					 val | 0x100);
	if (err < 0)
		return err;

	if (type_2r)
		err = snd_soc_update_bits_locked(codec,
						 reg2,
						 val_mask | 0x100,
						 val2 | 0x100);

	return err;
}

static int wau8822_pga_input_get_switch(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 input_left  = wau8822_read_reg_cache(codec, WAU8822_INPUT) & 0x7;
	u16 input_right = (wau8822_read_reg_cache(codec, WAU8822_INPUT) >> 4)
			  & 0x7;

	if (!input_left && !input_right) {
		/* PGA input none */
		ucontrol->value.integer.value[0] = 0;
	} else if (input_left == 0x3 && input_right == 0x3) {
		/* PGA input is Mic */
		ucontrol->value.integer.value[0] = 2;
	} else if (input_left == 0x4 && input_right == 0x4) {
		/* PGA input is Line-in */
		ucontrol->value.integer.value[0] = 1;
	} else {
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
	}

	return ret;
}

static int wau8822_pga_input_set_switch(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 input = wau8822_read_reg_cache(codec, WAU8822_INPUT) & ~0x77;

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		break;
	case 1:
		/* Line-in */
		input |= 0x44;
		break;
	case 2:
		/* Mic */
		input |= 0x33;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		goto error;
		break;
	}

	wau8822_write(codec, WAU8822_INPUT, input);

error:
	return ret;
}

static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 50, 0);
static const DECLARE_TLV_DB_SCALE(pga_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(hp_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(boost_tlv, -1500, 300, 0);

static const struct snd_kcontrol_new wau8822_snd_controls[] = {
	SOC_SINGLE("Digital Loopback Switch", WAU8822_COMP, 0, 1, 0),

	SOC_DOUBLE_R_EXT_TLV("DAC Playback Volume",
			     WAU8822_DACVOLL, WAU8822_DACVOLR, 0, 255, 0,
			     snd_soc_get_volsw, wau8822_vol_update, dac_tlv),

	SOC_DOUBLE_R_EXT_TLV("ADC Capture Volume",
			     WAU8822_ADCVOLL, WAU8822_ADCVOLR, 0, 255, 0,
			     snd_soc_get_volsw, wau8822_vol_update, dac_tlv),

	SOC_DOUBLE_R_EXT_TLV("PGA Capture Volume",
			     WAU8822_INPGAL, WAU8822_INPGAR, 0, 63, 0,
			     snd_soc_get_volsw, wau8822_vol_update, pga_tlv),

	SOC_DOUBLE_R_EXT_TLV("Headphone Playback Volume",
			     WAU8822_HPVOLL, WAU8822_HPVOLR, 0, 63, 0,
			     snd_soc_get_volsw, wau8822_vol_update, hp_tlv),
	SOC_DOUBLE_R("Headphone Playback Switch",
		     WAU8822_HPVOLL, WAU8822_HPVOLR, 6, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("Speaker Playback Volume",
			     WAU8822_SPKVOLL, WAU8822_SPKVOLR, 0, 63, 0,
			     snd_soc_get_volsw, wau8822_vol_update, hp_tlv),
	SOC_DOUBLE_R("Speaker Playback Switch",
		     WAU8822_SPKVOLL, WAU8822_SPKVOLR, 6, 1, 1),

	SOC_DOUBLE_R_TLV("Line In Capture Volume", WAU8822_ADCBOOSTL,
			 WAU8822_ADCBOOSTR, 4, 7, 0, boost_tlv),
	SOC_DOUBLE_R_TLV("Aux In Capture Volume", WAU8822_ADCBOOSTL,
			 WAU8822_ADCBOOSTR, 0, 7, 0, boost_tlv),
	SOC_DOUBLE_R("Capture Boost(+20dB)",
		     WAU8822_ADCBOOSTL, WAU8822_ADCBOOSTR,  8, 1, 0),

	SOC_ENUM_EXT("PGA Input Selection", wau8822_pga_input_sel_enum,
		     wau8822_pga_input_get_switch,
		     wau8822_pga_input_set_switch),
};

static int wau8822_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	u16 iface = wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x19f;
	u16 clk2 = wau8822_read_reg_cache(codec, WAU8822_CLOCK2) & 0x1f1;

	/* set word length */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0060;
		break;
	}

	/* set sample rate */
	switch (params_rate(params)) {
	case SNDRV_PCM_RATE_8000:
		clk2 |= 0x5 << 1;
		break;
	case SNDRV_PCM_RATE_11025:
		clk2 |= 0x4 << 1;
		break;
	case SNDRV_PCM_RATE_16000:
		clk2 |= 0x3 << 1;
		break;
	case SNDRV_PCM_RATE_22050:
		clk2 |= 0x2 << 1;
		break;
	case SNDRV_PCM_RATE_32000:
		clk2 |= 0x1 << 1;
		break;
	case SNDRV_PCM_RATE_44100:
	case SNDRV_PCM_RATE_48000:
		break;
	}

	wau8822_write(codec, WAU8822_IFACE, iface);
	wau8822_write(codec, WAU8822_CLOCK2, clk2);
	return 0;
}

static int wau8822_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x67;
	u16 clk1 = wau8822_read_reg_cache(codec, WAU8822_CLOCK1) & 0x1fe;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clk1 |= 0x0001;
		break;

	case SND_SOC_DAIFMT_CBS_CFS:
		break;

	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0010;
		break;

	case SND_SOC_DAIFMT_RIGHT_J:
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0008;
		break;

	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x00018;
		break;

	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;

	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0180;
		break;

	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0100;
		break;

	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0080;
		break;

	default:
		return -EINVAL;
	}

	wau8822_write(codec, WAU8822_IFACE, iface);
	wau8822_write(codec, WAU8822_CLOCK1, clk1);
	return 0;
}

static int wau8822_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				  int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WAU8822_OPCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_GPIO) & 0x1cf;
		wau8822_write(codec, WAU8822_GPIO, reg | div);
		break;
	case WAU8822_MCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1) & 0x11f;
		wau8822_write(codec, WAU8822_CLOCK1, reg | div);
		break;
	case WAU8822_ADCCLK:
		reg = wau8822_read_reg_cache(codec, WAU8822_ADC) & 0x1f7;
		wau8822_write(codec, WAU8822_ADC, reg | div);
		break;
	case WAU8822_DACCLK:
		reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0x1f7;
		wau8822_write(codec, WAU8822_DAC, reg | div);
		break;
	case WAU8822_BCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1) & 0x1e3;
		wau8822_write(codec, WAU8822_CLOCK1, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct pll_ {
	unsigned int pre_div:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct pll_ pll_div;

/*
 * The size in bits of the pll divide multiplied by 10
 * to allow rounding later
 */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div.pre_div = 1;
		Ndiv = target / source;
	} else {
		pll_div.pre_div = 0;
	}

	if ((Ndiv < 6) || (Ndiv > 12))
		pr_warn("WAU8822 N value %d outwith recommended range!\n",
			Ndiv);

	pll_div.n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xffffffff;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div.k = K;

	pr_debug(CODEC_NAME "%s: pll_factor, freq_in %d, freq_out %d, "
		 "pre_div %d, Ndiv %d, K 0x%x\n", __func__,
		 source, target, pll_div.pre_div, pll_div.n, pll_div.k);
}
static int wau8822_set_dai_pll(struct snd_soc_dai *codec_dai,
			       int pll_id, int source,
			       unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	dev_dbg(codec->dev, "%s: pll_id %d, freq_in %d, frq_out %d\n",
		__func__, pll_id, freq_in, freq_out);

	if (freq_in == 0 || freq_out == 0) {
		/* Clock CODEC directly from MCLK, MCLK is master clock */
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1);
		wau8822_write(codec, WAU8822_CLOCK1, reg & 0x0ff);

		/* Turn off PLL */
		reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
		wau8822_write(codec, WAU8822_POWER1, reg & 0x1df);
		return 0;
	}

	/* compute PLL factors */
	pll_factors(freq_out * 8, freq_in);

	wau8822_write(codec, WAU8822_PLLN, (pll_div.pre_div << 4) | pll_div.n);
	wau8822_write(codec, WAU8822_PLLK1, pll_div.k >> 18);
	wau8822_write(codec, WAU8822_PLLK2, (pll_div.k >> 9) & 0x1ff);
	wau8822_write(codec, WAU8822_PLLK3, pll_div.k & 0x1ff);

	/* Turn on PLL */
	reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
	wau8822_write(codec, WAU8822_POWER1, reg | 0x020);

	/* Run CODEC from PLL instead of MCLK */
	reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1) & 0x11f;
	wau8822_write(codec, WAU8822_CLOCK1, reg | 0x100);

	/* adapt the MCLK */
	reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1);
	wau8822_write(codec, WAU8822_CLOCK1, reg | (0x020 << pll_div.pre_div));

	/* to try to put the out in aiz */
	wau8822_write(codec, WAU8822_MISC, 0x0);

	/*
	 * to adapt bclk - McBSP driver currently
	 * only support 16bits data
	 */
	reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK1) & 0x1e3;

	/*
	 * if divided by 8 -> 16BCLK per channel
	 * if divided by 4 -> 32BCLK per channel
	 * wau8822_write(codec, WAU8822_CLOCK1, reg | WAU8822_BCLKDIV_8);
	 */
	wau8822_write(codec, WAU8822_CLOCK1, reg | (2 << 2));

	return 0;
}

static int wau8822_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 mute_reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0xffbf;

	if (mute)
		wau8822_write(codec, WAU8822_DAC, mute_reg | 0x40);
	else
		wau8822_write(codec, WAU8822_DAC, mute_reg);

	return 0;
}

static struct snd_soc_dai_ops wau8822_dai_ops_playback = {
	.hw_params    = wau8822_pcm_hw_params,
	.set_fmt      = wau8822_set_dai_fmt,
	.set_clkdiv   = wau8822_set_dai_clkdiv,
	.set_pll      = wau8822_set_dai_pll,
	.digital_mute = wau8822_mute,
};

static int wau8822_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	u16 power1 = wau8822_read_reg_cache(codec, WAU8822_POWER1) & ~0x3;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		power1 |= 0x1;  /* VMID 50k */
		wau8822_write(codec, WAU8822_POWER1, power1);
		break;

	case SND_SOC_BIAS_STANDBY:
		power1 |= 0x08 | 0x04;

		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* Initial cap charge at VMID 5k */
			wau8822_write(codec, WAU8822_POWER1, power1 | 0x3);
			mdelay(100);
		}

		power1 |= 0x2;  /* VMID 500k */
		wau8822_write(codec, WAU8822_POWER1, power1);
		break;

	case SND_SOC_BIAS_OFF:
		wau8822_write(codec, WAU8822_POWER1, 0);
		wau8822_write(codec, WAU8822_POWER2, 0);
		wau8822_write(codec, WAU8822_POWER3, 0);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static int wau8822_power_down(struct snd_soc_codec *codec)
{
	/* power2 shutdown */
	wau8822_write(codec, WAU8822_POWER2, 0x0);
	/* power3 shutdown except main mixer */
	wau8822_write(codec, WAU8822_POWER3, 0xc);
	/* power3 complete shutdown */
	wau8822_write(codec, WAU8822_POWER3, 0x0);
	/* power2 shutdown */
	wau8822_write(codec, WAU8822_POWER1, 0x0);
	/* headphone output mute */
	wau8822_write(codec, WAU8822_HPVOLL, 0x140);
	wau8822_write(codec, WAU8822_HPVOLR, 0x140);

	return wau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int wau8822_probe(struct snd_soc_codec *codec)
{
	struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int i;
	unsigned int value = 0;

	dev_info(codec->dev, "WAU8822 Audio Codec");

	codec->control_data = wau8822->control_data;
	ret = snd_soc_codec_set_cache_io(codec, 7, 9, wau8822->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		goto error;
	}

	/* Reset  */
	ret = wau8822_reset(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to reset codec: %d\n", ret);
		goto error;
	}

	/* power on device */
	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* initialize codec register */
	for (i = 0; i < ARRAY_SIZE(wau8822_reg_defaults); i++) {
		wau8822_write(codec, wau8822_reg_defaults[i].reg,
			      wau8822_reg_defaults[i].def);
	}

	/* update cache */
	for (i = 0; i < WAU8822_CACHEREG_NUM; i++) {
		value = snd_soc_read(codec, i);
		wau8822_write_reg_cache(codec, i, value);
	}

	/* init controls */
	dev_dbg(codec->dev, "init controls\n");
	snd_soc_add_codec_controls(codec, wau8822_snd_controls,
				   ARRAY_SIZE(wau8822_snd_controls));

error:
	return ret;

}

static int wau8822_remove(struct snd_soc_codec *codec)
{
	return wau8822_power_down(codec);
}

static int wau8822_suspend(struct snd_soc_codec *codec)
{
	struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
	u16 *cache = codec->reg_cache;

	/* we only need to suspend if we are a valid card */
	if (!codec->card)
		return 0;

	/* save cache for suspend */
	memcpy(wau8822->suspended_cache, cache,
	       WAU8822_CACHEREG_NUM * sizeof(u16));

	return wau8822_power_down(codec);
}

static int wau8822_resume(struct snd_soc_codec *codec)
{
	int ret;
	int i;
	u16 *cache = codec->reg_cache;
	struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

	/* we only need to resume if we are a valid card */
	if (!codec->card)
		return 0;

	/* Reset  */
	ret = wau8822_reset(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to reset codec: %d\n", ret);
		goto error;
	}

	/* power on device */
	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* initialize codec register */
	for (i = 0; i < ARRAY_SIZE(wau8822_reg_defaults); i++) {
		snd_soc_write(codec, wau8822_reg_defaults[i].reg,
			      wau8822_reg_defaults[i].def);
	}

	/* restore suspended bias level */
	wau8822_set_bias_level(codec, codec->dapm.suspend_bias_level);

	/* restore cached regs */
	memcpy(cache, wau8822->suspended_cache,
	       WAU8822_CACHEREG_NUM * sizeof(u16));
	for (i = 0; i < ARRAY_SIZE(wau8822_reg); i++) {
		if (i == WAU8822_RESET)
			continue;
		wau8822_write(codec, i, cache[i]);
	}

error:
	return ret;
}

static int wau8822_volatile_register(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	return reg == WAU8822_RESET;
}

struct snd_soc_codec_driver soc_codec_dev_wau8822 = {
	.probe             = wau8822_probe,
	.remove            = wau8822_remove,
	.suspend           = wau8822_suspend,
	.resume            = wau8822_resume,
	.set_bias_level    = wau8822_set_bias_level,
	.volatile_register = wau8822_volatile_register,
	.reg_cache_size    = sizeof(wau8822_reg),
	.reg_word_size     = sizeof(u16),
	.reg_cache_default = &wau8822_reg,
};

#define WAU8822_RATES (SNDRV_PCM_RATE_8000  |	\
		       SNDRV_PCM_RATE_11025 |	\
		       SNDRV_PCM_RATE_16000 |	\
		       SNDRV_PCM_RATE_22050 |	\
		       SNDRV_PCM_RATE_44100 |	\
		       SNDRV_PCM_RATE_48000)

#define WAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |	\
			 SNDRV_PCM_FMTBIT_S20_3LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE  |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver wau8822_dai[] = {
	{
		.name     = "wau8822-hifi",
		.id       = 0,
		.playback = {
			.stream_name  = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates        = WAU8822_RATES,
			.formats      = WAU8822_FORMATS,
		},
		.capture = {
			.stream_name  = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates        = WAU8822_RATES,
			.formats      = WAU8822_FORMATS,
		},
		.ops = &wau8822_dai_ops_playback,
	},
};

static int wau8822_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct wau8822_priv *wau8822;
	int ret;

	wau8822 = kzalloc(sizeof(struct wau8822_priv), GFP_KERNEL);
	if (wau8822 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, wau8822);
	wau8822->control_data = i2c;
	wau8822->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_wau8822,
				     wau8822_dai, ARRAY_SIZE(wau8822_dai));

	if (ret < 0) {
		kfree(wau8822);
		pr_err(CODEC_NAME  ": %s failed: ret = %d\n", __func__, ret);
	}

	return ret;
}

static int wau8822_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec);
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

/*
 * WAU8822 2 wire address is determined by A1 pin
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
static const struct i2c_device_id wau8822_i2c_table[] = {
	{"wau8822", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, wau8822_i2c_table);

/*  i2c codec control layer */
static struct i2c_driver wau8822_i2c_driver = {
	.driver = {
		.name  = "wau8822",
		.owner = THIS_MODULE,
	},
	.probe    = wau8822_i2c_probe,
	.remove   = wau8822_i2c_remove,
	.id_table = wau8822_i2c_table,
};

static int __init wau8822_modinit(void)
{
	int ret = 0;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	pr_info(CODEC_NAME ": module init\n");
	ret = i2c_add_driver(&wau8822_i2c_driver);
	if (ret != 0)
		pr_err(CODEC_NAME
		       ": Failed to register wau8822 I2C driver:%d\n",
		       ret);
#endif

	return ret;
}
module_init(wau8822_modinit);

static void __exit wau8822_modexit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	pr_info(CODEC_NAME  ": module exit\n");
	i2c_del_driver(&wau8822_i2c_driver);
#endif
}
module_exit(wau8822_modexit);

MODULE_DESCRIPTION("ASoC wau8822 driver");
MODULE_LICENSE("GPL");

