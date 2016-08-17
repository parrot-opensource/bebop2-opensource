
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

#include "wm8594.h"

#define CODEC_NAME	"wm8594"

#define wm8594_reset(c) wm8594_write(c, WM8594_RESET, 0)

struct wm8594_priv {
	enum snd_soc_control_type	control_type;
	void				*control_data;
};

/* wm8594 default cfg values */
static const u16 wm8594_reg[WM8594_CACHEREG_NUM] = {
	0x8594,	0x0000,	0x008a,	0x0000,
	0x0000,	0x00c8,	0x00c8, 0x008a,
	0x0000, 0x0000, 0x00c8, 0x00c8,
	0x0000, 0x200a, 0x0000, 0x0000,
	0x00c3, 0x00c3, 0x000c, 0x000c,
	0x000c, 0x000c, 0x000c, 0x000c,
	0x0003, 0x007e, 0x0048, 0x0000,
	0x0000, 0x0008, 0x0000, 0x0088,
	0x0163, 0x0040, 0x0010, 0x0002

};

static const struct reg_default wm8594_reg_defaults[] = {
	{WM8594_DAC1_CTRL1,	0x008e}, /*
					  * DAC1 cross enable
					  * 32 bits i2s format
					  */
	{WM8594_DAC1_CTRL2,	0x001b}, /*
					  * MCLK = 256 * Fs
					  * BLCK = 64  * Fs
					  */
	{WM8594_ADC_CTRL1,	0x200a}, /*
					  * ADC cross enable
					  * 32 bits i2s format
					  */
	{WM8594_DAC2_CTRL1,	0x008e}, /*
					  * DAC2 cross enable
					  * 32 bits i2s format
					  */
	{WM8594_DAC2_CTRL2,	0x001b}, /*
					  * MCLK = 256 * Fs
					  * BLCK = 64  * Fs
					  */
	{WM8594_INPUT_CTRL1,	0x09a9},
	{WM8594_INPUT_CTRL2,	0x0a9a}, /*
					  * connect PGA1 and PGA2
					  * to DAC1
					  */
	{WM8594_INPUT_CTRL3,	0x0080}, /* connect ADC to VIN1 */
};

static inline unsigned int wm8594_read_reg_cache(struct snd_soc_codec *codec,
						 unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8594_CACHEREG_NUM)
		return -1;
	return cache[reg];
}

static inline void wm8594_write_reg_cache(struct snd_soc_codec *codec,
					  u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8594_CACHEREG_NUM)
		return;
	cache[reg] = value;
}

static int wm8594_write(struct snd_soc_codec *codec,
			unsigned int reg,
			unsigned int value)
{
	int ret = 0;
	wm8594_write_reg_cache(codec, reg, value);

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

static const char * const wm8594_adc_input_sel_text[] = {
	"VIN1", "VIN2", "VIN3", "VIN4", "VIN5"
};

static const char * const wm8594_pga_sel_text[] = {
	"No Input", "VIN1", "VIN2", "VIN3", "VIN4", "VIN5", "DAC1", "DAC2"
};

static const struct soc_enum wm8594_adc_input_sel_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wm8594_adc_input_sel_text),
			    wm8594_adc_input_sel_text);

static const struct soc_enum wm8594_pga_sel_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wm8594_pga_sel_text),
			    wm8594_pga_sel_text);

static int wm8594_vol_update(struct snd_kcontrol *kcontrol,
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


static int wm8594_adc_input_get_switch(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 lsel = wm8594_read_reg_cache(codec, WM8594_INPUT_CTRL3) & 0x000f;
	u16 rsel = wm8594_read_reg_cache(codec, WM8594_INPUT_CTRL3) & 0x00f0;

	/*
	 * here we expect that no inversion has been made
	 * left input  <-> left adc
	 * right input <-> right adc
	 */
	switch (lsel) {
	case 0x0:
		/* VIN1L */
		ucontrol->value.integer.value[0] = 0;
		break;
	case 0x1:
		/* VIN2L */
		ucontrol->value.integer.value[0] = 1;
		break;
	case 0x2:
		/* VIN3L */
		ucontrol->value.integer.value[0] = 2;
		break;
	case 0x3:
		/* VIN4L */
		ucontrol->value.integer.value[0] = 3;
		break;
	case 0x4:
		/* VIN5L */
		ucontrol->value.integer.value[0] = 4;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		break;
	};

	switch (rsel) {
	case 0x80:
		/* VIN1R */
		ucontrol->value.integer.value[0] = 0;
		break;
	case 0x90:
		/* VIN2R */
		ucontrol->value.integer.value[0] = 1;
		break;
	case 0xa0:
		/* VIN3R */
		ucontrol->value.integer.value[0] = 2;
		break;
	case 0xb0:
		/* VIN4R */
		ucontrol->value.integer.value[0] = 3;
		break;
	case 0xc0:
		/* VIN5R */
		ucontrol->value.integer.value[0] = 4;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		break;
	};

	return ret;
}

static int wm8594_adc_input_set_switch(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 sel = wm8594_read_reg_cache(codec, WM8594_INPUT_CTRL3) & ~0x00ff;

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		/* VIN1 */
		sel |= 0x80;
		break;
	case 1:
		/* VIN2 */
		sel |= 0x91;
		break;
	case 2:
		/* VIN3 */
		sel |= 0xa2;
		break;
	case 3:
		/* VIN4 */
		sel |= 0xb3;
		break;
	case 4:
		/* VIN5 */
		sel |= 0xc4;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		goto error;
		break;
	}

	wm8594_write(codec, WM8594_INPUT_CTRL3, sel);

error:
	return ret;
}

static int wm8594_pga_sel_get_switch(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol,
				     u16 reg_left, u16 shift_left,
				     u16 reg_right, u16 shift_right)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 lsel = (wm8594_read_reg_cache(codec, reg_left)  >> shift_left)
		   & 0xf;
	u16 rsel = (wm8594_read_reg_cache(codec, reg_right) >> shift_right)
		   & 0xf;

	/*
	 * here we expect that no inversion has been made
	 */
	switch (lsel) {
	case 0x0:
		/* no input */
		ucontrol->value.integer.value[0] = 0;
		break;
	case 0x1:
		/* VIN1L */
		ucontrol->value.integer.value[0] = 1;
		break;
	case 0x2:
		/* VIN2L */
		ucontrol->value.integer.value[0] = 2;
		break;
	case 0x3:
		/* VIN3L */
		ucontrol->value.integer.value[0] = 3;
		break;
	case 0x4:
		/* VIN4L */
		ucontrol->value.integer.value[0] = 4;
		break;
	case 0x5:
		/* VIN5L */
		ucontrol->value.integer.value[0] = 5;
		break;
	case 0x9:
		/* DAC1L */
		ucontrol->value.integer.value[0] = 6;
		break;
	case 0xb:
		/* DAC2L */
		ucontrol->value.integer.value[0] = 7;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		break;
	};

	switch (rsel) {
	case 0x0:
		/* no input */
		ucontrol->value.integer.value[0] = 0;
		break;
	case 0x1:
		/* VIN1R */
		ucontrol->value.integer.value[0] = 1;
		break;
	case 0x2:
		/* VIN2R */
		ucontrol->value.integer.value[0] = 2;
		break;
	case 0x3:
		/* VIN3R */
		ucontrol->value.integer.value[0] = 3;
		break;
	case 0x4:
		/* VIN4R */
		ucontrol->value.integer.value[0] = 4;
		break;
	case 0x5:
		/* VIN5R */
		ucontrol->value.integer.value[0] = 5;
		break;
	case 0xa:
		/* DAC1R */
		ucontrol->value.integer.value[0] = 6;
		break;
	case 0xc:
		/* DAC2R */
		ucontrol->value.integer.value[0] = 7;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		break;
	};

	return ret;
}

static int wm8594_pga_sel_set_switch(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol,
				     u16 reg_left, u16 shift_left,
				     u16 reg_right, u16 shift_right)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 lsel = wm8594_read_reg_cache(codec, reg_left)
		   & ~(0xf << shift_left);
	u16 rsel = wm8594_read_reg_cache(codec, reg_right)
		   & ~(0xf << shift_right);

	if (reg_left == reg_right)
		lsel &= ~(0xf << shift_right);

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		/* no input */
		break;
	case 1:
		/* VIN1 */
		if (reg_left == reg_right) {
			lsel |= 0x1 << shift_left | 0x1 << shift_right;
		} else {
			lsel |= 0x1 << shift_left;
			rsel |= 0x1 << shift_right;
		}
		break;
	case 2:
		/* VIN2 */
		if (reg_left == reg_right) {
			lsel |= 0x2 << shift_left | 0x2 << shift_right;
		} else {
			lsel |= 0x2 << shift_left;
			rsel |= 0x2 << shift_right;
		}
		break;
	case 3:
		/* VIN3 */
		if (reg_left == reg_right) {
			lsel |= 0x3 << shift_left | 0x3 << shift_right;
		} else {
			lsel |= 0x3 << shift_left;
			rsel |= 0x3 << shift_right;
		}
		break;
	case 4:
		/* VIN4 */
		if (reg_left == reg_right) {
			lsel |= 0x4 << shift_left | 0x4 << shift_right;
		} else {
			lsel |= 0x4 << shift_left;
			rsel |= 0x4 << shift_right;
		}
		break;
	case 5:
		/* VIN5 */
		if (reg_left == reg_right) {
			lsel |= 0x5 << shift_left | 0x5 << shift_right;
		} else {
			lsel |= 0x5 << shift_left;
			rsel |= 0x5 << shift_right;
		}
		break;
	case 6:
		/* DAC1 */
		if (reg_left == reg_right) {
			lsel |= 0x9 << shift_left | 0xa << shift_right;
		} else {
			lsel |= 0x9 << shift_left;
			rsel |= 0xa << shift_right;
		}
		break;
	case 7:
		/* DAC2 */
		if (reg_left == reg_right) {
			lsel |= 0xb << shift_left | 0xc << shift_right;
		} else {
			lsel |= 0xb << shift_left;
			rsel |= 0xc << shift_right;
		}
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "%s: unexpected control value\n", __func__);
		goto error;
		break;
	}

	wm8594_write(codec, reg_left, lsel);
	if (reg_left != reg_right)
		wm8594_write(codec, reg_right, rsel);

error:
	return ret;
}


static int wm8594_pga1_sel_get_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL1, 0,
					 WM8594_INPUT_CTRL1, 4);
}

static int wm8594_pga1_sel_set_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL1, 0,
					 WM8594_INPUT_CTRL1, 4);
}

static int wm8594_pga2_sel_get_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL1, 8,
					 WM8594_INPUT_CTRL2, 0);
}

static int wm8594_pga2_sel_set_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL1, 8,
					 WM8594_INPUT_CTRL2, 0);
}

static int wm8594_pga3_sel_get_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL2, 4,
					 WM8594_INPUT_CTRL2, 8);
}

static int wm8594_pga3_sel_set_switch(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
					 WM8594_INPUT_CTRL2, 4,
					 WM8594_INPUT_CTRL2, 8);
}

static const DECLARE_TLV_DB_SCALE(dac_tlv, -10000, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, -9750, 50, 0);
static const DECLARE_TLV_DB_SCALE(pga_tlv, -7400, 50, 0xa0);

static const struct snd_kcontrol_new wm8594_snd_controls[] = {
	SOC_DOUBLE_R_EXT_TLV("DAC1 Playback Volume",
			     WM8594_DAC1L_VOL, WM8594_DAC1R_VOL, 0, 224, 0,
			     snd_soc_get_volsw, wm8594_vol_update, dac_tlv),
	SOC_SINGLE("DAC1 Mute Switch", WM8594_DAC1_CTRL1, 9, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("DAC2 Playback Volume",
			     WM8594_DAC2L_VOL, WM8594_DAC2R_VOL, 0, 224, 0,
			     snd_soc_get_volsw, wm8594_vol_update, dac_tlv),
	SOC_SINGLE("DAC2 Mute Switch", WM8594_DAC2_CTRL1, 9, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("ADC Capture Volume",
			     WM8594_ADCL_VOL, WM8594_ADCR_VOL, 0, 255, 0,
			     snd_soc_get_volsw, wm8594_vol_update, adc_tlv),

	SOC_DOUBLE_R_EXT_TLV("PGA1 Volume",
			     WM8594_PGA1L_VOL, WM8594_PGA1R_VOL, 0, 160, 1,
			     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("PGA1 Mute Switch", WM8594_PGA_CTRL2, 1, 2, 1, 1),
	SOC_ENUM_EXT("PGA1 Input Selection", wm8594_pga_sel_enum,
		     wm8594_pga1_sel_get_switch, wm8594_pga1_sel_set_switch),

	SOC_DOUBLE_R_EXT_TLV("PGA2 Volume",
			     WM8594_PGA2L_VOL, WM8594_PGA2R_VOL, 0, 160, 1,
			     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("PGA2 Mute Switch", WM8594_PGA_CTRL2, 3, 4, 1, 1),
	SOC_ENUM_EXT("PGA2 Input Selection", wm8594_pga_sel_enum,
		     wm8594_pga2_sel_get_switch, wm8594_pga2_sel_set_switch),

	SOC_DOUBLE_R_EXT_TLV("PGA3 Volume",
			     WM8594_PGA3L_VOL, WM8594_PGA3R_VOL, 0, 160, 1,
			     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("PGA3 Mute Switch", WM8594_PGA_CTRL2, 5, 4, 1, 1),
	SOC_ENUM_EXT("PGA3 Input Selection", wm8594_pga_sel_enum,
		     wm8594_pga3_sel_get_switch, wm8594_pga3_sel_set_switch),

	SOC_SINGLE("PGA Mute All Switch", WM8594_PGA_CTRL2, 0, 1, 1),

	SOC_SINGLE("ADC Gain Control", WM8594_INPUT_CTRL3, 8, 3, 0),
	SOC_ENUM_EXT("ADC Input Selection", wm8594_adc_input_sel_enum,
		     wm8594_adc_input_get_switch, wm8594_adc_input_set_switch),
};

static int wm8594_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	u16 dac1_ctrl1 = wm8594_read_reg_cache(codec, WM8594_DAC1_CTRL1) & ~0xc;
	u16 dac2_ctrl1 = wm8594_read_reg_cache(codec, WM8594_DAC2_CTRL1) & ~0xc;
	u16 add = wm8594_read_reg_cache(codec, WM8594_ADD_CTRL1) & ~0x70;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		dac1_ctrl1 |= 0x0004;
		dac2_ctrl1 |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dac1_ctrl1 |= 0x0008;
		dac2_ctrl1 |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dac1_ctrl1 |= 0x000c;
		dac2_ctrl1 |= 0x000c;
		break;
	}

	/* set sample rate */
	switch (params_rate(params)) {
	case SNDRV_PCM_RATE_32000:
		break;
	case SNDRV_PCM_RATE_44100:
		add |= 0x10;
		break;
	case SNDRV_PCM_RATE_88200:
		add |= 0x30;
		break;
	case SNDRV_PCM_RATE_96000:
		add |= 0x40;
		break;
	case SNDRV_PCM_RATE_176400:
		add |= 0x50;
		break;
	case SNDRV_PCM_RATE_192000:
		add |= 0x60;
		break;
	/* 48000 is the default value */
	case SNDRV_PCM_RATE_48000:
	default:
		add |= 0x20;
		break;
	}

	wm8594_write(codec, WM8594_DAC1_CTRL1, dac1_ctrl1);
	wm8594_write(codec, WM8594_DAC2_CTRL1, dac2_ctrl1);
	wm8594_write(codec, WM8594_ADD_CTRL1, add);
	return 0;
}

static int wm8594_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	u16 dac1_ctrl1 = wm8594_read_reg_cache(codec, WM8594_DAC1_CTRL1)
			 & ~0x33;
	u16 dac1_ctrl3 = wm8594_read_reg_cache(codec, WM8594_DAC1_CTRL3) & ~0x1;

	u16 dac2_ctrl1 = wm8594_read_reg_cache(codec, WM8594_DAC2_CTRL1)
			 & ~0x33;
	u16 dac2_ctrl3 = wm8594_read_reg_cache(codec, WM8594_DAC2_CTRL3) & ~0x1;

	u16 adc_ctrl1 = wm8594_read_reg_cache(codec, WM8594_ADC_CTRL1) & ~0x33;
	u16 adc_ctrl3 = wm8594_read_reg_cache(codec, WM8594_ADC_CTRL3) & ~0x1;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dac1_ctrl3 |= 0x0001;
		dac2_ctrl3 |= 0x0001;
		adc_ctrl3 |= 0x0001;
		break;

	case SND_SOC_DAIFMT_CBS_CFS:
		break;

	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dac1_ctrl1 |= 0x0002;
		dac2_ctrl1 |= 0x0002;
		adc_ctrl1 |= 0x0002;
		break;

	case SND_SOC_DAIFMT_RIGHT_J:
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		dac1_ctrl1 |= 0x0001;
		dac2_ctrl1 |= 0x0001;
		adc_ctrl1 |= 0x0001;
		break;

	case SND_SOC_DAIFMT_DSP_A:
		dac1_ctrl1 |= 0x0003;
		dac2_ctrl1 |= 0x0003;
		adc_ctrl1 |= 0x0003;
		break;

	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;

	case SND_SOC_DAIFMT_IB_IF:
		dac1_ctrl1 |= 0x0030;
		dac2_ctrl1 |= 0x0030;
		adc_ctrl1 |= 0x0030;
		break;

	case SND_SOC_DAIFMT_IB_NF:
		dac1_ctrl1 |= 0x0010;
		dac2_ctrl1 |= 0x0010;
		adc_ctrl1 |= 0x0010;
		break;

	case SND_SOC_DAIFMT_NB_IF:
		dac1_ctrl1 |= 0x0020;
		dac2_ctrl1 |= 0x0020;
		adc_ctrl1 |= 0x0020;
		break;

	default:
		return -EINVAL;
	}

	wm8594_write(codec, WM8594_DAC1_CTRL1, dac1_ctrl1);
	wm8594_write(codec, WM8594_DAC1_CTRL3, dac1_ctrl3);

	wm8594_write(codec, WM8594_DAC2_CTRL1, dac2_ctrl1);
	wm8594_write(codec, WM8594_DAC2_CTRL3, dac2_ctrl3);

	wm8594_write(codec, WM8594_ADC_CTRL1, adc_ctrl1);
	wm8594_write(codec, WM8594_ADC_CTRL3, adc_ctrl3);
	return 0;
}

static int wm8594_mute(struct snd_soc_dai *codec_dai, int mute)
{
	return 0;
}

static struct snd_soc_dai_ops wm8594_dai_ops_playback = {
	.hw_params    = wm8594_pcm_hw_params,
	.set_fmt      = wm8594_set_dai_fmt,
	.digital_mute = wm8594_mute,
};

static int wm8594_power_up(struct snd_soc_codec *codec)
{
	u16 bias = 0;
	u16 dac1 = wm8594_read_reg_cache(codec, WM8594_DAC1_CTRL1) & ~0x100;
	u16 dac2 = wm8594_read_reg_cache(codec, WM8594_DAC2_CTRL1) & ~0x100;
	u16 adc = wm8594_read_reg_cache(codec, WM8594_ADC_CTRL1) & ~0x40;

	/*
	 * set up initial biases
	 * SOFT_ST | FAST_EN | POBCTRL | BUFIO_EN
	 */
	bias |= 0x1d;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * enable output drivers to allow the AC coupling capacitors at the
	 * output charge to be precharged to DACVMID
	 * VOUTxL_EN | VOUTxR_EN
	 */
	wm8594_write(codec, WM8594_OUTPUT_CTRL3, 0x1fc0);

	/*
	 * enable DACVMID, 750k selected for pop reduction
	 * VMID_SEL = 10
	 */
	bias |= 0x80;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * wait until DACVMID has fully charged
	 */
	mdelay(1500);

	/*
	 * enable master bias
	 * BIAS_EN
	 */
	bias |= 0x20;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * switch output drivers to use the master bias instead of the
	 * power-up (fast) bias
	 * POBCTRL = 0
	 */
	bias &= ~0x1;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * enable all function required for use:
	 * - enable all inputs
	 * - enable codec
	 * - enable dac1, dac2 and adc
	 */
	wm8594_write(codec, WM8594_INPUT_CTRL4, 0xff);
	wm8594_write(codec, WM8594_ENABLE, 0x1);
	dac1 |= 0x100;
	wm8594_write(codec, WM8594_DAC1_CTRL1, dac1);
	dac2 |= 0x100;
	wm8594_write(codec, WM8594_DAC2_CTRL1, dac2);
	adc |= 0x40;
	wm8594_write(codec, WM8594_ADC_CTRL1, adc);

	/*
	 * unmute PGAs and switch DACVMID to 75k for normal operation
	 */
	bias &= ~0xc0;
	bias |= 0x40;
	wm8594_write(codec, WM8594_PGA_CTRL2, 0x0);

	return 0;
}

static int wm8594_power_down(struct snd_soc_codec *codec)
{
	u16 bias;
	u16 dac1 = wm8594_read_reg_cache(codec, WM8594_DAC1_CTRL1) & ~0x100;
	u16 dac2 = wm8594_read_reg_cache(codec, WM8594_DAC2_CTRL1) & ~0x100;
	u16 adc = wm8594_read_reg_cache(codec, WM8594_ADC_CTRL1) & ~0x40;
	u16 output = wm8594_read_reg_cache(codec, WM8594_OUTPUT_CTRL3) & ~0x40;

	/*
	 * mute all PGAs
	 */
	wm8594_write(codec, WM8594_PGA_CTRL2, 0x7f);

	/*
	 * set biases for power down mode
	 * FAST_EN = 1
	 * VMID_SEL = 01
	 * BIAS_EN = 1
	 * BUFIO_EN = 1
	 * VMIDTOG = 0
	 * SOFT_ST = 1
	 */
	bias = 0x7c;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * switch outputs to fast bias
	 * POBCTRL = 1
	 */
	bias |= 0x2;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * power down
	 */
	wm8594_write(codec, WM8594_DAC1_CTRL1, dac1);
	wm8594_write(codec, WM8594_DAC2_CTRL1, dac2);
	wm8594_write(codec, WM8594_ADC_CTRL1, adc);
	wm8594_write(codec, WM8594_INPUT_CTRL4, 0x00);
	wm8594_write(codec, WM8594_ENABLE, 0x0);

	/*
	 * power down VMID
	 * VMIDSEL = 00
	 */
	bias &= ~0xc0;
	wm8594_write(codec, WM8594_BIAS, bias);

	/*
	 * wait for DACVMID to be discharged
	 */
	mdelay(1500);

	/*
	 * clamp outputs to ground
	 * power down outputs
	 */
	wm8594_write(codec, WM8594_OUTPUT_CTRL3, output);
	wm8594_write(codec, WM8594_OUTPUT_CTRL3, 0);

	/*
	 * disable remaining bias controls
	 */
	bias = 0;
	wm8594_write(codec, WM8594_BIAS, bias);

	return 0;
}

static int wm8594_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
			wm8594_power_up(codec);
		break;

	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_OFF:
		wm8594_power_down(codec);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static int wm8594_probe(struct snd_soc_codec *codec)
{
	int i;
	int ret = 0;
	unsigned int value = 0;
	struct wm8594_priv *wm8594 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "WM8594 Audio Codec");

	codec->control_data = wm8594->control_data;
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, wm8594->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		goto error;
	}

	ret = wm8594_reset(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to reset codec: %d\n", ret);
		goto error;
	}

	/* initialize codec register */
	for (i = 0; i < ARRAY_SIZE(wm8594_reg_defaults); i++) {
		wm8594_write(codec, wm8594_reg_defaults[i].reg,
			     wm8594_reg_defaults[i].def);
	}

	/* power on device */
	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	wm8594_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* update cache */
	for (i = 0; i < WM8594_CACHEREG_NUM; i++) {
		value = snd_soc_read(codec, i);
		wm8594_write_reg_cache(codec, i, value);
	}

	/* init controls */
	snd_soc_add_codec_controls(codec, wm8594_snd_controls,
				   ARRAY_SIZE(wm8594_snd_controls));

error:
	return ret;

}

static int wm8594_remove(struct snd_soc_codec *codec)
{
	wm8594_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8594_suspend(struct snd_soc_codec *codec)
{
	/* we only need to suspend if we are a valid card */
	if (!codec->card)
		return 0;

	wm8594_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8594_resume(struct snd_soc_codec *codec)
{
	int i;
	u16 *cache = codec->reg_cache;

	/* we only need to resume if we are a valid card */
	if (!codec->card)
		return 0;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8594_reg); i++) {
		if (i == WM8594_RESET)
			continue;
		snd_soc_write(codec->control_data, i, cache[i]);
	}

	wm8594_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8594_set_bias_level(codec, codec->dapm.suspend_bias_level);
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_wm8594 = {
	.probe             = wm8594_probe,
	.remove            = wm8594_remove,
	.suspend           = wm8594_suspend,
	.resume            = wm8594_resume,
	.set_bias_level    = wm8594_set_bias_level,
	.reg_cache_size    = sizeof(wm8594_reg),
	.reg_word_size     = sizeof(u16),
	.reg_cache_default = &wm8594_reg,
};

#define WM8594_RATES (SNDRV_PCM_RATE_8000  |	\
		      SNDRV_PCM_RATE_11025 |	\
		      SNDRV_PCM_RATE_16000 |	\
		      SNDRV_PCM_RATE_22050 |	\
		      SNDRV_PCM_RATE_44100 |	\
		      SNDRV_PCM_RATE_48000)

#define WM8594_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |	\
			SNDRV_PCM_FMTBIT_S20_3LE |	\
			SNDRV_PCM_FMTBIT_S24_LE  |	\
			SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver wm8594_dai[] = {
	{
		.name     = "wm8594-hifi",
		.id       = 0,
		.playback = {
			.stream_name  = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates        = WM8594_RATES,
			.formats      = WM8594_FORMATS,
		},
		.capture = {
			.stream_name  = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates        = WM8594_RATES,
			.formats      = WM8594_FORMATS,
		},
		.ops = &wm8594_dai_ops_playback,
	},
};

static int wm8594_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wm8594_priv *wm8594;
	int ret;

	wm8594 = kzalloc(sizeof(struct wm8594_priv), GFP_KERNEL);
	if (wm8594 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, wm8594);
	wm8594->control_data = i2c;
	wm8594->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_wm8594,
				     wm8594_dai, ARRAY_SIZE(wm8594_dai));

	if (ret < 0) {
		kfree(wm8594);
		pr_err(CODEC_NAME  ": %s failed: ret = %d\n", __func__, ret);
	}

	return ret;
}

static int wm8594_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	snd_soc_unregister_codec(&client->dev);
	kfree(codec);
	return 0;
}


/*
 * WM8594 2 wires address is determined by pin 45
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1c
 */
static const struct i2c_device_id wm8594_i2c_table[] = {
	{"wm8594", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, wm8594_i2c_table);

/*  i2c codec control layer */
static struct i2c_driver wm8594_i2c_driver = {
	.driver = {
		.name  = "wm8594",
		.owner = THIS_MODULE,
	},
	.probe    = wm8594_i2c_probe,
	.remove   = wm8594_i2c_remove,
	.id_table = wm8594_i2c_table,
};

static int __init wm8594_modinit(void)
{
	int ret = 0;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	pr_info(CODEC_NAME ": module init\n");
	ret = i2c_add_driver(&wm8594_i2c_driver);
	if (ret != 0)
		pr_err(CODEC_NAME
		       ": Failed to register wm8594 I2C driver:%d\n",
		       ret);
#endif

	return ret;
}
module_init(wm8594_modinit);

static void __exit wm8594_modexit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	pr_info(CODEC_NAME  ": module exit\n");
	i2c_del_driver(&wm8594_i2c_driver);
#endif
}
module_exit(wm8594_modexit);

MODULE_DESCRIPTION("ASoC wm8594 driver");
MODULE_AUTHOR("Adrien Charruel");
MODULE_LICENSE("GPL");

