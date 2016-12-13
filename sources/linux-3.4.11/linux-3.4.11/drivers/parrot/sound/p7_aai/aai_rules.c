
#include "aai.h"
#include "aai_hw.h"
#include "aai_rules.h"
#include "aai_clk.h"

struct aai_rule_t {
	int		id;
	uint32_t	addr;
	uint32_t	shift;
	uint32_t	mask;
	uint32_t	value;
	int		excl[NB_EXCL_RULES];
	int		active;
	/* callback to start clock */
	int		(*apply)(struct card_data_t *aai);
	/* callback to stop clock  */
	int		(*remove)(struct card_data_t *aai);
};

/*
 * Callbacks to enable/ disable clocks
 */
static int enable_pcm1(struct card_data_t *aai)
{
	unsigned int err;

	err = clk_prepare_enable(aai->clk[pcm1_clk]);
	if (err) {
		dev_err(aai->dev, "failed to enable clock.\n");
		goto exit;
	}

exit:
	return err;
}

static int disable_pcm1(struct card_data_t *aai)
{
	aai_release_clk(aai, pcm1_clk);
	return 0;
}

static int enable_pcm2(struct card_data_t *aai)
{
	unsigned int err;
	err = clk_prepare_enable(aai->clk[pcm2_clk]);
	if (err) {
		dev_err(aai->dev, "failed to enable clock.\n");
		goto exit;
	}

exit:
	return err;
}

static int disable_pcm2(struct card_data_t *aai)
{
	aai_release_clk(aai, pcm2_clk);
	return 0;
}

static int enable_i2s(struct card_data_t *aai)
{
	/* aai_set_clk(aai, i2s_clk, AAI_NOMINAL_RATE); */
	return 0;
}

/*static int disable_i2s(struct card_data_t *aai)
{
	aai_release_clk(aai, i2s_clk);
	return 0;
}*/

static int enable_spdif(struct card_data_t *aai)
{
	unsigned int err;
	err = clk_prepare_enable(aai->clk[spdif_clk]);
	if (err) {
		dev_err(aai->dev, "failed to enable spdif clock.\n");
		return -EINVAL;
	}

	/* XXX release reset */
	/*aai_writereg(aai, 0, AAI_GBC_RESET, 0);*/

	return 0;
}

static int disable_spdif(struct card_data_t *aai)
{
	aai_release_clk(aai, spdif_clk);
	return 0;
}

static int enable_tdm(struct card_data_t *aai)
{
	uint32_t cfg;

	/*
	 * By default tdm is quad
	 * the rule set if quad or octo
	 * this generic function is appliable for both case
	 * tdm in and tdm out will the same configuration
	 */
	cfg  = aai_readreg(aai, AAI_AAICFG_TDM_IN) & 0x17f;
	cfg |= 1 << 31;

	aai_writereg(aai, cfg, AAI_AAICFG_TDM_IN, 0);
	cfg |= (aai_readreg(aai, AAI_AAICFG_TDM_OUT) & 0xfffffe80);
	aai_writereg(aai, cfg, AAI_AAICFG_TDM_OUT, 0);
	return 0;
}

static int disable_tdm(struct card_data_t *aai)
{
	uint32_t cfg;
	cfg = 0xeffffff;
	cfg &= aai_readreg(aai, AAI_AAICFG_TDM_IN);
	aai_writereg(aai, cfg, AAI_AAICFG_TDM_IN, 0);

	cfg = 0xeffffff;
	cfg &= aai_readreg(aai, AAI_AAICFG_TDM_OUT);
	aai_writereg(aai, cfg, AAI_AAICFG_TDM_OUT, 0);
	return 0;
}

static struct aai_rule_t aai_rules[] = {
	[RULE_VOICE_SPEAKER_8K] = {
		.id = RULE_VOICE_SPEAKER_8K,
		.addr = AAI_AAICFG_VOICE_8K,
		.shift = 1,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICE_SPEAKER_16K, RULE_END},
	},

	[RULE_VOICE_SPEAKER_16K] = {
		.id = RULE_VOICE_SPEAKER_16K,
		.addr = AAI_AAICFG_VOICE_8K,
		.shift = 1,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICE_SPEAKER_8K, RULE_END},
	},

	[RULE_VOICE_PHONE_8K] = {
		.id = RULE_VOICE_PHONE_8K,
		.addr = AAI_AAICFG_VOICE_8K,
		.shift = 0,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICE_PHONE_16K, RULE_END},
	},

	[RULE_VOICE_PHONE_16K] = {
		.id = RULE_VOICE_PHONE_16K,
		.addr = AAI_AAICFG_VOICE_8K,
		.shift = 0,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICE_PHONE_8K, RULE_END},
	},

	[RULE_LOOPBACK_0] = {
		.id = RULE_LOOPBACK_0,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 2,
		.mask = 1,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK_1] = {
		.id = RULE_LOOPBACK_1,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 10,
		.mask = 1,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK_2] = {
		.id = RULE_LOOPBACK_2,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 18,
		.mask = 1,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK_3] = {
		.id = RULE_LOOPBACK_3,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 26,
		.mask = 1,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK0_I2S0] = {
		.id = RULE_LOOPBACK0_I2S0,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 0,
		.mask = 3,
		.value = 0,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK0_I2S1] = {
		.id = RULE_LOOPBACK0_I2S1,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 0,
		.mask = 3,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK0_I2S2] = {
		.id = RULE_LOOPBACK0_I2S2,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 0,
		.mask = 3,
		.value = 2,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK0_I2S3] = {
		.id = RULE_LOOPBACK0_I2S3,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 0,
		.mask = 3,
		.value = 3,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK1_I2S0] = {
		.id = RULE_LOOPBACK1_I2S0,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 8,
		.mask = 3,
		.value = 0,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK1_I2S1] = {
		.id = RULE_LOOPBACK1_I2S1,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 8,
		.mask = 3,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK1_I2S2] = {
		.id = RULE_LOOPBACK1_I2S2,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 8,
		.mask = 3,
		.value = 2,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK1_I2S3] = {
		.id = RULE_LOOPBACK1_I2S3,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 8,
		.mask = 3,
		.value = 3,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK2_I2S0] = {
		.id = RULE_LOOPBACK2_I2S0,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 16,
		.mask = 3,
		.value = 0,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK2_I2S1] = {
		.id = RULE_LOOPBACK2_I2S1,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 16,
		.mask = 3,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK2_I2S2] = {
		.id = RULE_LOOPBACK2_I2S2,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 16,
		.mask = 3,
		.value = 2,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK2_I2S3] = {
		.id = RULE_LOOPBACK2_I2S3,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 16,
		.mask = 3,
		.value = 3,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK3_I2S0] = {
		.id = RULE_LOOPBACK3_I2S0,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 24,
		.mask = 3,
		.value = 0,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK3_I2S1] = {
		.id = RULE_LOOPBACK3_I2S1,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 24,
		.mask = 3,
		.value = 1,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK3_I2S2] = {
		.id = RULE_LOOPBACK3_I2S2,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 24,
		.mask = 3,
		.value = 2,
		.excl = {RULE_END},
	},

	[RULE_LOOPBACK3_I2S3] = {
		.id = RULE_LOOPBACK3_I2S3,
		.addr = AAI_AAICFG_MIC_MUX,
		.shift = 24,
		.mask = 3,
		.value = 3,
		.excl = {RULE_END},
	},

	[RULE_VOICEL0_INPUT_8K] = {
		.id = RULE_VOICEL0_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(0),
		.shift = 5,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICEL0_INPUT_16K, RULE_END},
	},

	[RULE_VOICER0_INPUT_8K] = {
		.id = RULE_VOICER0_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(0),
		.shift = 13,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICER0_INPUT_16K, RULE_END},
	},

	[RULE_VOICEL0_INPUT_16K] = {
		.id = RULE_VOICEL0_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(0),
		.shift = 5,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICEL0_INPUT_8K, RULE_END},
	},

	[RULE_VOICER0_INPUT_16K] = {
		.id = RULE_VOICER0_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(0),
		.shift = 13,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICER0_INPUT_8K, RULE_END},
	},

	[RULE_VOICEL1_INPUT_8K] = {
		.id = RULE_VOICEL1_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(1),
		.shift = 5,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICEL1_INPUT_16K, RULE_END},
	},

	[RULE_VOICER1_INPUT_8K] = {
		.id = RULE_VOICER1_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(1),
		.shift = 13,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICER1_INPUT_16K, RULE_END},
	},

	[RULE_VOICEL1_INPUT_16K] = {
		.id = RULE_VOICEL1_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(1),
		.shift = 5,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICEL1_INPUT_8K, RULE_END},
	},

	[RULE_VOICER1_INPUT_16K] = {
		.id = RULE_VOICER1_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(1),
		.shift = 13,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICER1_INPUT_8K, RULE_END},
	},

	[RULE_VOICEL2_INPUT_8K] = {
		.id = RULE_VOICEL2_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(2),
		.shift = 5,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICEL2_INPUT_16K, RULE_END},
	},

	[RULE_VOICER2_INPUT_8K] = {
		.id = RULE_VOICER2_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(2),
		.shift = 13,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICER2_INPUT_16K, RULE_END},
	},

	[RULE_VOICEL2_INPUT_16K] = {
		.id = RULE_VOICEL2_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(2),
		.shift = 5,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICEL2_INPUT_8K, RULE_END},
	},

	[RULE_VOICER2_INPUT_16K] = {
		.id = RULE_VOICER2_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(2),
		.shift = 13,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICER2_INPUT_8K, RULE_END},
	},

	[RULE_VOICEL3_INPUT_8K] = {
		.id = RULE_VOICEL3_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(3),
		.shift = 5,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICEL3_INPUT_16K, RULE_END},
	},

	[RULE_VOICER3_INPUT_8K] = {
		.id = RULE_VOICER3_INPUT_8K,
		.addr = AAI_AAICFG_VOICE(3),
		.shift = 13,
		.mask = 1,
		.value = 1,
		.excl = {RULE_VOICER3_INPUT_16K, RULE_END},
	},

	[RULE_VOICEL3_INPUT_16K] = {
		.id = RULE_VOICEL3_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(3),
		.shift = 5,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICEL3_INPUT_8K, RULE_END},
	},

	[RULE_VOICER3_INPUT_16K] = {
		.id = RULE_VOICER3_INPUT_16K,
		.addr = AAI_AAICFG_VOICE(3),
		.shift = 13,
		.mask = 1,
		.value = 0,
		.excl = {RULE_VOICER3_INPUT_8K, RULE_END},
	},

	[RULE_PCM0_MASTER] = {
		.id = RULE_PCM0_MASTER,
		.apply = &enable_pcm1,
		.remove = &disable_pcm1,
	},

	[RULE_PCM1_MASTER] = {
		.id = RULE_PCM1_MASTER,
		.apply = &enable_pcm2,
		.remove = &disable_pcm2,
	},

	[RULE_PCM0_8K] = {
		.id = RULE_PCM0_8K,
		.addr = AAI_GBC_P1FRAME,
		.shift = 0,
		.mask = 0xff,
		.value = 0x1e,
		.excl = {RULE_PCM0_16K, RULE_END},
	},

	[RULE_PCM0_16K] = {
		.id = RULE_PCM0_16K,
		.addr = AAI_GBC_P1FRAME,
		.shift = 0,
		.mask = 0xff,
		.value = 0x0e,
		.excl = {RULE_PCM0_8K, RULE_END},
	},

	[RULE_PCM1_8K] = {
		.id = RULE_PCM1_8K,
		.addr = AAI_GBC_P2FRAME,
		.shift = 0,
		.mask = 0xff,
		.value = 0x1e,
		.apply = &enable_pcm2,
		.remove = &disable_pcm2,
		.excl = {RULE_PCM1_16K, RULE_END},
	},

	[RULE_PCM1_16K] = {
		.id = RULE_PCM1_16K,
		.addr = AAI_GBC_P2FRAME,
		.shift = 0,
		.mask = 0x1e,
		.value = 0x0e,
		.apply = &enable_pcm2,
		.remove = &disable_pcm2,
		.excl = {RULE_PCM1_8K, RULE_END},
	},

	[RULE_I2S_96K] = {
		.id = RULE_I2S_96K,
		.addr = AAI_GBC_MCLK,
		.shift = 0,
		.mask = 0xff,
		.value = 0x0e,
		.apply = &enable_i2s,
		.excl = {RULE_I2S_48K, RULE_END},
	},

	[RULE_I2S_48K] = {
		.id = RULE_I2S_48K,
		.addr = AAI_GBC_MCLK,
		.shift = 0,
		.mask = 0xff,
		.value = 0x1e,
		.apply = &enable_i2s,
		.excl = {RULE_I2S_96K, RULE_END},
	},

	[RULE_TDM_OUT_OCTO] = {
		.id = RULE_TDM_OUT_OCTO,
		.addr = AAI_AAICFG_TDM_IN,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x40,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN_OCTO] = {
		.id = RULE_TDM_IN_OCTO,
		.addr = AAI_AAICFG_TDM_IN,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x40,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_OUT_QUAD] = {
		.id = RULE_TDM_OUT_QUAD,
		.addr = AAI_AAICFG_TDM_OUT,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x20,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN_QUAD] = {
		.id = RULE_TDM_IN_QUAD,
		.addr = AAI_AAICFG_TDM_IN,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x20,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_OUT_ST] = {
		.id = RULE_TDM_OUT_QUAD,
		.addr = AAI_AAICFG_TDM_OUT,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x10,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN_ST] = {
		.id = RULE_TDM_IN_QUAD,
		.addr = AAI_AAICFG_TDM_IN,
		.shift = 0,
		.mask = 0x7f,
		.value = 0x10,
		.apply = &enable_tdm,
		.remove = &disable_tdm,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN0] = {
		.id = RULE_TDM_IN0,
		.addr = AAI_AAICFG_MUS(0),
		.shift = 12,
		.mask = 0x7,
		.value = 0x4,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN1] = {
		.id = RULE_TDM_IN1,
		.addr = AAI_AAICFG_MUS(1),
		.shift = 12,
		.mask = 0x7,
		.value = 0x5,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN2] = {
		.id = RULE_TDM_IN2,
		.addr = AAI_AAICFG_MUS(2),
		.shift = 12,
		.mask = 0x7,
		.value = 0x6,
		.excl = {RULE_END},
	},

	[RULE_TDM_IN3] = {
		.id = RULE_TDM_IN3,
		.addr = AAI_AAICFG_MUS(3),
		.shift = 12,
		.mask = 0x7,
		.value = 0x7,
		.excl = {RULE_END},
	},

	[RULE_SPDIF_OUT] = {
		.id = RULE_SPDIF_OUT,
		.addr = AAI_AAICFG_SPDIFTX,
		.shift = 4,
		.mask = 1,
		.value = 1,
		.apply = &enable_spdif,
		.remove = &disable_spdif,
		.excl = {RULE_END},
	},

	[RULE_SPDIF_IN0] = {
		.id = RULE_SPDIF_IN0,
		.addr = AAI_AAICFG_MUS(0),
		.shift = 16,
		.mask = 0x7,
		.value = 0x7,
		.apply = &enable_spdif,
		.remove = &disable_spdif,
		.excl = {RULE_I2S_IN0, RULE_END},
	},

	[RULE_SPDIF_IN1] = {
		.id = RULE_SPDIF_IN1,
		.addr = AAI_AAICFG_MUS(1),
		.shift = 16,
		.mask = 0x7,
		.value = 0x7,
		.apply = &enable_spdif,
		.remove = &disable_spdif,
		.excl = {RULE_I2S_IN1, RULE_END},
	},

	[RULE_SPDIF_IN2] = {
		.id = RULE_SPDIF_IN2,
		.addr = AAI_AAICFG_MUS(2),
		.shift = 16,
		.mask = 0x7,
		.value = 0x7,
		.apply = &enable_spdif,
		.remove = &disable_spdif,
		.excl = {RULE_I2S_IN2, RULE_END},
	},

	[RULE_SPDIF_IN3] = {
		.id = RULE_SPDIF_IN3,
		.addr = AAI_AAICFG_MUS(3),
		.shift = 16,
		.mask = 0x7,
		.value = 0x7,
		.apply = &enable_spdif,
		.remove = &disable_spdif,
		.excl = {RULE_I2S_IN3, RULE_END},
	},

	[RULE_I2S_IN0] = {
		.id = RULE_I2S_IN0,
		.addr = AAI_AAICFG_MUS(0),
		.shift = 16,
		.mask = 0x7,
		.value = 0,
		.excl = {RULE_SPDIF_IN0, RULE_END},
	},

	[RULE_I2S_IN1] = {
		.id = RULE_I2S_IN1,
		.addr = AAI_AAICFG_MUS(1),
		.shift = 16,
		.mask = 0x7,
		.value = 0,
		.excl = {RULE_SPDIF_IN1, RULE_END},
	},

	[RULE_I2S_IN2] = {
		.id = RULE_I2S_IN2,
		.addr = AAI_AAICFG_MUS(2),
		.shift = 16,
		.mask = 0x7,
		.value = 0,
		.excl = {RULE_SPDIF_IN2, RULE_END},
	},

	[RULE_I2S_IN3] = {
		.id = RULE_I2S_IN3,
		.addr = AAI_AAICFG_MUS(3),
		.shift = 16,
		.mask = 0x7,
		.value = 0,
		.excl = {RULE_SPDIF_IN3, RULE_END},
	},

	[RULE_END] = {
		.id = RULE_END,
	}
};

static int aai_rule_is_active(struct card_data_t *aai, int ruleid)
{
	struct aai_rule_t *rule = &aai_rules[ruleid];
	return rule->active;
}

int aai_rule_apply(struct card_data_t *aai, int ruleid)
{
	int err = 0;
	uint32_t reg;
	int id;
	int apply = 1;
	struct aai_rule_t *excl_rule = NULL;
	struct aai_rule_t *rule = &aai_rules[ruleid];
	unsigned long flags = 0;

	if (ruleid >= RULE_END)
		goto exit;

	if (aai_rule_is_active(aai, ruleid))
		goto exit;

	spin_lock_irqsave(&aai->hwlock, flags);

	/* check exclusion rules */
	for (id = 0; id < NB_EXCL_RULES; id++) {
		excl_rule = &aai_rules[rule->excl[id]];

		if (excl_rule->id == RULE_END)
			break;

		if (excl_rule->active) {
			apply = 0;
			break;
		}
	}

	/* apply rule */
	if (apply) {
		if (rule->addr) {
			reg = aai_readreg(aai, rule->addr);
			reg &= ~(rule->mask << rule->shift);
			reg |= (rule->value & rule->mask) << rule->shift;
			aai_writereg(aai, reg, rule->addr, 0);
		}

		/* mark rule as active */
		rule->active = 1;
	}

	spin_unlock_irqrestore(&aai->hwlock, flags);

exit:
	if (apply) {
		/* if a callback is declared, call it */
		if (rule->apply)
			rule->apply(aai);
	}

	return err;
};

int aai_rule_remove(struct card_data_t *aai, int ruleid)
{
	int err = 0;
	struct aai_rule_t *rule = &aai_rules[ruleid];
	unsigned long flags = 0;

	if (!aai_rule_is_active(aai, ruleid))
		goto exit;

	spin_lock_irqsave(&aai->hwlock, flags);

	/* mark rule as inactive, do not change register values */
	rule->active = 0;

	spin_unlock_irqrestore(&aai->hwlock, flags);

exit:
	return err;
}

int aai_hw_params_voice(struct card_data_t *aai,
			struct aai_device_t *chan,
			struct snd_pcm_hw_params *hw_params)
{
	int ret;
	int rate = params_rate(hw_params);

	if (rate == 8000)
		ret = aai_rule_apply(aai, RULE_VOICE_SPEAKER_8K);
	else if (rate == 16000)
		ret = aai_rule_apply(aai, RULE_VOICE_SPEAKER_16K);
	else
		ret = -EINVAL;

	return ret;
}

int aai_close_voice(struct card_data_t *aai,
		    struct aai_device_t *chan)
{
	int ret = 0;

	if (aai_rule_is_active(aai, RULE_VOICE_SPEAKER_8K))
		ret |= aai_rule_remove(aai, RULE_VOICE_SPEAKER_8K);

	if (aai_rule_is_active(aai, RULE_VOICE_SPEAKER_16K))
		ret |= aai_rule_remove(aai, RULE_VOICE_SPEAKER_16K);

	return ret;
}

int aai_hw_params_pcm0(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params)
{
	int ret;
	int rate = params_rate(hw_params);

	if (rate == 8000) {
		ret = aai_rule_apply(aai, RULE_PCM0_8K);
		ret |= aai_rule_apply(aai, RULE_VOICE_PHONE_8K);
	} else if (rate == 16000) {
		ret = aai_rule_apply(aai, RULE_PCM0_16K);
		ret |= aai_rule_apply(aai, RULE_VOICE_PHONE_16K);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int aai_hw_params_pcm1(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params)
{
	int ret;
	int rate = params_rate(hw_params);
	uint32_t reg;

	/* set rate */
	if (rate == 8000) {
		ret = aai_rule_apply(aai, RULE_PCM1_8K);
		ret |= aai_rule_apply(aai, RULE_VOICE_PHONE_8K);
	} else if (rate == 16000) {
		ret = aai_rule_apply(aai, RULE_PCM1_16K);
		ret |= aai_rule_apply(aai, RULE_VOICE_PHONE_16K);
	} else {
		ret = -EINVAL;
	}

	/*
	 * pcm1 is on the same link than pcm0,
	 * run dual must be activated.
	 * start-two set to 0xf
	 */
	reg = aai_readreg(aai, AAI_AAICFG_PCM(0));

	reg |= 1 << 27 | 0xf;
	aai_writereg(aai, reg, AAI_AAICFG_PCM(0), 0);

	return ret;
}

int aai_close_pcm(struct card_data_t *aai,
		  struct aai_device_t *chan)
{
	int ret = 0;
	uint32_t reg;

	/* reset rate */
	if (aai_rule_is_active(aai, RULE_PCM0_8K)) {
		ret |= aai_rule_remove(aai, RULE_PCM0_8K);
		ret |= aai_rule_remove(aai, RULE_VOICE_PHONE_8K);
	}

	if (aai_rule_is_active(aai, RULE_PCM0_16K)) {
		ret |= aai_rule_remove(aai, RULE_PCM0_16K);
		ret |= aai_rule_remove(aai, RULE_VOICE_PHONE_16K);
	}

	if (aai_rule_is_active(aai, RULE_PCM1_8K)) {
		ret |= aai_rule_remove(aai, RULE_PCM1_8K);
		ret |= aai_rule_remove(aai, RULE_VOICE_PHONE_8K);
	}

	if (aai_rule_is_active(aai, RULE_PCM1_16K)) {
		ret |= aai_rule_remove(aai, RULE_PCM1_16K);
		ret |= aai_rule_remove(aai, RULE_VOICE_PHONE_16K);
	}

	/* disable run dual mode & start-two*/
	reg = aai_readreg(aai, AAI_AAICFG_PCM(0));
	reg &= ~(1 << 27);
	reg &= ~0xff;
	aai_writereg(aai, reg, AAI_AAICFG_PCM(0), 0);

	return ret;
}
