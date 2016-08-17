
#include "aai.h"
#include "aai_hw.h"
#include "aai_clk.h"

struct aai_rate_cfg_t {
	int     rate;
	int     freq;
	int     mclk_div;
	int     mframe_div;
};

struct aai_rate_cfg_t aai_rate_tab[] = {
	{
		.rate       = 44100,
		.freq       = 858000000,
		.mclk_div   = 0x24,
		.mframe_div = 0xff,
	},
	{
		.rate       = 48000,
		.freq       = 786500000,
		.mclk_div   = 0x3e,
		.mframe_div = 0x7f,
	},
	{
		.rate       = 88200,
		.freq       = 858000000,
		.mclk_div   = 0x11,
		.mframe_div = 0xff,
	},
	{
		.rate       = 96000,
		.freq       = 786500000,
		.mclk_div   = 0xe,
		.mframe_div = 0xff,
	},
};

#define AAI_RATE_NB     (sizeof(aai_rate_tab) / sizeof(struct aai_rate_cfg_t))

int aai_get_clk(struct card_data_t *aai, enum aai_clk num)
{
	int rate = -1;
	struct aai_rate_cfg_t *rate_cfg = NULL;
	int freq;
	int mclk_div;
	int mframe_div;
	int i;

	if (!aai->clk[num])
		goto exit;

	freq = clk_get_rate(aai->clk[num]);
	mclk_div = aai_readreg(aai, AAI_GBC_MCLK);
	mframe_div = aai_readreg(aai, AAI_GBC_MFRAME);

	for (i = 0; i < AAI_RATE_NB; i++) {
		rate_cfg = &aai_rate_tab[i];
		if (rate_cfg->freq       == freq &&
		    rate_cfg->mclk_div   == mclk_div &&
		    rate_cfg->mframe_div == mframe_div)
			break;
	}

	if (i == AAI_RATE_NB)
		goto exit;

	rate = rate_cfg->rate;

exit:
	return rate;
}

void aai_set_clk(struct card_data_t *aai, enum aai_clk num, int rate)
{
	struct aai_rate_cfg_t *rate_cfg = NULL;
	int i;

	dev_dbg(aai->dev, "%s: %d\n", __func__, rate);

	if (!aai->clk)
		goto exit;

	for (i = 0; i < AAI_RATE_NB; i++) {
		rate_cfg = &aai_rate_tab[i];
		if (rate_cfg->rate == rate)
			break;
	}

	if (i == AAI_RATE_NB) {
		dev_err(aai->dev, "%s: unsupported rate %d\n",
			__func__, rate);
		goto exit;
	}

	clk_set_rate(aai->clk[num],
		     clk_round_rate(aai->clk[num], rate_cfg->freq));
	clk_prepare_enable(aai->clk[num]);

	aai_writereg(aai, rate_cfg->mclk_div, AAI_GBC_MCLK, 0);
	aai_writereg(aai, rate_cfg->mframe_div, AAI_GBC_MFRAME, 0);

exit:
	return;
}

void aai_release_clk(struct card_data_t *aai, enum aai_clk num)
{
	clk_disable(aai->clk[num]);
	clk_put(aai->clk[num]);
}

