
#ifndef _AAI_SIGNALS_H_
#define _AAI_SIGNALS_H_

enum aai_signal_t {
	AAI_SIG_MCLK = 0,
	AAI_SIG_MAIN_I2S_FRAME,
	AAI_SIG_HALF_RATE_FRAME,
	AAI_SIG_ADC_BIT_CLOCK,
	AAI_SIG_DAC_BIT_CLOCK,
	AAI_SIG_AUX_BIT_CLOCK,
	AAI_SIG_TDM_OUT_BIT_CLOCK,
	AAI_SIG_OUT_DAC0,
	AAI_SIG_OUT_DAC1,
	AAI_SIG_OUT_DAC2,
	AAI_SIG_OUT_DAC3,
	AAI_SIG_IN_MIC0,
	AAI_SIG_IN_MIC1,
	AAI_SIG_IN_MIC2,
	AAI_SIG_IN_MIC3,
	AAI_SIG_I2S0_IN,
	AAI_SIG_I2S0_FRAME,
	AAI_SIG_I2S1_IN,
	AAI_SIG_I2S1_FRAME,
	AAI_SIG_I2S2_IN,
	AAI_SIG_I2S2_FRAME,
	AAI_SIG_I2S3_IN,
	AAI_SIG_I2S3_FRAME,
	AAI_SIG_TDM_IN,
	AAI_SIG_TDM_OUT,
	AAI_SIG_PCM1_CLK,
	AAI_SIG_PCM1_FRAME,
	AAI_SIG_PCM1_IN,
	AAI_SIG_PCM1_OUT,
	AAI_SIG_PCM2_CLK,
	AAI_SIG_PCM2_FRAME,
	AAI_SIG_PCM2_IN,
	AAI_SIG_PCM2_OUT,
	AAI_SIG_SPDIF_RX,
	AAI_SIG_SPDIF_TX,
	AAI_SIG_TDM_IN_BIT_CLOCK
};

enum pad_direction {
	PAD_IN  = 1 << 0,
	PAD_OUT = 1 << 1
};

struct aai_pad_t {
	int	signal;
	int	index;
	int	direction;
};

struct aai_conf_set {
	int	addr;
	int	shift;
	int	mask;
	int	value;
};

struct aai_platform_data {
	struct aai_pad_t	*pad;
	int			chiprev;
	struct aai_conf_set	*aai_conf;
	char			**device_list;
};

#endif
