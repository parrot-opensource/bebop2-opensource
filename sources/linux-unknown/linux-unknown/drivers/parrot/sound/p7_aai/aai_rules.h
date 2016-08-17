
#ifndef _AAI_RULES_H_
#define _AAI_RULES_H_

#define NB_EXCL_RULES   5

enum aai_rules_t {
	RULE_VOICE_SPEAKER_8K,
	RULE_VOICE_SPEAKER_16K,
	RULE_VOICE_PHONE_8K,
	RULE_VOICE_PHONE_16K,
	RULE_LOOPBACK_0,
	RULE_LOOPBACK_1,
	RULE_LOOPBACK_2,
	RULE_LOOPBACK_3,
	RULE_LOOPBACK0_I2S0,
	RULE_LOOPBACK0_I2S1,
	RULE_LOOPBACK0_I2S2,
	RULE_LOOPBACK0_I2S3,
	RULE_LOOPBACK1_I2S0,
	RULE_LOOPBACK1_I2S1,
	RULE_LOOPBACK1_I2S2,
	RULE_LOOPBACK1_I2S3,
	RULE_LOOPBACK2_I2S0,
	RULE_LOOPBACK2_I2S1,
	RULE_LOOPBACK2_I2S2,
	RULE_LOOPBACK2_I2S3,
	RULE_LOOPBACK3_I2S0,
	RULE_LOOPBACK3_I2S1,
	RULE_LOOPBACK3_I2S2,
	RULE_LOOPBACK3_I2S3,
	RULE_VOICEL0_INPUT_8K,
	RULE_VOICER0_INPUT_8K,
	RULE_VOICEL0_INPUT_16K,
	RULE_VOICER0_INPUT_16K,
	RULE_VOICEL1_INPUT_8K,
	RULE_VOICER1_INPUT_8K,
	RULE_VOICEL1_INPUT_16K,
	RULE_VOICER1_INPUT_16K,
	RULE_VOICEL2_INPUT_8K,
	RULE_VOICER2_INPUT_8K,
	RULE_VOICEL2_INPUT_16K,
	RULE_VOICER2_INPUT_16K,
	RULE_VOICEL3_INPUT_8K,
	RULE_VOICER3_INPUT_8K,
	RULE_VOICEL3_INPUT_16K,
	RULE_VOICER3_INPUT_16K,
	RULE_PCM0_MASTER,
	RULE_PCM1_MASTER,
	RULE_PCM0_8K,
	RULE_PCM0_16K,
	RULE_PCM1_8K,
	RULE_PCM1_16K,
	RULE_I2S_48K,
	RULE_I2S_96K,
	RULE_SPDIF_OUT,
	RULE_SPDIF_IN0,
	RULE_SPDIF_IN1,
	RULE_SPDIF_IN2,
	RULE_SPDIF_IN3,
	RULE_I2S_IN0,
	RULE_I2S_IN1,
	RULE_I2S_IN2,
	RULE_I2S_IN3,
	RULE_TDM_OUT_OCTO,
	RULE_TDM_IN_OCTO,
	RULE_TDM_OUT_QUAD,
	RULE_TDM_IN_QUAD,
	RULE_TDM_OUT_ST,
	RULE_TDM_IN_ST,
	RULE_TDM_IN0,
	RULE_TDM_IN1,
	RULE_TDM_IN2,
	RULE_TDM_IN3,
	RULE_END
};

int aai_rule_apply(struct card_data_t *aai, int ruleid);
int aai_rule_remove(struct card_data_t *aai, int ruleid);

int aai_hw_params_voice(struct card_data_t *aai,
			struct aai_device_t *chan,
			struct snd_pcm_hw_params *hw_params);
int aai_close_voice(struct card_data_t *aai,
		    struct aai_device_t *chan);
int aai_hw_params_mic0(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params);
int aai_close_mic0(struct card_data_t *aai,
		   struct aai_device_t *chan);
int aai_hw_params_mic1(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params);
int aai_close_mic1(struct card_data_t *aai,
		   struct aai_device_t *chan);
int aai_hw_params_pcm0(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params);
int aai_hw_params_pcm1(struct card_data_t *aai,
		       struct aai_device_t *chan,
		       struct snd_pcm_hw_params *hw_params);
int aai_close_pcm(struct card_data_t *aai,
		  struct aai_device_t *chan);

#endif

