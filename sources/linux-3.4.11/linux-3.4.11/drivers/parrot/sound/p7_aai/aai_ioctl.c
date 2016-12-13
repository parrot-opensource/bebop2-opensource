
#include "aai.h"
#include "aai_hw.h"
#include "aai_ioctl.h"
#include "aai_group.h"

/**
 * Get a data from user space
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 * @return error code
 */
static int get_data(void __user *src, void *dst, int size)
{
	int err = 0;
	if (copy_from_user(dst, src, size))
		err = -EFAULT;

	return err;
}

/**
 * Put data to userspace
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 *
 * @return error code
 */
#if 0
static int put_data(void *src, void __user *dst, int size)
{
	int err = 0;
	if (copy_to_user(dst, src, size))
		err = -EFAULT;

	return err;
}
#endif

static int aai_hwdep_set_music_conf(struct card_data_t *aai,
				    unsigned long arg,
				    int musx)
{
	int err = 0;
	struct aai_ioctl_music_conf_t music_conf;

	err = get_data((void __user *)arg, &music_conf,
		       sizeof(struct aai_ioctl_music_conf_t));
	if (!err) {
		if (0 <= music_conf.cycles_num && music_conf.cycles_num <= 127)
			aai_set_nbit(aai, AAI_AAICFG_MUS(musx),
				     AAI_MUSx_CYCLES_NUM_MSK,
				     AAI_MUSx_CYCLES_NUM_SHIFT,
				     music_conf.cycles_num);

		aai_setbit(aai, AAI_AAICFG_MUS(musx),
			   AAI_MUSx_MATSUSHITA_MSK, music_conf.matsushita);
		aai_setbit(aai, AAI_AAICFG_MUS(musx),
			   AAI_MUSx_LEFT_FRAME_MSK, music_conf.left_frame);
		aai_setbit(aai, AAI_AAICFG_MUS(musx),
			   AAI_MUSx_LSB_FIRST_MSK, music_conf.lsb_first);
		aai_setbit(aai, AAI_AAICFG_MUS(musx),
			   AAI_MUSx_RIGHT_JUST_MSK, music_conf.right_just);
		aai_setbit(aai, AAI_AAICFG_MUS(musx),
			   AAI_MUSx_24BITS_MSK, music_conf.bits24);
	}

	return err;
}

enum tdm_dir_t {
	TDM_IN,
	TDM_OUT
};

static int aai_hwdep_set_tdm_conf(struct card_data_t *aai,
				  unsigned long arg,
				  enum tdm_dir_t dir)
{
	int err = 0;
	struct aai_ioctl_tdm_conf_t tdm_conf;
	uint32_t offset;
	uint32_t cycle_mask;
	uint32_t cycle_shift;
	uint32_t lf_mask;

	if (dir == TDM_IN) {
		offset = AAI_AAICFG_TDM_IN;
		cycle_mask = AAI_TDM_IN_CYCLES_NUM_MSK;
		cycle_shift = AAI_TDM_IN_CYCLES_NUM_SHIFT;
		lf_mask = AAI_TDM_IN_LEFT_FRAME_MSK;
	} else {
		offset = AAI_AAICFG_TDM_OUT;
		cycle_mask = AAI_TDM_OUT_CYCLES_NUM_MSK;
		cycle_shift = AAI_TDM_OUT_CYCLES_NUM_SHIFT;
		lf_mask = AAI_TDM_OUT_LEFT_FRAME_MSK;
	}

	err = get_data((void __user *)arg, &tdm_conf,
		       sizeof(struct aai_ioctl_tdm_conf_t));
	if (!err) {
		if (8 <= tdm_conf.cycles_num && tdm_conf.cycles_num <= 127)
			aai_set_nbit(aai, offset, cycle_mask,
				     cycle_shift, tdm_conf.cycles_num);

		aai_setbit(aai, offset, lf_mask, tdm_conf.left_frame);
	}

	return err;
}

enum adc_type_t {
	AAI_ADC,
	AAI_DAC,
	AAI_AUX
};

static int aai_hwdep_set_adc_conf(struct card_data_t *aai,
				  unsigned long arg,
				  enum adc_type_t type)
{
	int err = 0;
	struct aai_ioctl_adc_conf_t adc_conf;

	switch (type) {
	case AAI_ADC:
		err = get_data((void __user *)arg, &adc_conf,
			       sizeof(struct aai_ioctl_adc_conf_t));
		if (!err) {
			if (0 <= adc_conf.cycles_num &&
			    adc_conf.cycles_num <= 127)
				aai_set_nbit(aai, AAI_AAICFG_ADC,
					     AAI_ADC_CYCLES_NUM_MSK,
					     AAI_ADC_CYCLES_NUM_SHIFT,
					     adc_conf.cycles_num);

			aai_setbit(aai, AAI_AAICFG_ADC,
				   AAI_ADC_MATSUSHITA_MSK, adc_conf.matsushita);
			aai_setbit(aai, AAI_AAICFG_ADC,
				   AAI_ADC_LEFT_FRAME_MSK, adc_conf.left_frame);
		}
		break;

	case AAI_DAC:
		err = get_data((void __user *)arg, &adc_conf,
			       sizeof(struct aai_ioctl_adc_conf_t));
		if (!err) {
			if (0 <= adc_conf.cycles_num &&
			    adc_conf.cycles_num <= 127)
				aai_set_nbit(aai, AAI_AAICFG_DAC,
					     AAI_DAC_CYCLES_NUM_MSK,
					     AAI_DAC_CYCLES_NUM_SHIFT,
					     adc_conf.cycles_num);

			aai_setbit(aai, AAI_AAICFG_DAC,
				   AAI_DAC_MATSUSHITA_MSK, adc_conf.matsushita);
			aai_setbit(aai, AAI_AAICFG_DAC,
				   AAI_DAC_LEFT_FRAME_MSK, adc_conf.left_frame);
		}
		break;

	case AAI_AUX:
		err = get_data((void __user *)arg, &adc_conf,
			       sizeof(struct aai_ioctl_adc_conf_t));
		if (!err) {
			if (0 <= adc_conf.cycles_num &&
			    adc_conf.cycles_num <= 127)
				aai_set_nbit(aai, AAI_AAICFG_AUX,
					     AAI_AUX_CYCLES_NUM_MSK,
					     AAI_AUX_CYCLES_NUM_SHIFT,
					     adc_conf.cycles_num);

			aai_setbit(aai, AAI_AAICFG_AUX,
				   AAI_AUX_HALF_FRAME_MSK, adc_conf.half_frame);
		}
		break;

	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int aai_hwdep_set_pcm_conf(struct card_data_t *aai,
				  unsigned long arg,
				  int nb)
{
	int err = 0;
	struct aai_ioctl_pcm_conf_t pcm_conf;

	err = get_data((void __user *)arg, &pcm_conf,
		       sizeof(struct aai_ioctl_pcm_conf_t));
	if (!err) {
		if (0 <= pcm_conf.start_two && pcm_conf.start_two <= 0xff)
			aai_set_nbit(aai, AAI_AAICFG_PCM(nb),
				     AAI_PCM_START_TWO_MSK,
				     AAI_PCM_START_TWO_SHIFT,
				     pcm_conf.start_two);

		if (0 <= pcm_conf.cycles_pcm_high &&
		    pcm_conf.cycles_pcm_high <= 0xff)
			aai_set_nbit(aai, AAI_AAICFG_PCM(nb),
				     AAI_PCM_CYCLES_HIGH_MSK,
				     AAI_PCM_CYCLES_HIGH_SHIFT,
				     pcm_conf.cycles_pcm_high);

		if (0 <= pcm_conf.cycles_pcm_low &&
		    pcm_conf.cycles_pcm_low <= 0x1ff)
			aai_set_nbit(aai, AAI_AAICFG_PCM(nb),
				     AAI_PCM_CYCLES_LOW_MSK,
				     AAI_PCM_CYCLES_LOW_SHIFT,
				     pcm_conf.cycles_pcm_low);

		aai_setbit(aai, AAI_AAICFG_PCM(nb),
			   AAI_PCM_OKI_MSK, pcm_conf.oki);
		aai_setbit(aai, AAI_AAICFG_PCM(nb),
			   AAI_PCM_RUN_DUAL_MSK, pcm_conf.run_dual);
	}

	return err;
}

static int aai_hwdep_group_start(struct card_data_t *aai, unsigned long arg)
{
	int err;
	int group_nb;

	err = get_data((void __user *)arg, &group_nb, sizeof(int));
	if (!err)
		err = aai_group_start(aai, group_nb);

	return err;
}

static int aai_hwdep_group_sync(struct card_data_t *aai, unsigned long arg)
{
	int err;
	struct aai_ioctl_group_t group;

	err = get_data((void __user *)arg, &group,
		       sizeof(struct aai_ioctl_group_t));
	if (!err)
		err = aai_group_set(aai, group.name, group.group);

	return err;
}

static int aai_hwdep_set_mframe_asym_conf(struct card_data_t *aai,
					  unsigned long arg)
{
	int err = 0;
	struct aai_ioctl_asym_conf_t conf;

	err = get_data((void __user *)arg, &conf,
		       sizeof(struct aai_ioctl_asym_conf_t));
	aai_setbit(aai, AAI_LBC_CLKEN, AAI_LBC_CKEN_MFRAME_MSK, conf.enable);
	aai_setbit(aai, AAI_LBC_MFRAME,
		   AAI_LBC_MFRAME_MFRAME_DIV_LOW_MSK, conf.div_low);
	aai_setbit(aai, AAI_LBC_MFRAME,
		   AAI_LBC_MFRAME_MFRAME_DIV_HIGH_MSK, conf.div_high);

	return err;
}

static int aai_hwdep_set_hframe_asym_conf(struct card_data_t *aai,
					  unsigned long arg)
{
	int err = 0;
	struct aai_ioctl_asym_conf_t conf;

	err = get_data((void __user *)arg, &conf,
		       sizeof(struct aai_ioctl_asym_conf_t));
	aai_setbit(aai, AAI_LBC_CLKEN, AAI_LBC_CKEN_HFRAME_MSK, conf.enable);
	aai_setbit(aai, AAI_LBC_HFRAME,
		   AAI_LBC_HFRAME_HFRAME_DIV_LOW_MSK, conf.div_low);
	aai_setbit(aai, AAI_LBC_HFRAME,
		   AAI_LBC_HFRAME_HFRAME_DIV_HIGH_MSK, conf.div_high);

	return err;
}

/**
 * AAI Driver IO control function
 */
static int aai_hwdep_ioctl(struct snd_hwdep *hwdep,
			   struct file      *file,
			   unsigned int     cmd,
			   unsigned long    arg)
{
	int err = -EINVAL;
	uint32_t val = 0;
	struct card_data_t *aai = hwdep->private_data;
	unsigned long flags = 0;

	spin_lock_irqsave(&aai->hwlock, flags);

	/**
	 * <ul>
	 */
	switch (cmd) {
	/**
	 * <li><b>AAI_MUS0_CONF:</b><br>
	 * Set AAI music 0 configuration register<br>
	 * Param  : AAI music configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_MUS0_CONF:
		err = aai_hwdep_set_music_conf(aai, arg, 0);
		break;

	/**
	 * <li><b>AAI_MUS1_CONF:</b><br>
	 * Set AAI music 1 configuration register<br>
	 * Param  : AAI music configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_MUS1_CONF:
		err = aai_hwdep_set_music_conf(aai, arg, 1);
		break;

	/**
	 * <li><b>AAI_MUS2_CONF:</b><br>
	 * Set AAI music 2 configuration register<br>
	 * Param  : AAI music configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_MUS2_CONF:
		err = aai_hwdep_set_music_conf(aai, arg, 2);
		break;

	/**
	 * <li><b>AAI_MUS3_CONF:</b><br>
	 * Set AAI music 3 configuration register<br>
	 * Param  : AAI music configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_MUS3_CONF:
		err = aai_hwdep_set_music_conf(aai, arg, 3);
		break;

	/**
	 * <li><b>AAI_TDMIN_CONF:</b><br>
	 * Set AAI TDM input configuration register<br>
	 * Param  : AAI TDM configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_TDMIN_CONF:
		err = aai_hwdep_set_tdm_conf(aai, arg, TDM_IN);
		break;

	/**
	 * <li><b>AAI_TDMOUT_CONF:</b><br>
	 * Set AAI TDM output configuration register<br>
	 * Param  : AAI TDM configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_TDMOUT_CONF:
		err = aai_hwdep_set_tdm_conf(aai, arg, TDM_OUT);
		break;

	/**
	 * <li><b>AAI_ADC_CONF:</b><br>
	 * Set AAI ADC configuration register<br>
	 * Param  : AAI ADC configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_ADC_CONF:
		err = aai_hwdep_set_adc_conf(aai, arg, AAI_ADC);
		break;

	/**
	 * <li><b>AAI_DAC_CONF:</b><br>
	 * Set AAI DAC configuration register<br>
	 * Param  : AAI ADC configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_DAC_CONF:
		err = aai_hwdep_set_adc_conf(aai, arg, AAI_DAC);
		break;

	/**
	 * <li><b>AAI_AUX_CONF:</b><br>
	 * Set AAI AUX configuration register<br>
	 * Param  : AAI ADC configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_AUX_CONF:
		err = aai_hwdep_set_adc_conf(aai, arg, AAI_AUX);
		break;

	/**
	 * <li><b>AAI_PCM1_CONF:</b><br>
	 * Set AAI PCM 1 configuration register<br>
	 * Param  : AAI PCM configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_PCM1_CONF:
		err = aai_hwdep_set_pcm_conf(aai, arg, 0);
		break;

	/**
	 * <li><b>AAI_PCM2_CONF:</b><br>
	 * Set AAI PCM 2 configuration register<br>
	 * Param  : AAI PCM configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_PCM2_CONF:
		err = aai_hwdep_set_pcm_conf(aai, arg, 1);
		break;

	/**
	 * <li><b>AAI_VOICE_SPEED:</b><br>
	 * Set AAI VOICE_SPEED register<br>
	 * Param  : requested voice speed, recommended: 0x810E35 for 16k/8k;
		    0xB1DAC7 for 22.05k/11k<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_VOICE_SPEED:
		err = get_data((void __user *)arg, &val, sizeof(uint32_t));
		if (!err) {
			if (0 <= val && val <= 0xffffff)
				aai_set_nbit(aai, AAI_AAICFG_VOICE_SPEED,
					     0xffffff, 0, val);
		}
		break;

	/**
	 * <li><b>AAI_VOICE_8K_SPEAKER:</b><br>
	 * Set AAI VOICE_8K register for speaker<br>
	 * Param  : Set 0 for 16k speaker, 1 for 8k speaker<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_VOICE_8K_SPEAKER:
		err = get_data((void __user *)arg, &val, sizeof(uint32_t));
		if (!err)
			aai_setbit(aai, AAI_AAICFG_VOICE_8K,
				   AAI_VOICE_8K_SPEAKER_MSK, !!val);
		break;

	/**
	 * <li><b>AAI_VOICE_8K_PHONE:</b><br>
	 * Set AAI VOICE_8K register for phone<br>
	 * Param  : Set 0 for 16k phone, 1 for 8k phone<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_VOICE_8K_PHONE:
		err = get_data((void __user *)arg, &val, sizeof(uint32_t));
		if (!err)
			aai_setbit(aai, AAI_AAICFG_VOICE_8K,
				   AAI_VOICE_8K_PHONE_MSK, !!val);
		break;

	/**
	 * <li><b>AAI_GROUP_START:</b><br>
	 * Start audio group specified in argument<br>
	 * Param  : group to be started<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_GROUP_START:
		err = aai_hwdep_group_start(aai, arg);
		break;

	/**
	 * <li><b>AAI_GROUP_SYNC:</b><br>
	 * Assign audio channel to a group<br>
	 * Param  : structure containing channel name and group number<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_GROUP_SYNC:
		err = aai_hwdep_group_sync(aai, arg);
		break;

	/**
	 * <li><b>AAI_I2S_ASYMMETRIC:</b><br>
	 * Set AAI I2S asymmetric configuration register<br>
	 * Param  : AAI I2S asymmetric configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_MFRAME_ASYMMETRIC_CONF:
		err = aai_hwdep_set_mframe_asym_conf(aai, arg);
		break;

	/**
	 * <li><b>AAI_TDM_ASYMMETRIC:</b><br>
	 * Set AAI TDM asymmetric configuration register<br>
	 * Param  : AAI TDM asymmetric configuration structure<br>
	 * Return : error code
	 * <hr>
	 */
	case AAI_HFRAME_ASYMMETRIC_CONF:
		err = aai_hwdep_set_hframe_asym_conf(aai, arg);
		break;

	default:
		break;
	}
	/**
	 * </ul>
	 */

	spin_unlock_irqrestore(&aai->hwlock, flags);
	return err;
}

static int aai_hwdep_dummy(struct snd_hwdep *hw, struct file *file)
{
	return 0;
}

int aai_ioctl_hwdep_new(struct card_data_t *aai)
{
	struct snd_hwdep *rhwdep;
	int err = 0;

	err = snd_hwdep_new(aai->card, "AAIhwdep", 0, &rhwdep);
	if (err < 0)
		return err;

	rhwdep->private_data = aai;
	rhwdep->ops.open     = aai_hwdep_dummy;
	rhwdep->ops.ioctl    = aai_hwdep_ioctl;
	rhwdep->ops.release  = aai_hwdep_dummy;

	return 0;
}

