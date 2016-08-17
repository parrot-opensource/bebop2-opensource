
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include "aai.h"
#include "aai_hw.h"
#include "aai_rules.h"

static int devdma_mmap(struct device *dev, struct snd_pcm_substream *substream,
		       struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return dma_mmap_coherent(NULL, vma, runtime->dma_area,
				 runtime->dma_addr, runtime->dma_bytes);
}

static int aai_ops_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	return devdma_mmap(NULL, substream, vma);
}

/**
 * @brief Open operator
 *        Initialize hw_params of the substream's runtime
 *
 * @param substream  to open
 *
 * @return error number
 */
static int aai_ops_open(struct snd_pcm_substream *substream)
{
	int err = 0;
	struct aai_device_t *chan = substream2chan(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aai_hw_ops *hw_ops = &chan->aai_hw_ops;

	/* copy parameters to runtime structure */
	runtime->hw.info             = chan->hw->info;
	runtime->hw.formats          = chan->hw->formats;
	runtime->hw.channels_min     = chan->hw->channels_min;
	runtime->hw.channels_max     = chan->hw->channels_max;
	runtime->hw.rates            = chan->hw->rates;
	runtime->hw.rate_min         = chan->hw->rate_min;
	runtime->hw.rate_max         = chan->hw->rate_max;
	runtime->hw.buffer_bytes_max = chan->hw->buffer_bytes_max;
	runtime->hw.period_bytes_min = chan->hw->period_bytes_min;
	runtime->hw.period_bytes_max = chan->hw->period_bytes_max;
	runtime->hw.periods_min      = chan->hw->periods_min;
	runtime->hw.periods_max      = chan->hw->periods_max;
	runtime->hw.fifo_size        = chan->hw->fifo_size;

	if (hw_ops && hw_ops->open)
		err = hw_ops->open(chan->driver_data, chan);

	return err;
}

/**
 * @brief Close operator. Disable device.
 *
 * @param substream  to close
 *
 * @return error number
 */
static int aai_ops_close(struct snd_pcm_substream *substream)
{
	int err = 0;
	struct aai_device_t *chan = substream2chan(substream);
	struct aai_hw_ops *hw_ops = &chan->aai_hw_ops;

	if (hw_ops && hw_ops->close)
		err = hw_ops->close(chan->driver_data, chan);

	return err;
}

/**
 * @brief Params operator
 *        This is called when the hardware parameter (hw_params) is set up by
 *        the application, that is, once when the buffer size, the period size,
 *        the format, etc. are defined for the pcm substream.
 *        Many hardware setups should be done in this callback, including
 *        the allocation of buffers.
 *        Parameters to be initialized are retrieved by params_xxx() macros
 *
 * @param substream  targeted substream
 * @param hw_params  parameters to set
 *
 * @return error number
 */
static int aai_ops_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *hw_params)
{
	int err = 0;
	struct aai_device_t *chan = substream2chan(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aai_hw_ops *hw_ops = &chan->aai_hw_ops;
	int access;

	chan->bufsize = params_buffer_bytes(hw_params);
	chan->rate    = params_rate(hw_params);
	access        = params_access(hw_params);

	/* Check access mode. Only interleaved is supported.  */
	if ((access != SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) &&
	    (access != SNDRV_PCM_ACCESS_RW_INTERLEAVED)) {
		dev_warn(chan2dev(chan),
			"%s (%d) - unsupported access type 0x%x\n",
			chan->name, chan->ipcm, access);
		err = -EINVAL;
		goto exit;
	}

	/* audio stream buffer allocation */
	err = aai_hw_dma_alloc(substream, chan->bufsize);
	if (err < 0)
		goto exit;

	/* Set fifo pointers to DMA area, for interleaved devices only */
	chan->fifo.pstart   = (void *)runtime->dma_addr;
	chan->fifo.pend     = chan->fifo.pstart + chan->bufsize;
	chan->fifo.pcurrent = chan->fifo.pstart;

	if (hw_ops && hw_ops->hw_params)
		err = hw_ops->hw_params(chan->driver_data, chan, hw_params);
exit:
	return err;
}

/**
 * @brief Prepare operator.
 *        This callback is called when the pcm is "prepared".
 *        You can set the format type, sample rate, etc. here.
 *        The difference from hw_params is that the prepare
 *        callback will be called each time snd_pcm_prepare()
 *        is called, i.e. when recovering after underruns, etc.
 *
 * @param substream  to be prepared
 *
 * @return error number
 */
static int aai_ops_prepare(struct snd_pcm_substream *substream)
{
	int err = 0;
	int i;
	struct aai_device_t *chan = substream2chan(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aai_hw_ops *hw_ops = &chan->aai_hw_ops;

	/* Init channel infos */
	chan->period_frames = runtime->period_size;
	chan->period_bytes = frames_to_bytes(runtime, runtime->period_size);
	chan->dma_area = runtime->dma_area;

	/* Prepare DMA area */
	err = aai_hw_dma_prepare(chan);
	if (err)
		goto exit;

	/* Apply rules */
	for (i = 0; i < NB_RULES; i++) {
		if (chan->rules[i] == RULE_END)
			break;
		err = aai_rule_apply(chan->driver_data, chan->rules[i]);
		if (err)
			goto exit;
	}

	chan->fifo.pcurrent = chan->fifo.pstart;

	if (hw_ops && hw_ops->prepare)
		err = hw_ops->prepare(chan->driver_data, chan);
exit:
	return err;
}

/**
 * @brief Free operator
 *
 * @param substream  to be released
 *
 * @return error number
 */
static int aai_ops_free(struct snd_pcm_substream *substream)
{
	int err = 0;
	int i;
	struct aai_device_t *chan = substream2chan(substream);
	struct aai_hw_ops *hw_ops = &chan->aai_hw_ops;

	aai_hw_dma_free(substream);

	/* Remove rules */
	for (i = 0; i < NB_RULES; i++) {
		if (chan->rules[i] == RULE_END)
			break;
		err = aai_rule_remove(chan->driver_data, chan->rules[i]);
		if (err)
			goto exit;
	}

	if (hw_ops && hw_ops->free)
		err = hw_ops->free(chan->driver_data, chan);
exit:
	return err;
}

/**
 * @brief Pointer operator.
 *
 * @param substream  targeted substream
 *
 * @return amount of frames used within the audio buffer
 */
static snd_pcm_uframes_t aai_ops_pointer(struct snd_pcm_substream *substream)
{
	struct aai_device_t *chan = substream2chan(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	ssize_t bytes = chan->fifo.pcurrent - chan->fifo.pstart;

	return bytes_to_frames(runtime, bytes);
}

/**
 * @brief Trigger operator
 *
 * @param substream  targeted substream
 * @param cmd        command to execute
 *
 * @return error number
 */
static int aai_ops_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct aai_device_t *chan = substream2chan(substream);
	struct card_data_t *aai = chan->driver_data;
	int err = 0;

	if (chan == NULL) {
		dev_warn(chan2dev(chan),
			 "device pointer is NULL substream @ %p", substream);
		err = -EINVAL;
		goto exit;
	}

	if (chan->ipcm >= aai->chans_nb) {
		dev_warn(chan2dev(chan),
			 "substream @ %p - device identifier %d is out of range",
			 substream, chan->ipcm);
		err = -EINVAL;
		goto exit;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dev_dbg(chan2dev(chan), "%s (%d) - TRIGGER_START\n",
			chan->name, chan->ipcm);
		err = aai_hw_start_channel(chan);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		dev_dbg(chan2dev(chan), "%s(%d) - TRIGGER_RESUME\n",
			chan->name, chan->ipcm);
		err = aai_ops_prepare(substream);
		if (err)
			goto exit;
		err = aai_hw_start_channel(chan);
		if (err)
			goto exit;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		dev_dbg(chan2dev(chan), "%s(%d) - TRIGGER_STOP it:%d %d\n",
			chan->name, chan->ipcm, chan->nbirq, chan->nbperiod);
		err = aai_hw_stop_channel(chan);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		dev_dbg(chan2dev(chan), "%s(%d) - TRIGGER_SUSPEND\n",
			chan->name, chan->ipcm);
		err = aai_hw_stop_channel(chan);
		break;

	default:
		dev_dbg(chan2dev(chan), "%s(%d) - TRIGGER UNKNOWN\n",
			chan->name, chan->ipcm);
		err = -EINVAL;
		break;
	}

exit:
	return err;
}

/**
 * @brief IOCTL operator
 *
 * @param substream  targeted substream
 * @param cmd        command to execute
 * @param arg        argument to the command
 *
 * @return error number
 */
static int aai_ops_ioctl(struct snd_pcm_substream *substream,
			 unsigned int cmd,
			 void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

struct snd_pcm_ops aai_pcm_ops = {
	.open      = aai_ops_open,
	.close     = aai_ops_close,
	.ioctl     = aai_ops_ioctl,
	.hw_params = aai_ops_params,
	.hw_free   = aai_ops_free,
	.prepare   = aai_ops_prepare,
	.trigger   = aai_ops_trigger,
	.pointer   = aai_ops_pointer,
	.mmap      = aai_ops_mmap,
};

