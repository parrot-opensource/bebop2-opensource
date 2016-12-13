
#include "aai.h"
#include "aai_hw.h"

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* Index 0-MAX */
static char *id[SNDRV_CARDS]  = SNDRV_DEFAULT_STR; /* ID for this card */

int aai_pcms_probe(struct card_data_t *aai)
{
	int err = 0;
	int ipcm;
	struct aai_device_t *chan;

	/* alloc pcms table */
	aai->pcms = kmalloc(aai->chans_nb * sizeof(struct snd_pcm *),
			    GFP_KERNEL);
	if (!aai->pcms) {
		err = -ENOMEM;
		goto exit;
	}

	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		chan->driver_data = aai;

		err = snd_pcm_new(aai->card,
				  chan->name,
				  ipcm,
				  ((chan->direction&AAI_TX) == AAI_TX),
				  ((chan->direction&AAI_RX) == AAI_RX),
				  &(aai->pcms[ipcm]));
		if (err < 0) {
			dev_err(aai->dev, "%s: %s (%d) init failed\n",
				__func__, chan->name, ipcm);
			goto exit;
		}

		aai->pcms[ipcm]->private_data = chan;
		aai->pcms[ipcm]->info_flags = 0;
		strncpy(aai->pcms[ipcm]->name, chan->name, DEV_NAME_LEN);

		/* Set channel operators */
		if (chan->direction & AAI_TX) {
			snd_pcm_set_ops(aai->pcms[ipcm],
					SNDRV_PCM_STREAM_PLAYBACK, chan->ops);
		} else if (chan->direction & AAI_RX) {
			snd_pcm_set_ops(aai->pcms[ipcm],
					SNDRV_PCM_STREAM_CAPTURE, chan->ops);
		} else {
			dev_err(aai->dev, "%s init failed\n", chan->name);
			err = -EINVAL;
			goto exit;
		}

		/* Buffer memory preallocation */
		err = snd_pcm_lib_preallocate_pages_for_all(aai->pcms[ipcm],
							    SNDRV_DMA_TYPE_DEV,
							    NULL,
							    64*1024,
							    64*1024);
		if (err) {
			dev_err(aai->dev, "%s: alsa memory preallocation "
				 "failed\n", __func__);
			err = -EINVAL;
			goto exit;
		}
	}

exit:
	return err;
}

int aai_pcms_init(struct card_data_t *aai)
{
	int err = 0;
	int ipcm;
	struct aai_device_t *chan;

	aai->fifo_address = 0;
	aai->fifo_tx      = 0;
	aai->fifo_rx      = 0;
	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		chan->ipcm = ipcm;
		err = aai_hw_pcm_init(aai, chan);
		if (err)
			goto exit;
	}

exit:
	return err;
}

struct snd_card *aai_alsa_probe(struct platform_device *pdev)
{
	int err;
	struct snd_card *card = NULL;
	struct card_data_t *aai;
	int dev = pdev->id;

	/* Create ALSA soundcard */
	err = snd_card_create(index[dev], id[dev], THIS_MODULE,
			      sizeof(struct card_data_t), &card);
	if (err < 0) {
		dev_err(&pdev->dev, "card creation failed\n");
		card = NULL;
		goto exit;
	}

	aai = card->private_data;
	aai->card = card;
	aai->dev = &pdev->dev;
	aai->pdata = dev_get_platdata(&pdev->dev);

	/* Call hardware specific card init */
	err = aai_hw_init_card(aai, dev);
	if (err) {
		dev_err(&pdev->dev, "hw init failed with error %d\n", err);
		goto __nodev;
	}

	err = aai_pcms_probe(aai);
	if (err) {
		dev_err(&pdev->dev, "pcm probe failed with error %d\n", err);
		goto __nodev;
	}

	/* Set device & register card to ALSA system */
	snd_card_set_dev(card, &pdev->dev);
	err = snd_card_register(card);
	if (err != 0) {
		dev_err(&pdev->dev, "card registration failed with error %d\n",
			err);
		goto __nodev;
	}

exit:
	return card;
__nodev:
	snd_card_free(card);
	card = NULL;
	return card;
}

int aai_alsa_remove(struct snd_card *card)
{
	int ipcm;
	struct card_data_t *aai;

	if (!card)
		goto exit;

	aai = card->private_data;

	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++)
		snd_pcm_lib_preallocate_free_for_all(aai->pcms[ipcm]);

	snd_card_free(card);

exit:
	return 0;
}

