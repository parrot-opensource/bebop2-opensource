/**
 * @file   aai_module.c
 * @brief  P7 AAI kernel module layer
 *
 * @author adrien.charruel@parrot.com
 * @date   2011-05-17
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#include "aai.h"
#include "aai_alsa.h"
#include "aai_hw.h"


static int aai_start(struct card_data_t *aai_data, struct platform_device *pdev)
{
	int err;

	/* prepare controller clock */
	err = clk_prepare_enable(aai_data->clk[ctrl_clk]);
	if (err) {
		dev_err(&pdev->dev, "failed to enable ctrl clock.\n");
		return err;
	}
	/* AAI hardware init */
	err = aai_hw_init(aai_data);
	if (err) {
		dev_err(&pdev->dev, "AAI hardware init failed\n");
		return -EINVAL;
	}

	/* Register devices */
	err = aai_pcms_init(aai_data);
	if (err) {
		dev_err(&pdev->dev, "aai pcms Init failed\n");
		return err;
	}

	return 0;
}

static int __devinit aai_module_probe(struct platform_device *pdev)
{
	int err;
	struct resource *res;
	struct snd_card *card;
	struct card_data_t *aai_data;
	struct clk *clk;

	/* Map registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto no_iores;
	}

	/* Init driver structures */
	card = aai_alsa_probe(pdev);
	if (!card) {
		dev_err(&pdev->dev, "Can't probe AAI alsa card\n");
		err = -ENXIO;
		goto no_iores;
	}
	aai_data = card->private_data;
	platform_set_drvdata(pdev, card);

	aai_data->pdata = dev_get_platdata(&pdev->dev);
	if (!aai_data->pdata->pad) {
		dev_err(&pdev->dev, "no pad configuration\n");
		err = -EINVAL;
		goto no_iores;
	}

	if (!aai_data->pdata->device_list) {
		dev_err(&pdev->dev, "no device list defined\n");
		err = -EINVAL;
		goto no_iores;
	}

	/* Init IO */
	aai_data->ioarea = request_mem_region(res->start,
					      res->end - res->start + 1,
					      pdev->name);
	if (!aai_data->ioarea) {
		dev_err(&pdev->dev, "Can't reserve IO region\n");
		err = -ENXIO;
		goto no_iores;
	}

	aai_data->iobase = ioremap(res->start, res->end - res->start + 1);
	if (!aai_data->iobase) {
		dev_err(&pdev->dev, "Can't map IO\n");
		err = -ENXIO;
		goto no_iomap;
	}

	/* Get controller clock */
	clk = clk_get(&pdev->dev, "ctrl_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get aai ctrl clk\n");
		err = -EINVAL;
		goto no_ctrlclk;
	}
	aai_data->clk[ctrl_clk] = clk;

	clk = clk_get(&pdev->dev, "i2s_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get aai i2s clk\n");
		err = -EINVAL;
		goto no_i2sclk;
	}
	aai_data->clk[i2s_clk] = clk;

	clk = clk_get(&pdev->dev, "pcm1_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get aai pcm1 clk\n");
		err = -EINVAL;
		goto no_pcm1clk;
	}
	aai_data->clk[pcm1_clk] = clk;

	clk = clk_get(&pdev->dev, "pcm2_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get aai pcm2 clk\n");
		err = -EINVAL;
		goto no_pcm2clk;
	}
	aai_data->clk[pcm2_clk] = clk;

	clk = clk_get(&pdev->dev, "spdif_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get aai spdif clk\n");
		err = -EINVAL;
		goto no_spdifclk;
	}
	aai_data->clk[spdif_clk] = clk;

	/*
	 * Get IRQ ressource
	 */
	aai_data->irq = platform_get_irq(pdev, 0);
	if (aai_data->irq < 0) {
		dev_err(&pdev->dev, "Can't retrieve IRQ number\n");
		err = -ENOENT;
		goto no_irq;
	}

	/*
	 * reset registers, set right default value
	 * enable main clk
	 * set irq
	 * The aai_start function is also used when
	 * resume after suspend
	 */
	err = aai_start(aai_data, pdev);
	if (err)
		goto no_clk_enable;

	/*
	 * Install IRQ handler
	 */
	err = request_irq(aai_data->irq, aai_data->irq_handler,
			  IRQF_DISABLED, pdev->name, aai_data);
	if (err) {
		dev_err(&pdev->dev, "failed attaching IRQ %d\n", aai_data->irq);
		return err;
	}

	aai_data->pin = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(aai_data->pin)) {
		dev_err(&pdev->dev, "Can't get pin ctrl\n");
		goto no_irq;
	}
	dev_info(&pdev->dev, "P7 AAI module initialized\n");
	return 0;

no_irq:
	clk_disable_unprepare(aai_data->clk[ctrl_clk]);
no_clk_enable:
	if (err)
		clk_put(aai_data->clk[spdif_clk]);
no_spdifclk:
	if (err)
		clk_put(aai_data->clk[pcm2_clk]);
no_pcm2clk:
	if (err)
		clk_put(aai_data->clk[pcm1_clk]);
no_pcm1clk:
	if (err)
		clk_put(aai_data->clk[i2s_clk]);
no_i2sclk:
	if (err)
		clk_put(aai_data->clk[ctrl_clk]);
no_ctrlclk:
	if (err)
		iounmap(aai_data->iobase);
no_iomap:
	if (err) {
		release_resource(aai_data->ioarea);
		kfree(aai_data->ioarea);
	}
no_iores:
	if (err)
		platform_set_drvdata(pdev, NULL);
	return err;
}

static int __devexit aai_module_remove(struct platform_device *pdev)
{
	int err;
	struct snd_card *card = platform_get_drvdata(pdev);
	struct card_data_t *aai_data = card->private_data;

	/* reset hardware layer */
	err = aai_hw_remove(aai_data);
	if (err) {
		dev_err(&pdev->dev, "AAI hardware remove failed\n");
		err = -EINVAL;
		goto exit;
	}

	free_irq(aai_data->irq, aai_data);

	/*
	 * beware, ths call will free drv_data which is included
	 * into card structure.
	 */
	aai_alsa_remove(card);

	iounmap(aai_data->iobase);
	release_resource(aai_data->ioarea);
	kfree(aai_data->ioarea);

	platform_set_drvdata(pdev, NULL);
	pinctrl_put(aai_data->pin);

exit:
	return err;
}

static int aai_module_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct card_data_t *aai = card->private_data;
	struct aai_device_t *chan;
	struct aai_hw_channel_t *fifo;
	struct aai_audio_chan_t *internal_chan;
	int ipcm, i;

	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		fifo = &chan->fifo;

		for (i = 0; i < fifo->chan_nb; i++) {
			internal_chan = fifo->chan_list[i];
			internal_chan->used = 0;
		}
		snd_pcm_suspend_all(aai->pcms[ipcm]);
	}

	free_irq(aai->irq, aai);

	clk_disable(aai->clk[i2s_clk]);
	clk_disable(aai->clk[ctrl_clk]);

	snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);

	return 0;
}

static int aai_module_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct card_data_t *aai = card->private_data;
	int err;

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	err = aai_start(aai, pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to resume %d\n", err);
			return err;
	}

	/* Install IRQ handler */
	err = request_irq(aai->irq, aai->irq_handler,
			  IRQF_DISABLED, pdev->name, aai);
	if (err) {
		dev_err(&pdev->dev, "failed attaching IRQ %d\n", aai->irq);

		return err;
	}

	return 0;
}

static struct platform_driver aai_module_driver = {
	.probe   = aai_module_probe,
	.remove  = aai_module_remove,
	.suspend = aai_module_suspend,
	.resume  = aai_module_resume,
	.driver  = {
		.name  = "aai",
		.owner = THIS_MODULE,
	},
};

static int __init aai_module_init(void)
{
	return platform_driver_register(&aai_module_driver);
}

static void __exit aai_module_exit(void)
{
	platform_driver_unregister(&aai_module_driver);
}

module_init(aai_module_init);
module_exit(aai_module_exit);

MODULE_DESCRIPTION("Parrot7 Advanced Audio Interface driver");
MODULE_AUTHOR("Adrien Charruel, <adrien.charruel@parrot.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{Parrot, P7AAI}}");

