
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/*
 * Digital audio interface glue - connects codec <--> CPU
 * Here the purpose is to simply declare the codec, no real connection is made
 * between codec DAI and CPU DAI. Hence we use a dummy cpu dai.
 */

/* Audio machine driver */

static int parrot_fc7100_soc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_soc_card *card;

	card = kzalloc(sizeof(struct snd_soc_card), GFP_KERNEL);
	if (!card) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	card->dai_link  = pdev->dev.platform_data;
	card->num_links = 1;
	card->dev       = &pdev->dev;
	card->owner     = THIS_MODULE;

	card->name = kasprintf(GFP_KERNEL, "codec control %s", card->dai_link->name);
	if (!card->name)
		dev_warn(&pdev->dev, "failed to set card name for codec '%s'\n",
			 card->dai_link->name);

	dev_info(&pdev->dev, "Parrot FC7100 SoC probe '%s'\n", card->name);
	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n",
			ret);

	return ret;
}

static int parrot_fc7100_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&pdev->dev);
	snd_soc_unregister_card(card);
	kfree(card->name);
	kfree(card);
	return 0;
}

int parrot_fc7100_soc_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM_SLEEP
	return snd_soc_suspend(&pdev->dev);
#else
	return 0;
#endif
}

int parrot_fc7100_soc_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM_SLEEP
	return snd_soc_resume(&pdev->dev);
#else
	return 0;
#endif
}

static struct platform_driver parrot_fc7100_soc_driver = {
	.driver		= {
		.name	= "parrot-fc7100-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= parrot_fc7100_soc_probe,
	.remove		= parrot_fc7100_soc_remove,
	.suspend	= parrot_fc7100_soc_suspend,
	.resume		= parrot_fc7100_soc_resume,
};

module_platform_driver(parrot_fc7100_soc_driver);

MODULE_AUTHOR("Adrien Charruel <adrien.charruel@parrot.com>");
MODULE_DESCRIPTION("ALSA SoC Parrot FC7100 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:parrot-fc7100");
