
#ifndef _AAI_ALSA_H_
#define _AAI_ALSA_H_

struct snd_card * __devinit aai_alsa_probe(struct platform_device *pdev);
int __devexit aai_alsa_remove(struct snd_card *card);

#endif

