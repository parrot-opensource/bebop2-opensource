
#ifndef _AAI_H_
#define _AAI_H_

#ifndef CONFIG_ARCH_VEXPRESS
# include <mach/p7.h>
#endif

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <sound/hwdep.h>

#include "aai_platform_data.h"

#define NB_RULES	32
#define NB_CLOCK	5
#define NB_GROUPS	15
#define DEV_NAME_LEN	32
#define CHAN_NAME_LEN	16

#define P7_CHIPREV_R1	0
#define P7_CHIPREV_R2	1
#define P7_CHIPREV_R3	2

struct aai_device_t;
struct card_data_t;

enum aai_dir_t {
	AAI_RX = 1,
	AAI_TX = 2
};

enum aai_format_t {
	AAI_FORMAT_MONO   = 1,
	AAI_FORMAT_STEREO = 2,
	AAI_FORMAT_QUAD   = 4,
	AAI_FORMAT_OCTO   = 8
};

enum aai_fifo_mode_t {
	FIFO_MODE_16 = 1,
	FIFO_MODE_32 = 2
};

enum aai_clk {
	ctrl_clk  = 0,
	i2s_clk   = 1,
	pcm1_clk  = 2,
	pcm2_clk  = 3,
	spdif_clk = 4
};

struct aai_audio_chan_t {
	int		id;
	char		name[CHAN_NAME_LEN];
	uint32_t	fifo_sel_reg;
	int		used;
};

struct aai_hw_ops {
	int	(*open)(struct card_data_t *aai,
			struct aai_device_t *chan);
	int	(*hw_params)(struct card_data_t *aai,
			     struct aai_device_t *chan,
			     struct snd_pcm_hw_params *hw_params);
	int	(*prepare)(struct card_data_t *aai,
			   struct aai_device_t *chan);
	int	(*close)(struct card_data_t *aai,
			 struct aai_device_t *chan);
	int	(*free)(struct card_data_t *aai,
			struct aai_device_t *chan);
};

struct aai_hw_channel_t {
	int			fifo_id;		/*!< fifo id for this channel */
	char			*pstart;		/*!< Ptr to start of transfer buffer */
	char			*pcurrent;		/*!< Ptr to next chunk to transfer */
	char			*pfollow;		/*!< Ptr to following chunk to transfer */
	char			*pend;			/*!< Ptr to end of transfer buffer */
	uint32_t		fifo_conf;		/*!< HW fifo config register */
	uint32_t		dmaca;			/*!< DMA current address */
	uint32_t		dmana;			/*!< DMA next address */
	uint32_t		dmacv;			/*!< DMA current valid ONLY FOR R2,R3 */
	uint32_t		dmanv;			/*!< DMA next valid ONLY FOR R2,R3 */
	uint32_t		memoffset;		/*!< offset in AAI internal memory */
	enum aai_format_t	chan_nb;		/*!< number of mono channels for this device */
	struct aai_audio_chan_t	*chan_list[8];		/*!< list of aai internal audio channels */
	enum aai_fifo_mode_t	mode;			/*!< fifo mode 16-bits or 32-bits */
};

struct aai_device_t {
	/* Misc informations */
	void			*driver_data;		/*!< Ptr to driver data */
	char			name[DEV_NAME_LEN];	/*!< Device name ("spk", ... etc) */
	int			ipcm;			/*!< pcm identifier number */
	enum aai_dir_t		direction;		/*!< RX/TX */
	int			fifo_size;		/*!< Size of FIFO tranfer */

	/* Alsa informations */
	struct snd_pcm_ops	*ops;			/*!< pcm callbacks operators */
	const struct snd_pcm_hardware *hw;		/*!< alsa hardware params pointer */
	uint32_t		period_frames;		/*!< alsa period size in frame */
	uint32_t		period_bytes;		/*!< alsa period size in bytes */
	uint32_t		bytes_count;		/*!< amount of bytes read or written */
	uint32_t		rate;			/*!< Rate value */

	/* Hardware informations */
	struct aai_hw_channel_t	fifo;			/*!< HW fifo informations */

	/* DMA informations */
	uint32_t		dma_burst_nb;		/*!< DMA transfer number per burst */
	uint32_t		dma_xfer_size;		/*!< size of a single DMA transfer */
	char			*dma_area;		/*!< pointer on dma transfer buffer */

	/* Buffer informations */
	uint32_t		bufsize;		/*!< Buffer size (for each fifo) */
	uint32_t		nbirq;			/*!< Number IRQ (Debug only) */
	uint32_t		nbperiod;		/*!< Number period elapsed (Debug only) */

	/* rules */
	int			rules[NB_RULES];
	struct aai_hw_ops	aai_hw_ops;

	/* channel synchronization group */
	int			group;

};

struct card_data_t {
	/* aai resources */
	struct aai_device_t	*chans;
	int			chans_nb;
	struct aai_pad_t	*pad;
	int			irq;

	/* alsa structures */
	struct snd_card		*card;
	struct snd_pcm		**pcms;

	/* memory resources */
	void __iomem		*iobase;
	struct resource		*ioarea;
	struct device		*dev;

	/* hardware dep */
	struct snd_hwdep	*hwdep;
	spinlock_t		mixer_lock;
	spinlock_t		hwlock;

	/* IRQ handle */
	irq_handler_t		irq_handler;

	/* clock */
	struct clk		*clk[NB_CLOCK];
	struct pinctrl		*pin;
	struct aai_platform_data *pdata;


	int			fifo_address;
	int			fifo_tx;
	int			fifo_rx;

	/* groups */
	int			groups_ref_count[NB_GROUPS];	/*!< ref count started groups */
	uint32_t		groups_started;			/*!< started groups reg cache */
};

/**
 * @brief Get pointer to substream's aai-pcm data
 *
 * @param substream
 * @return pointer to aai-pcm data
 */
static inline struct aai_device_t *substream2chan(struct snd_pcm_substream *substream)
{
	return (struct aai_device_t *)(substream->pcm->private_data);
}

/**
 * Get pointer to playback substream
 * We consider that there is only one substream per pcm
 *
 * @param aai private driver data
 * @param ipcm PCM index
 * @return pointer to capture substream
 */
static inline struct snd_pcm_substream *chan2pbacksubstream(struct card_data_t *aai, int ipcm)
{
	return (struct snd_pcm_substream *)((struct snd_pcm *)(aai->pcms)[ipcm]->streams[0].substream);
}

/**
 * Get pointer to capture substream
 * We consider that there is only one substream per pcm
 *
 * @param aai private driver data
 * @param ipcm PCM index
 * @return pointer to capture substream
 */
static inline struct snd_pcm_substream *chan2captsubstream(struct card_data_t *aai, int ipcm)
{
	return (struct snd_pcm_substream *)((struct snd_pcm *)(aai->pcms)[ipcm]->streams[1].substream);
}

#define chan2dev(_chan_) (((struct card_data_t *)_chan_->driver_data)->dev)

#endif

