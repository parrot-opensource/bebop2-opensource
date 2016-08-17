
#ifndef _AAI_HW_STRUCT_H_
#define _AAI_HW_STRUCT_H_

#define DFLT_BUFFER_BYTES_MAX	(128*1024)
#define DFLT_PERIOD_BYTES_MAX	(32*1024)
#define DFLT_PERIOD_MIN		(2)
#define DFLT_PERIOD_MAX		(PAGE_SIZE / 16)

static const struct snd_pcm_hardware aai_pcm_hw_music_stereo_rx = {
	.info             = SNDRV_PCM_INFO_MMAP           |
			    SNDRV_PCM_INFO_MMAP_VALID     |
			    SNDRV_PCM_INFO_INTERLEAVED    |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S24_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.rate_max         = 48000,
	.rate_min         = 48000,
	.channels_min     = 2,
	.channels_max     = 2,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 64,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_music_stereo_tx = {
	.info             = SNDRV_PCM_INFO_MMAP           |
			    SNDRV_PCM_INFO_MMAP_VALID     |
			    SNDRV_PCM_INFO_INTERLEAVED    |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S32_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.rate_max         = 48000,
	.rate_min         = 48000,
	.channels_min     = 2,
	.channels_max     = 2,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 64,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_music = {
	.info             = SNDRV_PCM_INFO_MMAP           |
			    SNDRV_PCM_INFO_MMAP_VALID     |
			    SNDRV_PCM_INFO_INTERLEAVED    |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S32_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.rate_max         = 48000,
	.rate_min         = 48000,
	.channels_min     = 1,
	.channels_max     = 8,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 64,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_music_96k = {
	.info             = SNDRV_PCM_INFO_MMAP           |
			    SNDRV_PCM_INFO_MMAP_VALID     |
			    SNDRV_PCM_INFO_INTERLEAVED    |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S24_LE,
	.rates            = SNDRV_PCM_RATE_96000,
	.rate_max         = 96000,
	.rate_min         = 96000,
	.channels_min     = 1,
	.channels_max     = 8,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 64,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_voice = {
	.info             = SNDRV_PCM_INFO_MMAP           |
			    SNDRV_PCM_INFO_MMAP_VALID     |
			    SNDRV_PCM_INFO_INTERLEAVED    |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S24_LE,
	.rates            = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
	.rate_max         = 16000,
	.rate_min         = 8000,
	.channels_min     = 2,
	.channels_max     = 2,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 8,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_pcm = {
	.info             = SNDRV_PCM_INFO_MMAP             |
			    SNDRV_PCM_INFO_MMAP_VALID       |
			    SNDRV_PCM_INFO_INTERLEAVED      |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER   |
			    SNDRV_PCM_INFO_RESUME,

	/*
	 * for pcm the real format is 16 bits in 32 bits due to hardware
	 * bug (16 bits dma not working).
	 */
	.formats          = SNDRV_PCM_FMTBIT_S32_LE,
	.rates            = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
	.rate_max         = 16000,
	.rate_min         = 8000,
	.channels_min     = 1,
	.channels_max     = 1,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 8,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_ram2ram = {
	.info             = SNDRV_PCM_INFO_MMAP             |
			    SNDRV_PCM_INFO_MMAP_VALID       |
			    SNDRV_PCM_INFO_INTERLEAVED      |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER   |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_S24_LE,
	.rates            = SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_16000,
	.rate_max         = 192000,
	.rate_min         = 44100,
	.channels_min     = 1,
	.channels_max     = 8,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 8,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

static const struct snd_pcm_hardware aai_pcm_hw_iec958 = {
	.info             = SNDRV_PCM_INFO_MMAP             |
			    SNDRV_PCM_INFO_MMAP_VALID       |
			    SNDRV_PCM_INFO_INTERLEAVED      |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER   |
			    SNDRV_PCM_INFO_RESUME,

	.formats          = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME |
			    SNDRV_PCM_FMTBIT_S32_LE,
	.rates            = SNDRV_PCM_RATE_44100 |
			    SNDRV_PCM_RATE_48000 |
			    SNDRV_PCM_RATE_88200 |
			    SNDRV_PCM_RATE_96000,
	.rate_min         = 44100,
	.rate_max         = 96000,
	.channels_min     = 1,
	.channels_max     = 8,
	.buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
	.period_bytes_min = 512,
	.period_bytes_max = DFLT_PERIOD_BYTES_MAX,
	.periods_min      = DFLT_PERIOD_MIN,
	.periods_max      = DFLT_PERIOD_MAX,
};

/* List of AAI internal audio channels RX */
static struct aai_audio_chan_t aai_internal_audio_chans_rx[] = {
	{.id = 0,	.name = "2ram-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(0)},
	{.id = 1,	.name = "2ram-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(1)},
	{.id = 2,	.name = "2ram-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(2)},
	{.id = 3,	.name = "2ram-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(3)},
	{.id = 4,	.name = "2ram-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(4)},
	{.id = 5,	.name = "2ram-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(5)},
	{.id = 6,	.name = "2ram-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(6)},
	{.id = 7,	.name = "2ram-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(7)},

	{.id = 8,	.name = "pcm-in-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(8)},
	{.id = 9,	.name = "pcm-in-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(9)},
	{.id = 10,	.name = "pcm-in-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(10)},
	{.id = 11,	.name = "pcm-in-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(11)},

	{.id = 12,	.name = "mus-in-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(12)},
	{.id = 13,	.name = "mus-in-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(13)},
	{.id = 14,	.name = "mus-in-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(14)},
	{.id = 15,	.name = "mus-in-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(15)},
	{.id = 16,	.name = "mus-in-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(16)},
	{.id = 17,	.name = "mus-in-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(17)},
	{.id = 18,	.name = "mus-in-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(18)},
	{.id = 19,	.name = "mus-in-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(19)},

	{.id = 20,	.name = "micfs-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(20)},
	{.id = 21,	.name = "micfs-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(21)},
	{.id = 22,	.name = "micfs-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(22)},
	{.id = 23,	.name = "micfs-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(23)},
	{.id = 24,	.name = "micfs-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(24)},
	{.id = 25,	.name = "micfs-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(25)},
	{.id = 26,	.name = "micfs-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(26)},
	{.id = 27,	.name = "micfs-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(27)},

	{.id = 28,	.name = "michs-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(28)},
	{.id = 29,	.name = "michs-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(29)},
	{.id = 30,	.name = "michs-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(30)},
	{.id = 31,	.name = "michs-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(31)},
	{.id = 32,	.name = "michs-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(32)},
	{.id = 33,	.name = "michs-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(33)},
	{.id = 34,	.name = "michs-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(34)},
	{.id = 35,	.name = "michs-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(35)},

	{.id = 36,	.name = "binary-in",	.fifo_sel_reg = AAI_FIFOCFG_SEL_RX(36)}
};

/* List of AAI internal audio channels TX */
static struct aai_audio_chan_t aai_internal_audio_chans_tx[] = {
	{.id = 0,	.name = "2ram-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(0)},
	{.id = 1,	.name = "2ram-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(1)},
	{.id = 2,	.name = "2ram-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(2)},
	{.id = 3,	.name = "2ram-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(3)},
	{.id = 4,	.name = "2ram-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(4)},
	{.id = 5,	.name = "2ram-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(5)},
	{.id = 6,	.name = "2ram-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(6)},
	{.id = 7,	.name = "2ram-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(7)},

	{.id = 8,	.name = "pcm-out-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(8)},
	{.id = 9,	.name = "pcm-out-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(9)},
	{.id = 10,	.name = "pcm-out-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(10)},
	{.id = 11,	.name = "pcm-out-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(11)},

	{.id = 12,	.name = "spk-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(12)},
	{.id = 13,	.name = "spk-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(13)},
	{.id = 14,	.name = "spk-2",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(14)},
	{.id = 15,	.name = "spk-3",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(15)},
	{.id = 16,	.name = "spk-4",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(16)},
	{.id = 17,	.name = "spk-5",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(17)},
	{.id = 18,	.name = "spk-6",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(18)},
	{.id = 19,	.name = "spk-7",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(19)},

	{.id = 20,	.name = "voice-0",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(20)},
	{.id = 21,	.name = "voice-1",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(21)},

	{.id = 22,	.name = "binary-out",	.fifo_sel_reg = AAI_FIFOCFG_SEL_TX(22)}
};

#endif

