
#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_struct.h"
#include "aai_rules.h"

/*
 * Static list of audio devices
 * - 11 fifo tx
 * - 18 fifo rx
 * - total internal memory size 4096 * 32bits
 */
struct aai_device_t aai_pcm_devices[] = {
	/* music output channels */
	{
		.name = "music-out-stereo0",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_tx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_tx[12],
				&aai_internal_audio_chans_tx[13]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-out-stereo1",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_tx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_tx[14],
				&aai_internal_audio_chans_tx[15]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-out-stereo2",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_tx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_tx[16],
				&aai_internal_audio_chans_tx[17]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-out-stereo3",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_tx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_tx[18],
				&aai_internal_audio_chans_tx[19]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	/* voice output channels */
	{
		.name = "voice-out-stereo",
		.direction = AAI_TX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_tx[20],
				&aai_internal_audio_chans_tx[21]
			},
			.mode = FIFO_MODE_32,
		},
		.aai_hw_ops = {
			.hw_params = aai_hw_params_voice,
			.close = aai_close_voice,
		},
	},

	/* pcm output channels */
	{
		.name = "pcm0-out",
		.direction = AAI_TX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_pcm,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_tx[8],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
		.aai_hw_ops = {
			.hw_params = aai_hw_params_pcm0,
			.close = aai_close_pcm,
		},
	},

	{
		.name = "pcm1-out",
		.direction = AAI_TX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_pcm,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_tx[9],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
		.aai_hw_ops = {
			.hw_params = aai_hw_params_pcm1,
			.close = aai_close_pcm,
		},
	},

	/* music input channels */
	{
		.name = "music-in-stereo0",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_rx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[12],
				&aai_internal_audio_chans_rx[13]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-in-stereo1",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_rx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[14],
				&aai_internal_audio_chans_rx[15]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-in-stereo2",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_rx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[16],
				&aai_internal_audio_chans_rx[17]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "music-in-stereo3",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_rx,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[18],
				&aai_internal_audio_chans_rx[19]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	/* voice input channels */
	{
		.name = "mic0-8k",
		.direction = AAI_RX,
		.fifo_size = 0x20,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[28],
				&aai_internal_audio_chans_rx[29]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_VOICEL0_INPUT_8K,
			RULE_VOICER0_INPUT_8K,
			RULE_END
		},
	},

	{
		.name = "mic1-8k",
		.direction = AAI_RX,
		.fifo_size = 0x20,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[30],
				&aai_internal_audio_chans_rx[31]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_VOICEL1_INPUT_8K,
			RULE_VOICER1_INPUT_8K,
			RULE_END
		},
	},

	{
		.name = "mic0-16k",
		.direction = AAI_RX,
		.fifo_size = 0x20,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[20],
				&aai_internal_audio_chans_rx[21]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	{
		.name = "mic1-16k",
		.direction = AAI_RX,
		.fifo_size = 0x20,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[22],
				&aai_internal_audio_chans_rx[23]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	/* pcm input channels */
	{
		.name = "pcm0-in",
		.direction = AAI_RX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_pcm,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_rx[8],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
		.aai_hw_ops = {
			.hw_params = aai_hw_params_pcm0,
			.close = aai_close_pcm,
		},
	},

	{
		.name = "pcm1-in",
		.direction = AAI_RX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_pcm,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_rx[9],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
		.aai_hw_ops = {
			.hw_params = aai_hw_params_pcm1,
			.close = aai_close_pcm,
		},
	},

	/* spdif channels */
	{
		.name = "spdif-out",
		.direction = AAI_TX,
		.fifo_size = 0x200,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_iec958,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_tx[22],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_SPDIF_OUT, RULE_END},
	},

	{
		.name = "spdif-in",
		.direction = AAI_RX,
		.fifo_size = 0x200,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_iec958,
		.fifo = {
			.chan_nb = AAI_FORMAT_MONO,
			.chan_list = {
				&aai_internal_audio_chans_rx[36],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {RULE_END},
	},

	/* music complex output channels */
	{
		.name = "music-out-octo",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music,
		.fifo = {
			.chan_nb = AAI_FORMAT_OCTO,
			.chan_list = {
				&aai_internal_audio_chans_tx[12],
				&aai_internal_audio_chans_tx[13],
				&aai_internal_audio_chans_tx[14],
				&aai_internal_audio_chans_tx[15],
				&aai_internal_audio_chans_tx[16],
				&aai_internal_audio_chans_tx[17],
				&aai_internal_audio_chans_tx[18],
				&aai_internal_audio_chans_tx[19]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_OUT_OCTO,
			RULE_END
		},
	},

	{
		.name = "music-out-quad0",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music,
		.fifo = {
			.chan_nb = AAI_FORMAT_QUAD,
			.chan_list = {
				&aai_internal_audio_chans_tx[12],
				&aai_internal_audio_chans_tx[13],
				&aai_internal_audio_chans_tx[14],
				&aai_internal_audio_chans_tx[15],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_OUT_QUAD,
			RULE_END
		},
	},

	{
		.name = "music-out-quad1",
		.direction = AAI_TX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music,
		.fifo = {
			.chan_nb = AAI_FORMAT_QUAD,
			.chan_list = {
				&aai_internal_audio_chans_tx[16],
				&aai_internal_audio_chans_tx[17],
				&aai_internal_audio_chans_tx[18],
				&aai_internal_audio_chans_tx[19],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_OUT_QUAD,
			RULE_END
		},
	},

	/* music complex input channels */
	{
		.name = "music-in-octo",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music,
		.fifo = {
			.chan_nb = AAI_FORMAT_OCTO,
			.chan_list = {
				&aai_internal_audio_chans_rx[12],
				&aai_internal_audio_chans_rx[13],
				&aai_internal_audio_chans_rx[14],
				&aai_internal_audio_chans_rx[15],
				&aai_internal_audio_chans_rx[16],
				&aai_internal_audio_chans_rx[17],
				&aai_internal_audio_chans_rx[18],
				&aai_internal_audio_chans_rx[19]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_IN_OCTO,
			RULE_TDM_IN0,
			RULE_TDM_IN1,
			RULE_TDM_IN2,
			RULE_TDM_IN3,
			RULE_END
		},
	},

	{
		.name = "music-in-quad0",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music,
		.fifo = {
			.chan_nb = AAI_FORMAT_QUAD,
			.chan_list = {
				&aai_internal_audio_chans_rx[12],
				&aai_internal_audio_chans_rx[13],
				&aai_internal_audio_chans_rx[14],
				&aai_internal_audio_chans_rx[15],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_IN_QUAD,
			RULE_TDM_IN0,
			RULE_TDM_IN1,
			RULE_END
		},
	},

	{
		.name = "music-in-quad1",
		.direction = AAI_RX,
		.fifo_size = 0x100,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_music_stereo_rx,
		.fifo = {
			.chan_nb = AAI_FORMAT_QUAD,
			.chan_list = {
				&aai_internal_audio_chans_rx[16],
				&aai_internal_audio_chans_rx[17],
				&aai_internal_audio_chans_rx[18],
				&aai_internal_audio_chans_rx[19],
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_TDM_IN_QUAD,
			RULE_TDM_IN0,
			RULE_TDM_IN1,
			RULE_END
		},
	},

	/* loopbacks channels */
	{
		.name = "loopback-8k",
		.direction = AAI_RX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[32],
				&aai_internal_audio_chans_rx[33]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_LOOPBACK_2,
			RULE_LOOPBACK2_I2S0,
			RULE_VOICEL2_INPUT_8K,
			RULE_VOICER2_INPUT_8K,
			RULE_END
		},
	},

	{
		.name = "loopback-16k",
		.direction = AAI_RX,
		.fifo_size = 0x40,
		.ops = &aai_pcm_ops,
		.hw = &aai_pcm_hw_voice,
		.fifo = {
			.chan_nb = AAI_FORMAT_STEREO,
			.chan_list = {
				&aai_internal_audio_chans_rx[24],
				&aai_internal_audio_chans_rx[25]
			},
			.mode = FIFO_MODE_32,
		},
		.rules = {
			RULE_LOOPBACK_2,
			RULE_LOOPBACK2_I2S0,
			RULE_END
		},
	},
};
