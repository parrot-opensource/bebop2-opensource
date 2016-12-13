/*********************************************************************
 * avi_lcd register map
 *
 * Vendor:   Parrot
 * Library:  AVI
 * Version:  P7R3
 * Gen-date: (Date of generation of this C code, not the IP-Xact file)
 *           2013-10-02
 *
 * WARNING: This code is automatically generated from the hardware
 * IP-Xact XML files. Do not edit directly.
 *********************************************************************/

#ifndef _AVI_LCD_H_
#define _AVI_LCD_H_

#define AVI_LCD_STATUS                            0x00
#define AVI_LCD_ITSOURCE                          0x04
#define AVI_LCD_INTERFACE                         0x08
#define AVI_LCD_DPD                               0x0c
#define AVI_LCD_TOP_FORMAT_CTRL                   0x10
#define AVI_LCD_TOP_H_TIMING0                     0x20
#define AVI_LCD_TOP_H_TIMING1                     0x24
#define AVI_LCD_TOP_V_TIMING0                     0x30
#define AVI_LCD_TOP_V_TIMING1                     0x34
#define AVI_LCD_TOP_V_TIMING2                     0x38
#define AVI_LCD_TOP_V_TIMING3                     0x3c
#define AVI_LCD_BOT_FORMAT_CTRL                   0x40
#define AVI_LCD_BOT_H_TIMING0                     0x50
#define AVI_LCD_BOT_H_TIMING1                     0x54
#define AVI_LCD_BOT_V_TIMING0                     0x60
#define AVI_LCD_BOT_V_TIMING1                     0x64
#define AVI_LCD_BOT_V_TIMING2                     0x68
#define AVI_LCD_BOT_V_TIMING3                     0x6c
#define AVI_LCD_FORCE_CLEAR                       0x70

union avi_lcd_status
{
	struct
	{
		uint32_t done            :  1;
		uint32_t fuf             :  1;
		uint32_t field           :  1;
	};
	uint32_t _register;
};

union avi_lcd_itsource
{
	struct
	{
		uint32_t done_en         :  1;
		uint32_t fuf_en          :  1;
	};
	uint32_t _register;
};

union avi_lcd_interface
{
	struct
	{
		uint32_t ivs             :  1;
		uint32_t ihs             :  1;
		uint32_t ipc             :  1;
		uint32_t ioe             :  1;
		uint32_t psync_rf        :  1;
		uint32_t psync_en        :  1;
		uint32_t itu656          :  1;
		uint32_t prog            :  1;
		uint32_t clip_en         :  1;
		uint32_t free_run        :  1;
		uint32_t pad_select      :  2;
		uint32_t vsync_gen       :  1;
	};
	uint32_t _register;
};

union avi_lcd_dpd
{
	struct
	{
		uint32_t dpd             : 24;
		uint32_t colorbar        :  2;
	};
	uint32_t _register;
};

union avi_lcd_top_format_ctrl
{
	struct
	{
		uint32_t top_format_control :  5;
	};
	uint32_t _register;
};

union avi_lcd_top_h_timing0
{
	struct
	{
		uint32_t top_hsync_off   : 16;
		uint32_t top_hactive_on  : 16;
	};
	uint32_t _register;
};

union avi_lcd_top_h_timing1
{
	struct
	{
		uint32_t top_hactive_off : 16;
		uint32_t top_htotal      : 16;
	};
	uint32_t _register;
};

union avi_lcd_top_v_timing0
{
	struct
	{
		uint32_t top_vsync_hon   : 16;
		uint32_t top_vsync_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_top_v_timing1
{
	struct
	{
		uint32_t top_vsync_hoff  : 16;
		uint32_t top_vsync_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_top_v_timing2
{
	struct
	{
		uint32_t top_vactive_on  : 16;
		uint32_t top_vactive_off : 16;
	};
	uint32_t _register;
};

union avi_lcd_top_v_timing3
{
	struct
	{
		uint32_t top_vtotal      : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_format_ctrl
{
	struct
	{
		uint32_t bot_format_control :  5;
	};
	uint32_t _register;
};

union avi_lcd_bot_h_timing0
{
	struct
	{
		uint32_t bot_hsync_off   : 16;
		uint32_t bot_hactive_on  : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_h_timing1
{
	struct
	{
		uint32_t bot_hactive_off : 16;
		uint32_t bot_htotal      : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_v_timing0
{
	struct
	{
		uint32_t bot_vsync_hon   : 16;
		uint32_t bot_vsync_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_v_timing1
{
	struct
	{
		uint32_t bot_vsync_hoff  : 16;
		uint32_t bot_vsync_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_v_timing2
{
	struct
	{
		uint32_t bot_vactive_on  : 16;
		uint32_t bot_vactive_off : 16;
	};
	uint32_t _register;
};

union avi_lcd_bot_v_timing3
{
	struct
	{
		uint32_t bot_vtotal      : 16;
	};
	uint32_t _register;
};

union avi_lcd_force_clear
{
	struct
	{
		uint32_t force_clear     :  2;
	};
	uint32_t _register;
};

struct avi_lcd_regs
{
	union avi_lcd_status                     status;                     /* 0x000 */
	union avi_lcd_itsource                   itsource;                   /* 0x004 */
	union avi_lcd_interface                  interface;                  /* 0x008 */
	union avi_lcd_dpd                        dpd;                        /* 0x00c */
	union avi_lcd_top_format_ctrl            top_format_ctrl;            /* 0x010 */
	unsigned                                 /*unused*/ : 32;            /* 0x014 */
	unsigned                                 /*unused*/ : 32;            /* 0x018 */
	unsigned                                 /*unused*/ : 32;            /* 0x01c */
	union avi_lcd_top_h_timing0              top_h_timing0;              /* 0x020 */
	union avi_lcd_top_h_timing1              top_h_timing1;              /* 0x024 */
	unsigned                                 /*unused*/ : 32;            /* 0x028 */
	unsigned                                 /*unused*/ : 32;            /* 0x02c */
	union avi_lcd_top_v_timing0              top_v_timing0;              /* 0x030 */
	union avi_lcd_top_v_timing1              top_v_timing1;              /* 0x034 */
	union avi_lcd_top_v_timing2              top_v_timing2;              /* 0x038 */
	union avi_lcd_top_v_timing3              top_v_timing3;              /* 0x03c */
	union avi_lcd_bot_format_ctrl            bot_format_ctrl;            /* 0x040 */
	unsigned                                 /*unused*/ : 32;            /* 0x044 */
	unsigned                                 /*unused*/ : 32;            /* 0x048 */
	unsigned                                 /*unused*/ : 32;            /* 0x04c */
	union avi_lcd_bot_h_timing0              bot_h_timing0;              /* 0x050 */
	union avi_lcd_bot_h_timing1              bot_h_timing1;              /* 0x054 */
	unsigned                                 /*unused*/ : 32;            /* 0x058 */
	unsigned                                 /*unused*/ : 32;            /* 0x05c */
	union avi_lcd_bot_v_timing0              bot_v_timing0;              /* 0x060 */
	union avi_lcd_bot_v_timing1              bot_v_timing1;              /* 0x064 */
	union avi_lcd_bot_v_timing2              bot_v_timing2;              /* 0x068 */
	union avi_lcd_bot_v_timing3              bot_v_timing3;              /* 0x06c */
	union avi_lcd_force_clear                force_clear;                /* 0x070 */
};

#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_0      0x80
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_OFF_0     0x84
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_1      0x88
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_OFF_1     0x8c
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_2      0x90
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_OFF_2     0x94
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_3      0x98
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_OFF_3     0x9c
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_4      0xa0
#define AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_OFF_4     0xa4

union avi_lcd_vsync_gen_on_0
{
	struct
	{
		uint32_t vsync_gen_hon   : 16;
		uint32_t vsync_gen_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_off_0
{
	struct
	{
		uint32_t vsync_gen_hoff  : 16;
		uint32_t vsync_gen_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_on_1
{
	struct
	{
		uint32_t vsync_gen_hon   : 16;
		uint32_t vsync_gen_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_off_1
{
	struct
	{
		uint32_t vsync_gen_hoff  : 16;
		uint32_t vsync_gen_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_on_2
{
	struct
	{
		uint32_t vsync_gen_hon   : 16;
		uint32_t vsync_gen_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_off_2
{
	struct
	{
		uint32_t vsync_gen_hoff  : 16;
		uint32_t vsync_gen_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_on_3
{
	struct
	{
		uint32_t vsync_gen_hon   : 16;
		uint32_t vsync_gen_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_off_3
{
	struct
	{
		uint32_t vsync_gen_hoff  : 16;
		uint32_t vsync_gen_voff  : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_on_4
{
	struct
	{
		uint32_t vsync_gen_hon   : 16;
		uint32_t vsync_gen_von   : 16;
	};
	uint32_t _register;
};

union avi_lcd_vsync_gen_off_4
{
	struct
	{
		uint32_t vsync_gen_hoff  : 16;
		uint32_t vsync_gen_voff  : 16;
	};
	uint32_t _register;
};

struct avi_lcd_avi_vsync_gen_regs
{
	union avi_lcd_vsync_gen_on_0             vsync_gen_on_0;             /* 0x080 */
	union avi_lcd_vsync_gen_off_0            vsync_gen_off_0;            /* 0x084 */
	union avi_lcd_vsync_gen_on_1             vsync_gen_on_1;             /* 0x088 */
	union avi_lcd_vsync_gen_off_1            vsync_gen_off_1;            /* 0x08c */
	union avi_lcd_vsync_gen_on_2             vsync_gen_on_2;             /* 0x090 */
	union avi_lcd_vsync_gen_off_2            vsync_gen_off_2;            /* 0x094 */
	union avi_lcd_vsync_gen_on_3             vsync_gen_on_3;             /* 0x098 */
	union avi_lcd_vsync_gen_off_3            vsync_gen_off_3;            /* 0x09c */
	union avi_lcd_vsync_gen_on_4             vsync_gen_on_4;             /* 0x0a0 */
	union avi_lcd_vsync_gen_off_4            vsync_gen_off_4;            /* 0x0a4 */
};

#endif /* _AVI_LCD_H_ */
