/*
 * Galileo I register map
 *
 * @author Eng-Hong Sron <eng-hong.sron@parrot.com>
 * @date   Sun May  4 13:01:40 CEST 2014
 *
 */
#ifndef _GALILEO2_REG_H_
#define _GALILEO2_REG_H_

/* I2C parameters */
#define PMIC_CHIPID                 0x2102
#define GALILEO2_CHIPID             0x2170

/*
 * PMIC: MAX14638
 */
#define HLAF_WINDOWS_CONTROL	    0x3480
#define HLAF_WINDOWS_STATUS	    0x3481 /* status register */
#define SLAVE_I2C_BUS_CONFIG1	    0x3482
#define SLAVE_I2C_BUS_CONFIG2	    0x3483
#define HLAFD_SLV_ADR		    0x3484
#define HLAFD_SUB_ADR		    0x3485
#define HLAFD_RW_DATA		    0x3486
#define BYTE_ACCESS_ENABLE	    0x3487
#define MC_ID			    0x3490 /* Manufacturer and Chip ID */
#define V_ID			    0x3491 /* Version ID */
#define SYSCTRL			    0x3492 /* System Control */
#define ISET_H			    0x3493 /* VCM Output Current MSBs */
#define ISET_L			    0x3494 /* VCM Output Current LSBs */
#define STAT			    0x3495 /* Diagnostic /Protection Status */
#define RING_CFG		    0x3496 /* Ring Comp Configuration */
#define VCMTM_H			    0x3497 /* VCM Resonance Period MSBs */
#define VCMTM_L			    0x3498 /* VCM Resonance Period LSBs */
#define DIGCFG1			    0x3499 /* DC-DC Config1 */
#define DIGCFG2			    0x349A /* DC-DC Config2 */
#define VTMIN_H			    0x349B /* VCM Self-Tuning Min Res MSBs */
#define VTMIN_L			    0x349C /* VCM Self-Tuning Min Res LSBs */
#define RES			    0x349D /* RESERVED */
#define VTCFG			    0x349E /* VCM Self-Tuning Configuration */
#define VTCUR			    0x349F /* VCM Self-Tuning Current */
#define VTRES			    0x34A0 /* VCM Self-Tuning Resonance */
#define VTTIME			    0x34A1 /* VCM Self-Tuning Timings */
#define FRAM0			    0x34A2 /* Filter Engine Data0 */
#define FRAM1			    0x34A3 /* Filter Engine Data1 */
#define FRAM2			    0x34A4 /* Filter Engine Data2 */
#define FRAM3			    0x34A5 /* Filter Engine Data3 */
#define FRAM4			    0x34A6 /* Filter Engine Data4 */
#define FRAM5			    0x34A7 /* Filter Engine Data5 */
#define FRAM6			    0x34A8 /* Filter Engine Data6 */
#define FRAM7			    0x34A9 /* Filter Engine Data7 */

/* high-level AutoFocus interface, 16bit registers */
#define FOCUS_CHANGE		    0x0D80 /* RW */
#define FOCUS_CHANGE_CONTROL	    0x0D82 /* RW */
#define FOCUS_CHANGE_NUMBER_PHASE1  0x0D84 /* RW */
#define FOCUS_CHANGE_NUMBER_PHASE2  0x0D86 /* RW */
#define STROBE_COUNT_PHASE1	    0x0D88 /* RW */
#define STROBE_COUNT_PHASE2	    0x0D89 /* RW */
#define POSITION		    0x0D8A /* Read Only */

union focus_change_control {
	struct {
		uint8_t enable   : 1;
		uint8_t async    : 1;
		uint8_t frames   : 1;
		uint8_t strobes  : 1;
		uint8_t phase    : 1;
		uint8_t dir      : 1;
		uint8_t measpos  : 1;
		uint8_t movemeas : 1;
	};
	uint8_t _register;
};

union mech_shutter_control {
	struct {
		uint8_t enable        : 1;
		uint8_t demux         : 1;
		uint8_t control_level : 1;
		uint8_t swreset       : 1;
	};
	uint8_t _register;
};

union operation_mode {
	struct {
		uint8_t smode     : 1;
		uint8_t sedge     : 2;
		uint8_t sdirm_cfg : 1;
		uint8_t nmode     : 1;
		uint8_t nedge     : 2;
		uint8_t ndirm_cfg : 1;
	};
	uint8_t _register;
};

union drive_cfg {
	struct {
		uint8_t mclk_div : 3;
		uint8_t polarity : 1;
		uint8_t h_bridge : 1;
		uint8_t slope    : 3;
	};
	uint8_t _register;
};

union buck_cfg {
	struct {
		uint8_t b_en   : 1;
		uint8_t b_mode : 1;
	};
	uint8_t _register;
};

union timing {
	struct {
		uint8_t rsvd     : 4;
		uint8_t shontime : 4;
	};
	uint8_t _register;
};

union config {
	struct {
		unsigned /* unused */ : 2;
		uint8_t  vldo         : 3;
	};
	uint8_t _register;
};

union nr_risefall {
	struct {
		uint8_t ns_330 : 1;
		uint8_t ns_360 : 1;
		uint8_t ns_385 : 1;
		uint8_t ns_410 : 1;
		uint8_t ns_490 : 1;
		uint8_t ns_560 : 1;
		uint8_t ns_680 : 1;
		uint8_t ns_950 : 1;
	};
	uint8_t _register;
};

union nr_rf_pulse {
	struct {
		uint8_t nr_rf_pulse : 4;
	};
	uint8_t _register;
};

union status {
	struct {
		uint8_t over_temp : 1;
		uint8_t low_vbatt : 1;
		uint8_t busy      : 1;
	};
	uint8_t _register;
};

/*
 * Sensor
 */
#define MODEL_ID                           0x0000
#define SENSOR_MODEL_ID                    0x0016

/* General Set-up */
#define MODE_SELECT                        0x0100
#define IMAGE_ORIENTATION                  0x0101
#define SOFTWARE_RESET                     0x0103
#define GROUPED_PARAMETER_HOLD             0x0104
#define MASK_CORRUPTED_FRAME               0x0105
#define FAST_STANDBY_CTRL                  0x0106
#define CCI_ADDRESS_CONTROL                0x0107
#define CSI_CHANNEL_IDENTIFIER             0x0110
#define CSI_SIGNALING_MODE                 0x0111
#define CSI_DATA_FORMAT_SOURCE             0x0112
#define CSI_DATA_FORMAT_DESTINATION        0x0113
#define CSI_LANE_MODE                      0x0114
#define CSI_10_TO_8_DT                     0x0115
#define CSI_10_TO_6_DT                     0x0117
#define CSI_12_T0_8_DT                     0x0118
#define CSI_12_T0_6_DT                     0x011A

union image_orientation {
	struct {
		uint8_t h_mirror : 1;
		uint8_t v_mirror : 1;
	};
	uint8_t _register;
};

/* Operating Voltage */
#define VANA_VOLTAGE                       0x0130
#define VDIG_VOLTAGE                       0x0132
#define VIO_VOLTAGE                        0x0134
#define EXTCLK_FRQ_MHZ                     0x0136

/* Integration Time */
#define FINE_INTEGRATION_TIME              0x0200
#define COARSE_INTEGRATION_TIME            0x0202

/* Analog gain */
#define ANALOG_GAIN_CODE_GLOBAL            0x0204

/* Clock Set-up */
#define VT_PIX_CLK_DIV                     0x0300
#define VT_SYS_CLK_DIV                     0x0302
#define PRE_PLL_CLK_DIV                    0x0304
#define PLL_MULTIPLIER                     0x0306
#define OP_PIX_CLK_DIV                     0x0308
#define OP_SYS_CLK_DIV                     0x030A

/* Frame Timing Register */
#define VT_FRAME_LENGTH_LINES              0x0340
#define VT_LINE_LENGTH_PCK                 0x0342

/* Image Size */
#define X_ADDR_START                       0x0344
#define Y_ADDR_START                       0x0346
#define X_ADDR_END                         0x0348
#define Y_ADDR_END                         0x034A
#define X_OUTPUT_SIZE                      0x034C
#define Y_OUTPUT_SIZE                      0x034E

/* Imsage Scaling */
#define SCALING_MODE                       0x0400
#define SPATIAL_SAMPLING                   0x0402
#define DIGITAL_CROP_X_OFFSET              0x0408
#define DIGITAL_CROP_Y_OFFSET              0x040A
#define DIGITAL_CROP_IMAGE_WIDTH           0x040C
#define DIGITAL_CROP_IMAGE_HEIGHT          0x040E

/* CSI-2 Configuration */
#define DPHY_CTRL                          0x0808
#define REQUESTED_LINK_BIT_RATE_MBPS_31_24 0x0820
#define REQUESTED_LINK_BIT_RATE_MBPS_23_16 0x0821
#define REQUESTED_LINK_BIT_RATE_MBPS_15_8  0x0822
#define REQUESTED_LINK_BIT_RATE_MBPS_7_0   0x0823

/* Binning Configuration */
#define BINNING_MODE                       0x0900
#define BINNING_TYPE                       0x0901
#define BINNING_WEIGHTING                  0x0902

/* Data Upload/Download Configuration */
#define DATA_TRANSFER_IF_1_CTRL            0x0A00
#define DATA_TRANSFER_IF_1_STATUS          0x0A01
#define DATA_TRANSFER_IF_1_PAGE_SELECT     0x0A02
#define DATA_TRANSFER_IF_1_DATA            0x0A04

/* Non-Volatile Memory */
#define NVM_PAGE_NB 32
#define NVM_PAGE_SZ 64

#define NVM_SIZE (NVM_PAGE_NB * NVM_PAGE_SZ)

#define NVM_VERSION        0x21

#define NVM_MEMORY_ADDRESS 0x0003

union data_transfer_if_1_status {
	struct {
		uint8_t read_if_ready     : 1;
		uint8_t write_if_ready    : 1;
		uint8_t data_corrupted    : 1;
		uint8_t improper_if_usage : 1;
	};
	uint8_t _register;
};

/* Due to alignement constraint, even if the set of register is on 7 bytes, we
 * declare it on 8 bytes on purpose.
 */
union nvm_memaddr {
	struct {
		uint64_t dummy    : 10;
		uint64_t af       : 12;
		uint64_t defect   : 12;
		uint64_t ls       :  7;
		uint64_t nd       :  5;
		uint64_t ms       :  5;
		uint64_t checksum : 13;
	};
	uint64_t _registers;
};

union nvm_af {
	struct {
		uint16_t near_end;
		uint16_t macro;
		uint16_t infinity;
		uint16_t far_end;
	};
	uint64_t _registers;
};

/* Ideal Raw */
#define MAPPED_COUPLET_CORRECT_ENABLE                 0x0B05
#define SINGLE_DEFECT_CORRECT_ENABLE                  0x0B06
#define SINGLE_DEFECT_CORRECT_WEIGHT                  0x0B07
#define COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_ENABLE 0x0B0A
#define COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_WEIGHT 0x0B0B
#define MAPPED_LINE_DEFECT_CORRECT_ENABLE             0x0B0E

/* Timer Configuration */
#define GLOBAL_RESET_CTRL1                 0x0C00
#define GLOBAL_RESET_CTRL2                 0x0C01
#define GLOBAL_RESET_MODE_CONFIG1          0x0C02
#define GLOBAL_RESET_MODE_CONFIG2          0x0C03
#define TRDY_CTRL                          0x0C04
#define TRDOUT_CTRL                        0x0C06
#define TSHUTTER_STROBE_DELAY_CTRL         0x0C08
#define TSHUTTER_STROBE_WIDTH_CTRL         0x0C0A
#define TFLASH_STROBE_DELAY_CTRL           0x0C0C
#define TFLASH_STROBE_WIDTH_HIGH_CTRL      0x0C0E
#define TGRST_INTERVAL_CTRL                0x0C10
#define TFLASH_STROBE_ADJUSTMENT           0x0C12
#define TFLASH_STROBE_START_POINT          0x0C14
#define TFLASH_STROBE_DELAY_RS_CTRL        0x0C16
#define TFLASH_STROBE_WIDTH_HIGH_RS_CTRL   0x0C18
#define FLASH_MODE_RS                      0x0C1A
#define FLASH_TRIGGER_RS                   0x0C1B
#define FLASH_STATUS                       0x0C1C

union global_reset_mode_ctrl1 {
	struct {
		uint8_t start_global_reset  : 1;
		uint8_t start_long_exposure : 1;
	};
	uint8_t _register;
};

union global_reset_mode_config1 {
	struct {
		uint8_t vf_to_glbrst                : 1;
		uint8_t glbrst_to_vf                : 1;
		uint8_t readout_start               : 1;
		uint8_t long_exposure_mode          : 1;
		uint8_t continous_global_reset_mode : 1;
		uint8_t flash_strobe                : 1;
		uint8_t sstrobe_muxing              : 1;
		uint8_t sastrobe_muxing             : 1;
	};
	uint8_t _register;
};

#if 0

/* PLL for ADC parameters */
#define AD_CNTL                            0x3381
#define ST_CNTL                            0x3383
#define PRE_PLL_CNTL_ST                    0x3385
#define PLL_MULTI_ST                       0x3386
#define PLL_HRG_CNTL                       0x3388
#define HREG_CNTL                          0x3389
#define HREG_PLLSEL_SINGLE                 0x338A
#define BST_CNTL                           0x338C
#define OPCK_PLLSEL                        0x35C4

/* New Scaler Parameters */
#define GREEN_AVERAGED_BAYER               0x3404
#define FILTER_COEFFICIENT_CONTROL_X       0x3405
#define FILTER_COEFFICIENT_CONTROL_Y       0x3406
#define HORIZONTAL_DIGITAL_BINNING         0x3407
#define VERTICAL_DIGITAL_BINNING           0x3408
#define SCALER_BLANKING_PCK                0x3409
#define X_SCALE_RATIO                      0x3410
#define Y_SCALE_RATIO                      0x3412

/* Row Noise Improve Setting */
#define BLC_SEL                            0x3206
#define CSI2_DELAY                         0x3584

/* No documentation... */
#define POSBSTSEL                          0x3000
#define READVDSEL                          0x3002
#define RSTVDSEL                           0x3004
#define BSVBPSEL                           0x3006
#define HREG_TEST                          0x300C
#define DRESET                             0x3015
#define FRACEXP_TIME1                      0x3021
#define PORTGRESET_U                       0x302A
#define PORTGRESET_W                       0x302C
#define ROREAD                             0x302F
#define DRCUT                              0x3039
#define GDMOSCNT                           0x3042
#define CDS_STOPBST                        0x3064
#define BSTCKLFIX_ADC                      0x3067
#define BSC_AGRNG2                         0x30E0
#define BSC_AGRNG1                         0x30E1
#define BSC_AGRNG0                         0x30E2
#define KBIASCNT_RNG32                     0x30E3
#define KBIASCNT_RNG10                     0x30E4
#define GDMOSHEN                           0x30E6
#define BSDIGITAL_MODE                     0x30EA
#define PS_VZS_NML_COEF                    0x310D
#define PS_VZS_NML_INTC                    0x310E
#define ZSV_IN_LINES                       0x3110
#define FBC_IN_RANGE                       0x3114
#define OB_CLPTHRSH_NEAR                   0x3202
#define OB_CLPTHRSH_FAR                    0x3203
#define WKUP_WAIT_ON                       0x3300
#define HALF_VTAP_MODE                     0x340B
#define HALF_DRATE_LIMIT                   0x340C
#define CCP2BLKD                           0x3580

#endif

/* White Gain */
#define DM_LOW_GAIN_WHITE                  0x3280
#define DM_MID_GAIN_WHITE                  0x3281
#define DM_HIGH_GAIN_WHITE                 0x3282

/* Black Gain */
#define DM_LOW_GAIN_BLACK                  0x3283
#define DM_MID_GAIN_BLACK                  0x3284
#define DM_HIGH_GAIN_BLACK                 0x3285
#define OUTPUT_IMAGE_WIDTH                 0x3400
#define OUTPUT_IMAGE_HEIGHT                0x3402

/* TEMPERATURE SENSOR */
#define TEMP_SENSOR_OUTPUT                 0x013A

#endif /* _GALILEO2_REG_H_ */
