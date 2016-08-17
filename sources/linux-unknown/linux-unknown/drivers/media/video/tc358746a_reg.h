/*
 * tc358746a register map
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Wed Jul  2 09:16:13 CEST 2014
 *
 */
#ifndef __TC358746A_REG_H__
#define __TC358746A_REG_H__

/* I2C parameters */
#define TC358746A_CHIPID_REV0      0x4400
#define TC358746A_CHIPID           0x4401

/* Clocks constraints */
#define TC358746A_MIN_REFCLK  UL(  6000000)
#define TC358746A_MAX_REFCLK  UL( 40000000)
#define TC358746A_MIN_PCLK    UL(        0)
#define TC358746A_MAX_PCLK    UL(100000000)
#define TC358746A_MIN_PPICLK  UL( 66000000)
#define TC358746A_MAX_PPICLK  UL(125000000)

/* Global */
#define CHIPID            0x0000
#define SYSCTL            0x0002
#define CONFCTL           0x0004
#define FIFOCTL           0x0006
#define DATAFMT           0x0008
#define MCLKCTL           0x000C
#define GPIOEN            0x000E
#define GPIODIR           0x0010
#define GPIOIN            0x0012
#define GPIOOUT           0x0014
#define PLLCTL0           0x0016
#define PLLCTL1           0x0018
#define CLKCTL            0x0020
#define WORDCNT           0x0022
#define CSITX_DT          0x0050
#define PHYCLKCTL         0x0056
#define PHYDATA0CTL       0x0058
#define PHYDATA1CTL       0x005A
#define PHYDATA2CTL       0x005C
#define PHYDATA3CTL       0x005E
#define PHYTIMDLY         0x0060
#define PHYSTA            0x0062
#define CSISTATUS         0x0064
#define CSIERREN          0x0066
#define MDLSYNERR         0x0068
#define CSIDID            0x006A
#define CSIDIDERR         0x006C
#define CSIPKTLEN         0x006E
#define CSIRX_DPCTL       0x0070

/* CSI2-RX Status Counters */
#define FRMERRCNT         0x0080
#define CRCERRCNT         0x0082
#define CORERRCNT         0x0084
#define HDRERRCNT         0x0086
#define EIDERRCNT         0x0088
#define CTLERRCNT         0x008A
#define SOTERRCNT         0x008C
#define SYNERRCNT         0x008E
#define MDLERRCNT         0x0090
#define FIFOSTATUS        0x00F8

/* TX PHY */
#define CLW_DPHYCONTTX    0x0100
#define D0W_DPHYCONTTX    0x0104
#define D1W_DPHYCONTTX    0x0108
#define D2W_DPHYCONTTX    0x010C
#define D3W_DPHYCONTTX    0x0110
#define CLW_CNTRL         0x0140
#define D0W_CNTRL         0x0144
#define D1W_CNTRL         0x0148
#define D2W_CNTRL         0x014C
#define D3W_CNTRL         0x0150
#define STARTCNTRL        0x0204
#define STATUS            0x0208
#define LINEINITCNT       0x0210
#define LPTXTIMECNT       0x0214
#define TCLK_HEADERCNT    0x0218
#define TCLK_TRAILCNT     0x021C
#define THS_HEADERCNT     0x0220
#define TWAKEUP           0x0224
#define TCLK_POSTCNT      0x0228
#define THS_TRAILCNT      0x022C
#define HSTXVREGCNT       0x0230
#define HSTXVREGEN        0x0234

/* TX CTRL */
#define CSI_CONTROL       0x040C
#define CSI_STATUS        0x0410
#define CSI_INT           0x0414
#define CSI_INT_ENA       0x0418
#define CSI_ERR           0x044C
#define CSI_ERR_INTENA    0x0450
#define CSI_ERR_HALT      0x0454
#define CSI_CONFW         0x0500
#define CSI_RESET         0x0504
#define CSI_INT_CLR       0x050C
#define CSI_START         0x0518

/* Debug Tx */
#define DBG_LCNT          0x00e0
#define DBG_WIDTH         0x00e2
#define DBG_VBLANK        0x00e4
#define DBG_DATA          0x00e8

/*
 * Global Registers
 */
union sysctl {
	struct {
		uint16_t sreset : 1;
		uint16_t sleep  : 1;
	};
	uint16_t _register;
};

union confctl {
	struct {
		uint16_t datalane     : 2;
		uint16_t auto_incr    : 1;
		uint16_t pclkp        : 1;
		uint16_t hsyncp       : 1;
		uint16_t vsyncp       : 1;
		uint16_t ppen         : 1;
		unsigned /* unused */ : 1;
		uint16_t pdataf       : 2;
		unsigned /* unused */ : 2;
		uint16_t bt656en      : 1;
		uint16_t inte2n       : 1;
		unsigned /* unused */ : 1;
		uint16_t trien        : 1;
	};
	uint16_t _register;
};

union fifoctl {
	struct {
		uint16_t fifolevel : 9;
	};
	uint16_t _register;
};

union datafmt {
	struct {
		uint16_t udt_en       : 1;
		unsigned /* unused */ : 3;
		uint16_t pdformat     : 4;
	};
	uint16_t _register;
};

union mclkctl {
	struct {
		uint16_t mclk_low  : 8;
		uint16_t mclk_high : 8;
	};
	uint16_t _register;
};

union gpioen {
	struct {
		unsigned /* unused */ :  4;
		uint16_t gpioen       : 12;
	};
	uint16_t _register;
};

union gpiodir {
	struct {
		uint16_t gpiodir : 16;
	};
	uint16_t _register;
};

union gpiopin {
	struct {
		uint16_t gpioin : 16;
	};
	uint16_t _register;
};

union gpioout {
	struct {
		uint16_t gpioout : 16;
	};
	uint16_t _register;
};

union pllctl0 {
	struct {
		uint16_t pll_fbd      : 9;
		unsigned /* unused */ : 3;
		uint16_t pll_prd      : 4;
	};
	uint16_t _register;
};

union pllctl1 {
	struct {
		uint16_t pll_en       : 1;
		uint16_t resetb       : 1;
		unsigned /* unused */ : 2;
		uint16_t cken         : 1;
		uint16_t bypcken      : 1;
		uint16_t lfbren       : 1;
		unsigned /* unused */ : 1;
		uint16_t pll_lbws     : 2;
		uint16_t pll_frs      : 2;
	};
	uint16_t _register;
};

union clkctl {
	struct {
		uint16_t sclkdiv    : 2;
		uint16_t mclkrefdiv : 2;
		uint16_t ppiclkdiv  : 2;
	};
	uint16_t _register;
};


union wordcnt {
	struct {
		uint16_t wordcnt : 16;
	};
	uint16_t _register;
};

union csitx_dt {
	struct {
		uint16_t csitx_dt : 8;
	};
	uint16_t _register;
};

/*
 * Rx control Registers
 */
union phyclkctl {
	struct {
		uint16_t clkdly  : 4;
		uint16_t hsrxrs  : 2;
		uint16_t cap     : 2;
	};
	uint16_t _register;
};

union phydata0ctl {
	struct {
		uint16_t datadly : 4;
		uint16_t hsrxrs  : 2;
		uint16_t cap     : 2;
	};
	uint16_t _register;
};

union phydata1ctl {
	struct {
		uint16_t datadly : 4;
		uint16_t hsrxrs  : 2;
		uint16_t cap     : 2;
	};
	uint16_t _register;
};

union phydata2ctl {
	struct {
		uint16_t datadly : 4;
		uint16_t hsrxrs  : 2;
		uint16_t cap     : 2;
	};
	uint16_t _register;
};

union phydata3ctl {
	struct {
		uint16_t datadly : 4;
		uint16_t hsrxrs  : 2;
		uint16_t cap     : 2;
	};
	uint16_t _register;
};

union phytimdly {
	struct {
		uint16_t dsettle      : 7;
		uint16_t td_term_sel  : 1;
		unsigned /* unused */ : 7;
		uint16_t tc_term_sel  : 1;
	};
	uint16_t _register;
};

union physta {
	struct {
		uint16_t synerr0 : 1;
		uint16_t soterr0 : 1;
		uint16_t synerr1 : 1;
		uint16_t soterr1 : 1;
		uint16_t synerr2 : 1;
		uint16_t soterr2 : 1;
		uint16_t synerr3 : 1;
		uint16_t soterr3 : 1;
	};
	uint16_t _register;
};

union csistatus {
	struct {
		uint16_t synerr : 1;
		uint16_t soterr : 1;
		uint16_t ctlerr : 1;
		uint16_t eiderr : 1;
		uint16_t hdrerr : 1;
		uint16_t corerr : 1;
		uint16_t crcerr : 1;
		uint16_t frmerr : 1;
		uint16_t mdlerr : 1;
	};
	uint16_t _register;
};

union csierren {
	struct {
		uint16_t synen : 1;
		uint16_t soten : 1;
		uint16_t ctlen : 1;
		uint16_t eiden : 1;
		uint16_t hdren : 1;
		uint16_t coren : 1;
		uint16_t crcen : 1;
		uint16_t frmen : 1;
		uint16_t mdlen : 1;
	};
	uint16_t _register;
};

union mdlsynerr {
	struct {
		uint16_t sync0 : 1;
		uint16_t sync1 : 1;
		uint16_t sync2 : 1;
		uint16_t sync3 : 1;
	};
	uint16_t _register;
};

union csidid {
	struct {
		uint16_t datatype : 8;
	};
	uint16_t _register;
};

union csididerr {
	struct {
		uint16_t errtype : 8;
	};
	uint16_t _register;
};

union csipktlen {
	struct {
		uint16_t pktlen : 16;
	};
	uint16_t _register;
};

union csirx_dpctl {
	struct {
		uint16_t rxch0_cntrl : 2;
		uint16_t rxch1_cntrl : 2;
		uint16_t rxch2_cntrl : 2;
		uint16_t rxch3_cntrl : 2;
		uint16_t rxck_cntrl  : 2;
	};
	uint16_t _register;
};

/*
 * Rx Status Register
 */
union frmerrcnt {
	struct {
		uint16_t frmerrcnt : 8;
	};
	uint16_t _register;
};

union crcerrcnt {
	struct {
		uint16_t crcerrcnt : 8;
	};
	uint16_t _register;
};

union corerrcnt {
	struct {
		uint16_t corerrcnt : 8;
	};
	uint16_t _register;
};

union hdrerrcnt {
	struct {
		uint16_t hdrerrcnt : 8;
	};
	uint16_t _register;
};

union eiderrcnt {
	struct {
		uint16_t eiderrcnt : 8;
	};
	uint16_t _register;
};


union ctlerrcnt {
	struct {
		uint16_t ctlerrcnt : 8;
	};
	uint16_t _register;
};

union soterrcnt {
	struct {
		uint16_t soterrcnt : 8;
	};
	uint16_t _register;
};

union synerrcnt {
	struct {
		uint16_t synerrcnt : 8;
	};
	uint16_t _register;
};

union mdlerrcnt {
	struct {
		uint16_t mdlerrcnt : 8;
	};
	uint16_t _register;
};

union fifostatus {
	struct {
		uint16_t vb_oflow : 1;
		uint16_t vb_uflow : 1;
	};
	uint16_t _register;
};

/*
 * TxDebug Register
 */
union dbg_lcnt {
	struct {
		uint16_t db_alcnt     : 10;
		unsigned /* unused */ :  4;
		uint16_t db_cen       :  1;
		uint16_t db_wsram     :  1;
	};
	uint16_t _register;
};

union dbg_width {
	struct {
		uint16_t db_width : 12;
	};
	uint16_t _register;
};

union dbg_vblank {
	struct {
		uint16_t db_vb : 7;
	};
	uint16_t _register;
};

union dbg_data {
	struct {
		uint16_t db_data : 16;
	};
	uint16_t _register;
};

#endif /* __TC358746A_REG_H__ */
