#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/bug.h>

#include "ov16825_common.h"
#include "ov16825_clk.h"

/* The limits on VCO clock comes from the Omnivion timing spreadsheet */
#define MHZ (1000 * 1000)

#define PLL1_VCO_MIN             500 * MHZ
#define PLL1_VCO_MAX            1000 * MHZ
// XXX Value in spreadsheet file from Will
//#define PLL1_VCO_MAX            1300 * MHZ
#define PLL2_VCO_MIN             100 * MHZ
#define PLL2_VCO_MAX             400 * MHZ
#define PLLADC_VCO_MIN           200 * MHZ
#define PLLADC_VCO_MAX           240 * MHZ

#define IS_BETWEEN(_f, _min, _max) ((_f >= _min) && (_f <= _max))

/* PLL Block helpers */

static inline u32 div_sys(u32 val, unsigned char _reg)
{
	unsigned reg = _reg & 0x7;
	if (reg < 4)
		return val;
	return val / (2 * (reg - 3));
}

static inline u32 div_pix(u32 val, unsigned char reg)
{
	return val / ((reg & 0x3)+ 4);
}

static inline u32 div2(u32 val, unsigned char reg, int mask)
{
	if (reg & mask)
		return val / 2;
	return val;
}

static u32 seld5(u32 val, unsigned char _reg)
{
	unsigned reg = _reg & 0x3;
	if (reg == 2)
		return val / 2;
	if (reg == 3)
		return (val * 2) / 5;
	return val;
}

static u32 pre_div(u32 val, unsigned char reg)
{
	return val / ((reg & 0x3) + 1);
}

static u32 pre_div_adc(u32 val, unsigned char _reg)
{
	unsigned reg = _reg & 0x3;
	if (reg == 1)
		return (val * 2) / 3;
	if (reg == 2 || reg == 3)
		return val / reg;
	return val;
}

/* Clocks getters */

/* PLL1 */
static inline u32 ov16825_PLL1_vco_clk(u32 refclk,
                                       struct ov16825_pll *c)
{
	return pre_div(refclk, c->prediv) * c->multi;
}

static inline u32 ov16825_phy_pclk(u32 vco_clk,
                                   struct ov16825_pll *c)
{
	return div_sys(vco_clk, c->op_sys);
}

static inline u32 ov16825_pll_pclk(u32 phy_pclk,
                                   struct ov16825_pll *c)
{
	u32 res = div_pix(phy_pclk, c->op_pix);
	return div2(res, c->op_2lane, BIT(0));
}

static inline u32 ov16825_pll_sclk(u32 vco_clk,
                                   struct ov16825_pll *c)
{
	u32 res = div_sys(vco_clk, c->vt_sys);
	res = div_pix(res, c->vt_pix);
	return div2(res, c->vt_pix, BIT(2));
}

static inline u32 ov16825_pclk(u32 phy_pclk,
                               struct ov16825_reg_conf *c)
{
	u32 res = ov16825_pll_pclk(phy_pclk, &c->pll);
	if (c->r3020 & BIT(3))
		res = res / 2;
	return res;
}

/* Common */

static u32 ov16825_common_pll_clk(u32 prediv_clk,
                                  struct ov16825_pll2 *c,
                                  int dac_mode)
{
	u32 res = div2(prediv_clk, c->rdiv, BIT(0));
	if (dac_mode)
		res = res * (c->multi & 0x1F);
	else
		res = res * (c->multi & 0x3F);

	res = res / (c->divr + 1);
	return  seld5(res, c->seld5);
}

/* PLL2 */

static inline u32 ov16825_pll2_sclk(u32 refclk,
                                    struct ov16825_pll2 *c)
{
	return ov16825_common_pll_clk(pre_div(refclk, c->prediv),
	                              c, 0);
}

/* PLL ADC */

static inline u32 ov16825_dacclk(u32 refclk,
                                 struct ov16825_pll2 *c)
{
	return ov16825_common_pll_clk(pre_div_adc(refclk, c->prediv),
	                              c, 1);
}

/* Debug */

void ov16825_print_pll_common(struct ov16825_pll2 *c)
{
	printk("    PREDIV              = 0x%x\n", c->prediv);
	printk("    RDIV                = 0x%x\n", c->rdiv);
	printk("    MULTI               = 0x%x\n", c->multi);
	printk("    DIVR                = 0x%x\n", c->divr);
	printk("    SELD5               = 0x%x\n", c->seld5);
}

void ov16825_print_configuration(struct ov16825_reg_conf *c)
{
	printk("PLL:\n");
	printk("    PREDIV              = 0x%x\n", c->pll.prediv);
	printk("    MULTI               = 0x%x\n", c->pll.multi);
	printk("    VT_SYS              = 0x%x\n", c->pll.vt_sys);
	printk("    VT_PIX              = 0x%x\n", c->pll.vt_pix);
	printk("    OP_SYS              = 0x%x\n", c->pll.op_sys);
	printk("    OP_PIX              = 0x%x\n", c->pll.op_pix);
	printk("    OP_2LANE            = 0x%x\n", c->pll.op_2lane);
	printk("PLL2:\n");
	ov16825_print_pll_common(&c->pll2);
	printk("PLL ADC:\n");
	ov16825_print_pll_common(&c->adc);
	printk("SC_CMMN_CLOCK_SEL       = 0x%x\n", c->r3020);
	printk("0x3030                  = 0x%x\n", c->r3030);
	printk("SC_CMMN_CORE_CTRL0      = 0x%x\n", c->r3032);
	printk("SC_CMMN_CORE_CTRL1      = 0x%x\n", c->r3033);
	printk("0x3106                  = 0x%x\n", c->r3106);
}

int ov16825_current_configuration(struct v4l2_subdev *sd,
                                  struct ov16825_reg_conf *c)
{
	int ret;

#define READ_REGISTER(_reg, _buf)               \
	ret = sensor_read(sd, _reg, _buf);      \
	if (ret < 0)                            \
		return ret;

	READ_REGISTER(PLL1_CTRL0, &c->pll.prediv);
	READ_REGISTER(PLL1_CTRL2, &c->pll.multi);
	READ_REGISTER(PLL1_CTRL3, &c->pll.vt_sys);
	READ_REGISTER(PLL1_CTRL4, &c->pll.vt_pix);
	READ_REGISTER(PLL1_CTRL5, &c->pll.op_sys);
	READ_REGISTER(PLL1_CTRL6, &c->pll.op_pix);
	READ_REGISTER(PLL1_CTRL7, &c->pll.op_2lane);
	READ_REGISTER(PLL2_CTRL0, &c->pll2.prediv);
	READ_REGISTER(PLL2_CTRL1, &c->pll2.multi);
	READ_REGISTER(PLL2_CTRL2, &c->pll2.rdiv);
	READ_REGISTER(PLL2_CTRL3, &c->pll2.divr);
	READ_REGISTER(PLL2_CTRL4, &c->pll2.seld5);
	READ_REGISTER(PLL3_CTRL0, &c->adc.prediv);
	READ_REGISTER(PLL3_CTRL1, &c->adc.multi);
	READ_REGISTER(PLL3_CTRL2, &c->adc.rdiv);
	READ_REGISTER(PLL3_CTRL3, &c->adc.divr);
	READ_REGISTER(PLL3_CTRL4, &c->adc.seld5);
	READ_REGISTER(SC_CMMN_CLOCK_SEL, &c->r3020);
	READ_REGISTER(0x3030, &c->r3030);
	READ_REGISTER(SC_CMMN_CORE_CTRL0, &c->r3032);
	READ_REGISTER(SC_CMMN_CORE_CTRL1, &c->r3033);
	READ_REGISTER(0x3106, &c->r3106);

	return 0;
}

int ov16825_print_clocks(u32 refclk,
                         struct ov16825_reg_conf *c)
{
	u32 pll_vco_clk;
	u32 phy_pclk;
	u32 pll_pclk;
	u32 pll_sclk;
	u32 pclk;
	u32 pll2_sclk;
	u32 dacclk;

	pll_vco_clk = ov16825_PLL1_vco_clk(refclk, &c->pll);

	phy_pclk = ov16825_phy_pclk(pll_vco_clk, &c->pll);
	pll_pclk = ov16825_pll_pclk(phy_pclk, &c->pll);
	pclk = ov16825_pclk(phy_pclk, c);
	pll_sclk = ov16825_pll_sclk(pll_vco_clk, &c->pll);

	pll2_sclk = ov16825_pll2_sclk(refclk, &c->pll2);

	dacclk = ov16825_dacclk(refclk, &c->adc);

	printk("refclk      = %u Hz\n", refclk);
	printk("pll_vco_clk = %u Hz\n", pll_vco_clk);
	printk("phy_pclk    = %u Hz\n", phy_pclk);
	printk("pll_pclk    = %u Hz\n", pll_pclk);
	printk("pclk        = %u Hz\n", pclk);
	printk("pll_sclk    = %u Hz\n", pll_sclk);
	printk("pll2_sclk   = %u Hz\n", pll2_sclk);
	printk("dacclk      = %u Hz\n", dacclk);

	return 0;
}

static void select_pll_phy_clk(u32 refclk,
                               u32 target_clk,
                               u32 *best_clk,
                               struct ov16825_pll *best_conf,
                               struct ov16825_pll *chalenger)
{
	u32 tmp_clk;
	u32 target_vco_clk;
	u32 multi_tmp;

	tmp_clk = pre_div(refclk, chalenger->prediv);

	if ((chalenger->op_sys & 0x7) < 4)
		target_vco_clk = target_clk;
	else
		target_vco_clk = target_clk * (2 * (chalenger->op_sys - 3));

	multi_tmp = DIV_ROUND_UP(target_vco_clk, tmp_clk);
	if (multi_tmp > 0xFF)
		multi_tmp = 0xFF;

	tmp_clk = ov16825_PLL1_vco_clk(refclk, chalenger);
	tmp_clk = ov16825_phy_pclk(tmp_clk, chalenger);

	if (tmp_clk >= target_clk && (!*best_clk || tmp_clk < *best_clk)) {
		*best_clk = tmp_clk;
		*best_conf = *chalenger;
	}
}

/* common */
static void select_common_clk(u32 refclk,
                              u32 targetclk,
                              u32 *best_clk,
                              struct ov16825_pll2 *best_conf,
                              struct ov16825_pll2 *chalenger,
                              int dac_mode)
{
	u32 tmp_clk;
	u32 target_vco_clk;
	u32 clk;
	u32 multi_tmp;

	if (dac_mode)
		tmp_clk = pre_div_adc(refclk, chalenger->prediv);
	else
		tmp_clk = pre_div(refclk, chalenger->prediv);

	tmp_clk =  div2(tmp_clk, chalenger->multi, BIT(0));

	if (chalenger->seld5 == 0x2)
		target_vco_clk = targetclk * 2;
	else if (chalenger->seld5 == 0x3)
		target_vco_clk = (targetclk * 5) / 2;
	else
		target_vco_clk = targetclk;

	target_vco_clk = target_vco_clk * (chalenger->divr + 1);

	multi_tmp = DIV_ROUND_UP(target_vco_clk, tmp_clk);
	if (dac_mode && multi_tmp > 0x1F)
		multi_tmp = 0x1F;
	else if (!dac_mode && multi_tmp > 0x3F)
		multi_tmp = 0x3F;
	chalenger->multi = multi_tmp;

	if (dac_mode)
		clk = ov16825_dacclk(refclk, chalenger);
	else
		clk = ov16825_pll2_sclk(refclk, chalenger);

	if (clk >= targetclk && (!*best_clk || clk < *best_clk)) {
		*best_clk = clk;
		*best_conf = *chalenger;
	}
}

static int ov16825_set_common_clk(u32 refclk,
                                  u32 target_clk,
                                  struct ov16825_pll2 *c,
                                  int dac_mode)
{
	struct ov16825_pll2 best_conf;
	u32 best_clk = 0;

	for (c->prediv = 0; c->prediv < 4;  c->prediv++)
		for (c->rdiv = 0; c->rdiv < 2;  c->rdiv++)
			for (c->divr = 0; c->divr < 16;  c->divr++)
				for (c->seld5 = 0; c->seld5 < 4;  c->seld5++)
					select_common_clk(refclk,
					                  target_clk,
					                  &best_clk,
					                  &best_conf,
					                  c,
					                  dac_mode);
	if (best_clk == 0)
		return -EINVAL;

	printk("Selecting clock %u Hz for %u Hz target clock\n",
	       best_clk, target_clk);

	*c = best_conf;

	return 0;
}

int ov16825_set_phy_pclk(u32 refclk,
                         u32 target_clk,
                         struct ov16825_reg_conf *c)
{
	struct ov16825_pll      best_conf;
	u32                     best_clk = 0;

	for (c->pll.prediv = 0; c->pll.prediv < 4;  c->pll.prediv++)
		for (c->pll.op_sys = 0; c->pll.op_sys < 8;  c->pll.op_sys++)
			select_pll_phy_clk(refclk,
			                   target_clk,
			                   &best_clk,
			                   &best_conf,
			                   &c->pll);

	if (best_clk == 0)
		return -EINVAL;

	printk("Selecting PHYCLK clock %u Hz for %u Hz target clock\n",
	       best_clk, target_clk);

	c->pll = best_conf;

	return 0;
}

/* This function use div_op_pix, div_2lane and r3020 register to compute a
 * divider allowing to pass from phy_pclk to pclk.
 *      - DIV_OP_PIX can only be 4, 5, 6 or 8
 *      - DIV_2LANE and R3020 can be used to device by 1, 2 or 4
 *      
 * This means that there are only 10 possible dividers :
 *
    ------------------------------------------------------------
 * | DIVIDER    | DIV_OP_PIX    | DIV_2_LANE    | DIV_R3020     |
   |------------+---------------+---------------+---------------|
   |          4 |             4 |             1 |             1 |
   |          5 |             5 |             1 |             1 |
   |          6 |             6 |             1 |             1 |
   |          8 |             8 |             1 |             1 |
   |         10 |             5 |             2 |             1 |
   |         12 |             6 |             2 |             1 |
   |         16 |             8 |             2 |             1 |
   |         20 |             5 |             2 |             2 |
   |         24 |             6 |             2 |             2 |
   |         32 |             8 |             2 |             2 |
    ------------------------------------------------------------
*/
static void ov16825_pclk_closest_conf(u32 div,
                                      struct ov16825_reg_conf *c)
{

	/* We first choose if we need to divide by 2 or 4 */
	if (div < 10) {
		c->pll.op_2lane = OP_2LANE_NODIV;
		c->r3020 = c->r3020 & ~BIT(3);
	} else if (div < 20){
		c->pll.op_2lane = OP_2LANE_DIV2;
		c->r3020 = c->r3020 & ~BIT(3);
		div = div / 2;
	} else {
		c->pll.op_2lane = OP_2LANE_DIV2;
		c->r3020 = c->r3020 | BIT(3);
		div = div / 4;
	}

	/* Then choose the closer value for the rest of the divider. We round
	 * the divider down to get at least the pclk asked. */
	if (div < 5)
		c->pll.op_pix = OP_PIX_DIV4;
	else if (div < 6)
		c->pll.op_pix = OP_PIX_DIV5;
	else if (div < 8)
		c->pll.op_pix = OP_PIX_DIV6;
	else
		c->pll.op_pix = OP_PIX_DIV8;

	/* XXX HACK : The test on board show a problem when we're using a 8
	 * divider in op_pix. As we dont understand this problem yet at the
	 * moment we return only a working 8 divider.*/
	c->pll.op_pix = OP_PIX_DIV4;
	c->pll.op_2lane = OP_2LANE_NODIV;
	c->r3020 = c->r3020 | BIT(3);
}

int ov16825_set_pclk(u32 phy_pclk,
                     u32 target_clk,
                     struct ov16825_reg_conf *c)
{
	u32 pclk;

	ov16825_pclk_closest_conf(phy_pclk / target_clk, c);

	pclk = ov16825_pclk(phy_pclk, c);

	printk("Selecting PCLK clock %u Hz for %u Hz target clock\n",
	       pclk, target_clk);

	if (pclk < target_clk)
		return -EINVAL;

	return 0;
}

int ov16825_set_daccclk(u32 refclk,
                        u32 target_clk,
                        struct ov16825_reg_conf *c)
{
	return ov16825_set_common_clk(refclk,
	                              target_clk,
	                              &c->adc,
	                              1);
}

int ov16825_set_pll2_sclk(u32 refclk,
                          u32 target_clk,
                          struct ov16825_reg_conf *c)
{
	return ov16825_set_common_clk(refclk,
	                              target_clk,
	                              &c->pll2,
	                              0);
}

int ov16825_apply_configuration(struct v4l2_subdev *sd,
                                struct ov16825_reg_conf *c)
{
	int ret;

#define WRITE_REGISTER(_reg, _buf)               \
	ret = sensor_write(sd, _reg, _buf);      \
	if (ret < 0)                            \
		return ret;

	c->r3033 = (c->r3033 & ~SEL_DACCLK_PLL_DACCLK) | SEL_DACCLK_PLL_DACCLK;
	c->r3032 = (c->r3032 & ~SEL_SCLK_PLL2) | SEL_SCLK_PLL2;

	WRITE_REGISTER(PLL1_CTRL0, c->pll.prediv);
	WRITE_REGISTER(PLL1_CTRL2, c->pll.multi);
	WRITE_REGISTER(PLL1_CTRL3, c->pll.vt_sys);
	WRITE_REGISTER(PLL1_CTRL4, c->pll.vt_pix);
	WRITE_REGISTER(PLL1_CTRL5, c->pll.op_sys);
	WRITE_REGISTER(PLL1_CTRL6, c->pll.op_pix);
	WRITE_REGISTER(PLL1_CTRL7, c->pll.op_2lane);
	WRITE_REGISTER(PLL2_CTRL0, c->pll2.prediv);
	WRITE_REGISTER(PLL2_CTRL1, c->pll2.multi);
	WRITE_REGISTER(PLL2_CTRL2, c->pll2.rdiv);
	WRITE_REGISTER(PLL2_CTRL3, c->pll2.divr);
	WRITE_REGISTER(PLL2_CTRL4, c->pll2.seld5);
	WRITE_REGISTER(PLL3_CTRL0, c->adc.prediv);
	WRITE_REGISTER(PLL3_CTRL1, c->adc.multi);
	WRITE_REGISTER(PLL3_CTRL2, c->adc.rdiv);
	WRITE_REGISTER(PLL3_CTRL3, c->adc.divr);
	WRITE_REGISTER(PLL3_CTRL4, c->adc.seld5);
	WRITE_REGISTER(SC_CMMN_CLOCK_SEL, c->r3020);
	//WRITE_REGISTER(0x3030, c->r3030);
	WRITE_REGISTER(SC_CMMN_CORE_CTRL0, c->r3032);
	WRITE_REGISTER(SC_CMMN_CORE_CTRL1, c->r3033);
	//WRITE_REGISTER(0x3106, c->r3106);

	return 0;
}
