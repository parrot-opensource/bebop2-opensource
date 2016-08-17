#include <linux/module.h>

#include "avi_compat.h"

/***************************
 * P7R1 Compatibility layer
 ***************************/

static void avi_compat_r1_conv_write(u32 val, u32 addr)
{
	u32 off = addr & 0xff;

	if (off <= AVI_ISP_CHROMA_COEFF_22) {
		/* The coeff matrix layout didn't change */
		__raw_writel(val, addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_RY) {
		union avi_isp_chroma_offset_ry reg = { ._register = val};
		u32 r1_addr = (addr & ~0xff) + 0x14;
		u32 r1_offsets = __raw_readl(r1_addr);

		r1_offsets &= 0xff00ff;
		r1_offsets |= (reg.offset_in & 0xff)  << 8;
		r1_offsets |= (reg.offset_out & 0xff) << 24;

		__raw_writel(r1_offsets, r1_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_CLIP_RY) {
		union avi_isp_chroma_clip_ry reg = { ._register = val};
		u32 r1_addr = (addr & ~0xff) + 0x18;
		u32 r1_clip = __raw_readl(r1_addr);

		r1_clip &= 0xffff;
		r1_clip |= (reg.clip_min & 0xff) << 16;
		r1_clip |= (reg.clip_max & 0xff) << 24;

		__raw_writel(r1_clip, r1_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_GU) { /* Offset GU */
		union avi_isp_chroma_offset_gu reg = { ._register = val};
		u32 r1_addr = (addr & ~0xff) + 0x14;
		u32 r1_offsets = __raw_readl(r1_addr);

		r1_offsets &= 0xff00ff00;
		r1_offsets |= (reg.offset_in & 0xff)  << 0;
		r1_offsets |= (reg.offset_out & 0xff) << 16;

		__raw_writel(r1_offsets, r1_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_CLIP_GU) {
		union avi_isp_chroma_clip_gu reg = { ._register = val};
		u32 r1_addr = (addr & ~0xff) + 0x18;
		u32 r1_clip = __raw_readl(r1_addr);

		r1_clip &= 0xffff0000;
		r1_clip |= (reg.clip_min & 0xff) << 0;
		r1_clip |= (reg.clip_max & 0xff) << 8;

		__raw_writel(r1_clip, r1_addr);
		return;
	}

	/* The Offsets and Clip values of the GU and BV components are not
	 * separate in P7R1. We use the GU value for both above, so we just
	 * ignore all writes to the BV regs */
	return;
}

static u32 avi_compat_r1_conv_read(u32 addr)
{
	u32 off = addr & 0xff;

	if (off <= AVI_ISP_CHROMA_COEFF_22) {
		/* The coeff matrix layout didn't change */
		return __raw_readl(addr);
	}

	if (off == AVI_ISP_CHROMA_OFFSET_RY) { /* Offset RY */
		union avi_isp_chroma_offset_ry reg;

		u32 r1_addr = (addr & ~0xff) + 0x14;
		u32 r1_offsets = __raw_readl(r1_addr);

		reg.offset_in  = (r1_offsets >> 8)  & 0xff;
		reg.offset_out = (r1_offsets >> 24) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_CLIP_RY) { /* Clip RY */
		union avi_isp_chroma_clip_ry reg;

		u32 r1_addr = (addr & ~0xff) + 0x18;
		u32 r1_clip = __raw_readl(r1_addr);

		reg.clip_min = (r1_clip >> 16) & 0xff;
		reg.clip_max = (r1_clip >> 24) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_GU ||
	    off == AVI_ISP_CHROMA_OFFSET_BV) {
		union avi_isp_chroma_offset_gu reg;

		u32 r1_addr = (addr & ~0xff) + 0x14;
		u32 r1_offsets = __raw_readl(r1_addr);

		reg.offset_in  = (r1_offsets >> 0)  & 0xff;
		reg.offset_out = (r1_offsets >> 16) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_CLIP_GU ||
	    off == AVI_ISP_CHROMA_CLIP_BV) {
		union avi_isp_chroma_clip_gu reg;

		u32 r1_addr = (addr & ~0xff) + 0x18;
		u32 r1_clip = __raw_readl(r1_addr);

		reg.clip_min = (r1_clip >> 0) & 0xff;
		reg.clip_max = (r1_clip >> 8) & 0xff;

		return reg._register;
	}

	/* Unknown register */
	return __raw_readl(addr);
}

static inline void avi_compat_r1_gam_write_reg(u32 val, u32 addr)
{
	/* RTL workarounds: AVI R1 slave locks when it receives bursts to the
	 * gamma CLUT. To prevent that, I added a memory barrier to force all
	 * entries to be written one by one. There's an other bug that causes
	 * silent failres when accessing (read/write) the CLUT RAM. To make sure
	 * the writel is effective, I loop it until readl returns the correct
	 * value. Needless to say, since both writel *and* readl can fail, it
	 * can loop a while before settling, so expect poor performances. NOT
	 * FOOLPROOF!*/

	do {
		wmb();
		__raw_writel(val, addr);
		wmb();
	} while (readl(addr) != (val));
}

static void avi_compat_r1_gam_write(u32 val, u32 addr)
{
	int gam  = !!(addr & 0x4000);
	u32 bank = addr &  0x3000;
	u32 off  = addr &  0x0fff;

	/* The LUTs are smaller in R1 because there's no 10 bit
	 * support. Therefore the offsets are different. */
	u32 base_r1 = (addr & ~0x7fffUL) + gam * 0x1000;

	switch (bank) {
	case AVI_ISP_GAMMA_CORRECTOR_CONF:
		if (off == AVI_ISP_GAMMA_CORRECTOR_CONF) {
			union avi_isp_gamma_corrector_conf cfg =
				{ ._register = val };

			/* R1 ISP does not support 10 bit mode */
			BUG_ON(cfg.comp_width);

			avi_compat_r1_gam_write_reg(val, base_r1);
			break;
		}
		/* Nothing here */
		BUG_ON(val);
		break;
	case AVI_ISP_GAMMA_CORRECTOR_RY_LUT:
	case AVI_ISP_GAMMA_CORRECTOR_GU_LUT:
	case AVI_ISP_GAMMA_CORRECTOR_BV_LUT:
		/* P7 R1 has smaller LUTs */
		if ((off >> 2) > 0xff)
			BUG_ON(val);
		else
			avi_compat_r1_gam_write_reg(val,
						    base_r1 + (bank >> 2)
						    + off);
		break;
	default:
		BUG();
	}
}

static inline u32 avi_compat_r1_gam_read_reg(u32 addr)
{
	/* RTL Workaroud: same problem as avi_compat_r1_gam_write_reg above. I
	 * read the register until I get twice the same value in a row. NOT
	 * FOOLPROOF! */
	u32 r, rprev;

	r = readl(addr);
	do {
		rprev = r;
		r = readl(addr);
	} while (r != rprev);

	return r;
}

static u32 avi_compat_r1_gam_read(u32 addr)
{
	int gam  = !!(addr & 0x4000);
	u32 bank = addr &  0x3000;
	u32 off  = addr &  0x0fff;

	/* The LUTs are smaller in R1 because there's no 10 bit
	 * support. Therefore the offsets are different. */
	u32 base_r1 = (addr & ~0x7fffUL) + gam * 0x1000;

	switch (bank) {
	case AVI_ISP_GAMMA_CORRECTOR_CONF:
		if (off == AVI_ISP_GAMMA_CORRECTOR_CONF) {
			return avi_compat_r1_gam_read_reg(base_r1);
			break;
		}
		/* Nothing here */
		return 0;
		break;
	case AVI_ISP_GAMMA_CORRECTOR_RY_LUT:
	case AVI_ISP_GAMMA_CORRECTOR_GU_LUT:
	case AVI_ISP_GAMMA_CORRECTOR_BV_LUT:
		/* P7 R1 has smaller LUTs */
		if ((off >> 2) > 0xff)
			return 0;
		else
			return avi_compat_r1_gam_read_reg(base_r1 + (bank >> 2)
							  + off);
		break;
	default:
		return 0;
	}
}

void avi_compat_r1_write(u32 val, u32 addr)
{
	u32 block = addr & 0x1ff00;

	/* Match each block in ascending order of the AVI reg map */
	if (block < 0x100)
		/* CFG */
		goto write;
	else if (block < 0x200)
		/* INTER */
		goto write;
	else if (block < 0x1000)
		/* ISP INTER, not implemented */
		BUG_ON(val);
	else if (block < 0x3000)
		/* FIFO */
		goto write;
	else if (block < 0x4000)
		/* CONV */
		avi_compat_r1_conv_write(val, addr);
	else if (block < 0x5000)
		/* BLEND */
		goto write;
	else if (block < 0x6000)
		/* LCD */
		goto write;
	else if (block < 0x7000)
		/* CAM */
		goto write;
	else if (block < 0x8000)
		/* SCALROT */
		goto write;
	else if (block < 0x10000)
		avi_compat_r1_gam_write(val, addr);
	else
		goto write;

	return;
write:
	__raw_writel(val, addr);
}
EXPORT_SYMBOL(avi_compat_r1_write);

u32 avi_compat_r1_read(u32 addr)
{
	u32 block = addr & 0x1ff00;

	/* Match each block in ascending order of the AVI reg map */
	if (block < 0x100)
		/* CFG */
		goto read;
	else if (block < 0x200)
		/* INTER */
		goto read;
	else if (block < 0x1000)
		/* ISP INTER, not implemented */
		goto read;
	else if (block < 0x3000)
		/* FIFO */
		goto read;
	else if (block < 0x4000)
		/* CONV */
		return avi_compat_r1_conv_read(addr);
	else if (block < 0x5000)
		/* BLEND */
		goto read;
	else if (block < 0x6000)
		/* LCD */
		goto read;
	else if (block < 0x7000)
		/* CAM */
		goto read;
	else if (block < 0x8000)
		/* SCALROT */
		goto read;
	else if (block < 0x10000)
		/* GAM */
		return avi_compat_r1_gam_read(addr);
	else
		goto read;
read:

	return __raw_readl(addr);
}
EXPORT_SYMBOL(avi_compat_r1_read);

/***************************
 * P7R2 Compatibility layer
 ***************************/
union avi_cfg_r2_isp_enable3
{
	struct
	{
		u32 inter0       : 1;
		u32 inter1       : 1;
		u32 rip0         : 1;
		u32 rip1         : 1;
		u32 anti_vign0   : 1;
		u32 anti_vign1   : 1;
		u32 chroma_aber0 : 1;
		u32 chroma_aber1 : 1;
		u32 denoise_ee0  : 1;
		u32 denoise_ee1  : 1;
		u32 bayer0       : 1;
		u32 bayer1       : 1;
		u32 drop0        : 1;
		u32 drop1        : 1;
	};
	u32 _register;
};

static void avi_compat_r2_cfg_write(u32 val, u32 addr)
{
	u32 off      = addr &  0xff;
	u32 i3d_addr = addr & ~0xff;

	switch (off) {
	case AVI_CFG_ISP_ENABLE3:
		i3d_addr += 0x04;
		goto write;
	case AVI_CFG_ISP_APPLY3:
		i3d_addr += 0x0C;
		goto write;
	case AVI_CFG_ISP_LOCK3:
		i3d_addr += 0x14;
		goto write;
	case AVI_CFG_ISP_ITFLG3:
		i3d_addr += 0x24;
		goto write;
	case AVI_CFG_ISP_ITEN3:
		i3d_addr += 0x2C;
		goto write;
	case AVI_CFG_ISP_ITACK3:
		i3d_addr += 0x34;
write:
	{
		union avi_cfg_isp_enable3 reg = { ._register = val};

		union avi_cfg_r2_isp_enable3 r2 = {
			.inter0       = reg.inter0_enable,
			.inter1       = reg.inter1_enable,
			.rip0         = reg.rip0_enable,
			.rip1         = reg.rip1_enable,
			.anti_vign0   = reg.lsc0_enable,
			.anti_vign1   = reg.lsc1_enable,
			.chroma_aber0 = reg.chroma_aber0_enable,
			.chroma_aber1 = reg.chroma_aber1_enable,
			.denoise_ee0  = reg.denoise0_enable,
			.denoise_ee1  = reg.denoise1_enable,
			.bayer0       = reg.bayer0_enable,
			.bayer1       = reg.bayer1_enable,
			.drop0        = reg.drop0_enable,
			.drop1        = reg.drop1_enable,
		};

		writel(r2._register, addr);
		writel(readl(i3d_addr)           |
		       reg.i3d_lut0_enable << 28 |
		       reg.i3d_lut1_enable << 29, i3d_addr);
	}
	break;

	default:
		writel(val, addr);
		return;
	}
}

static u32 avi_compat_r2_cfg_read(u32 addr)
{
	u32 off      = addr &  0xff;
	u32 i3d_addr = addr & ~0xff;

	switch (off) {
	case AVI_CFG_ISP_ENABLE3:
		i3d_addr += 0x04;
		goto read;
	case AVI_CFG_ISP_APPLY3:
		i3d_addr += 0x0C;
		goto read;
	case AVI_CFG_ISP_LOCK3:
		i3d_addr += 0x14;
		goto read;
	case AVI_CFG_ISP_ITFLG3:
		i3d_addr += 0x24;
		goto read;
	case AVI_CFG_ISP_ITEN3:
		i3d_addr += 0x2C;
		goto read;
	case AVI_CFG_ISP_ITACK3:
		i3d_addr += 0x34;
read:
	{
		union avi_cfg_isp_enable3 reg = {
			._register = 0,
		};

		union avi_cfg_r2_isp_enable3 r2 = {
			._register = readl(addr),
		};

		reg.inter0_enable       = r2.inter0;
		reg.inter1_enable       = r2.inter1;
		reg.rip0_enable         = r2.rip0;
		reg.rip1_enable         = r2.rip1;
		reg.lsc0_enable         = r2.anti_vign0;
		reg.lsc1_enable         = r2.anti_vign1;
		reg.chroma_aber0_enable = r2.chroma_aber0;
		reg.chroma_aber1_enable = r2.chroma_aber1;
		reg.denoise0_enable     = r2.denoise_ee0;
		reg.denoise1_enable     = r2.denoise_ee1;
		reg.bayer0_enable       = r2.bayer0;
		reg.bayer1_enable       = r2.bayer1;
		reg.drop0_enable        = r2.drop0;
		reg.drop1_enable        = r2.drop1;
		reg.i3d_lut0_enable     = (readl(i3d_addr) >> 28) & 0x1;
		reg.i3d_lut1_enable     = (readl(i3d_addr) >> 29) & 0x1;

		return reg._register;
	}
	default:
		return readl(addr);
	}
}

union avi_inter_ids
{
	u8  ids[4];
	u32 _register;
};

/* ID Translation between P7R2 and P7R3 */
static inline u32 avi_compat_r2_inter_id_write(u32 val)
{
	int i;

	union avi_inter_ids reg = { ._register = val };

	for (i = 0 ; i < 4 ; i++) {
		if (reg.ids[i] == 0x2f)
			reg.ids[i] = 0x2b;
		else if (reg.ids[i] == 0x30)
			reg.ids[i] = 0x2c;
	}

	return reg._register;
}

static inline u32 avi_compat_r2_inter_id_read(u32 val)
{
	int i;

	union avi_inter_ids reg = { ._register = val };

	for (i = 0 ; i < 4 ; i++) {
		if (reg.ids[i] == 0x2b)
			reg.ids[i] = 0x2f;
		else if (reg.ids[i] == 0x2c)
			reg.ids[i] = 0x30;
	}

	return reg._register;
}

static void avi_compat_r2_inter_write(u32 val, u32 addr)
{
	u32 off = addr & 0xff;

	val = avi_compat_r2_inter_id_write(val);

	switch (off) {
	case AVI_INTER_SINKK:
	{
		u32 r3_stats_yuv_src       =  val        & 0xffff;
		u32 r3_isp_chain_bayer_src = (val >> 16) & 0xffff;
		u32 r2_i3d_lut_src         = readl(addr) & 0xffff0000;

		writel(r2_i3d_lut_src | r3_stats_yuv_src, addr);
		writel(r3_isp_chain_bayer_src, addr + 4);
	}
	break;
	case AVI_INTER_SINKL:
	{
		u32 r3_isp_chain_yuv_src = (val & 0xffff) << 16;
		u32 r2_stats_yuv_src     = readl(addr - 4) & 0xffff;

		writel(r3_isp_chain_yuv_src | r2_stats_yuv_src, addr - 4);
	}
	break;
	default:
		writel(val, addr);
	}
}

static u32 avi_compat_r2_inter_read(u32 addr)
{
	u32 off = addr & 0xff;
	u32 reg;

	switch (off) {
	case AVI_INTER_SINKK:
	{
		u32 r3_stats_yuv_src       = readl(addr) & 0xffff;
		u32 r3_isp_chain_bayer_src = readl(addr + 4) << 16;

		reg = (r3_isp_chain_bayer_src | r3_stats_yuv_src);
	}
	break;
	case AVI_INTER_SINKL:
	{
		u32 r3_isp_chain_yuv_src = readl(addr - 4) >> 16;

		reg = r3_isp_chain_yuv_src;
	}
	break;
	default:
		reg = readl(addr);
	}

	return avi_compat_r2_inter_id_read(reg);
}

static void avi_compat_r2_conv_write(u32 val, u32 addr)
{
	u32 off = addr & 0xff;

	if (off <= AVI_ISP_CHROMA_COEFF_22) {
		/* The coeff matrix layout didn't change */
		writel(val, addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_RY) {
		union avi_isp_chroma_offset_ry reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x14;
		u32 r2_ry = readl(r2_addr);

		r2_ry &= 0xffff;
		r2_ry |= (reg.offset_in & 0xff)  << 16;
		r2_ry |= (reg.offset_out & 0xff) << 24;

		writel(r2_ry, r2_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_CLIP_RY) {
		union avi_isp_chroma_clip_ry reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x14;
		u32 r2_ry = readl(r2_addr);

		r2_ry &= 0xffff0000;
		r2_ry |= (reg.clip_min & 0xff) << 0;
		r2_ry |= (reg.clip_max & 0xff) << 8;

		writel(r2_ry, r2_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_GU) {
		union avi_isp_chroma_offset_gu reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x18;
		u32 r2_gu = readl(r2_addr);

		r2_gu &= 0xffff;
		r2_gu |= (reg.offset_in & 0xff)  << 16;
		r2_gu |= (reg.offset_out & 0xff) << 24;

		writel(r2_gu, r2_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_CLIP_GU) {
		union avi_isp_chroma_clip_gu reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x18;
		u32 r2_gu = readl(r2_addr);

		r2_gu &= 0xffff0000;
		r2_gu |= (reg.clip_min & 0xff) << 0;
		r2_gu |= (reg.clip_max & 0xff) << 8;

		writel(r2_gu, r2_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_BV) {
		union avi_isp_chroma_offset_bv reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x1c;
		u32 r2_bv = readl(r2_addr);

		r2_bv &= 0xffff;
		r2_bv |= (reg.offset_in & 0xff)  << 16;
		r2_bv |= (reg.offset_out & 0xff) << 24;

		writel(r2_bv, r2_addr);
		return;
	}

	if (off == AVI_ISP_CHROMA_CLIP_BV) {
		union avi_isp_chroma_clip_bv reg = { ._register = val};
		u32 r2_addr = (addr & ~0xff) + 0x1c;
		u32 r2_bv = readl(r2_addr);

		r2_bv &= 0xffff0000;
		r2_bv |= (reg.clip_min & 0xff) << 0;
		r2_bv |= (reg.clip_max & 0xff) << 8;

		writel(r2_bv, r2_addr);
		return;
	}

	return;
}

static u32 avi_compat_r2_conv_read(u32 addr)
{
	u32 off = addr & 0xff;

	if (off <= AVI_ISP_CHROMA_COEFF_22) {
		/* The coeff matrix layout didn't change */
		return readl(addr);
	}

	if (off == AVI_ISP_CHROMA_OFFSET_RY) {
		union avi_isp_chroma_offset_ry reg;

		u32 r2_addr = (addr & ~0xff) + 0x14;
		u32 r2_ry = readl(r2_addr);

		reg.offset_in  = (r2_ry >> 16) & 0xff;
		reg.offset_out = (r2_ry >> 24) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_CLIP_RY) {
		union avi_isp_chroma_clip_ry reg;

		u32 r2_addr = (addr & ~0xff) + 0x14;
		u32 r2_ry = readl(r2_addr);

		reg.clip_min = (r2_ry >> 0) & 0xff;
		reg.clip_max = (r2_ry >> 8) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_GU) {
		union avi_isp_chroma_offset_gu reg;

		u32 r2_addr = (addr & ~0xff) + 0x18;
		u32 r2_gu = readl(r2_addr);

		reg.offset_in  = (r2_gu >> 16) & 0xff;
		reg.offset_out = (r2_gu >> 24) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_CLIP_GU) {
		union avi_isp_chroma_clip_gu reg;

		u32 r2_addr = (addr & ~0xff) + 0x18;
		u32 r2_gu = readl(r2_addr);

		reg.clip_min = (r2_gu >> 0) & 0xff;
		reg.clip_max = (r2_gu >> 8) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_OFFSET_BV) {
		union avi_isp_chroma_offset_bv reg;

		u32 r2_addr = (addr & ~0xff) + 0x1c;
		u32 r2_bv = readl(r2_addr);

		reg.offset_in  = (r2_bv >> 16) & 0xff;
		reg.offset_out = (r2_bv >> 24) & 0xff;

		return reg._register;
	}

	if (off == AVI_ISP_CHROMA_CLIP_BV) {
		union avi_isp_chroma_clip_bv reg;

		u32 r2_addr = (addr & ~0xff) + 0x1c;
		u32 r2_bv = readl(r2_addr);

		reg.clip_min = (r2_bv >> 0) & 0xff;
		reg.clip_max = (r2_bv >> 8) & 0xff;

		return reg._register;
	}

	/* Unknown register */
	return readl(addr);
}

union avi_isp_inter_r2_src0
{
	struct
	{
		u32 isp_out_src      : 3;
		unsigned /*unused */ : 5;
		u32 rip_src          : 3;
		unsigned /*unused */ : 5;
		u32 lsc_src          : 3;
		unsigned /*unused */ : 5;
		u32 chroma_aber_src  : 3;
	};
	u32 _register;
};

union avi_isp_inter_r2_src1
{
	struct
	{
		u32 denoise_src      : 3;
		unsigned /*unused */ : 5;
		u32 bayer_src        : 3;
		unsigned /*unused */ : 5;
		u32 drop_src         : 3;
	};
	u32 _register;
};

/*
 * ISP interconnect in R2 is a partial interconnect, based on SRC and SINK
 * selection (like the main interconnect), whereas in R3, it is a simple
 * 'bypass' register.
 *
 * In P7R2, the ISP order is:
 * RIP -> LSC -> Chroma Abera -> Denoise -> Bayer
 */
static u32 avi_compat_r2_isp_inter_read(u32 addr)
{
	union avi_isp_inter_r2_src0 r2_src0 = {
		._register = readl(addr),
	};

	union avi_isp_inter_r2_src1 r2_src1 = {
		._register = readl(addr + 0x4),
	};

	/* R2 non-implemented modules are considered as bypassed */
	union avi_isp_chain_bayer_inter_module_bypass r3 = {
		.pedestal_bypass     = 1,
		.grim_bypass         = 1,
		.color_matrix_bypass = 1,
	};

	/* We assume a simple scheme where the user won't set the sources id
	 * even if the module is unused. */
	r3.rip_bypass         = (r2_src0.rip_src         == 0);
	r3.lsc_bypass         = (r2_src0.lsc_src         == 0);
	r3.chroma_aber_bypass = (r2_src0.chroma_aber_src == 0);
	r3.denoise_bypass     = (r2_src1.denoise_src     == 0);
	r3.bayer_bypass       = (r2_src1.bayer_src       == 0);

	return r3._register;
}

static void avi_compat_r2_isp_inter_write(u32 val, u32 addr)
{
	union avi_isp_inter_r2_src0 r2_src0 = {
		._register = 0,
	};

	/* Drop will be handle by YUV chain */
	union avi_isp_inter_r2_src1 r2_src1 = {
		._register = 0,
	};

	union avi_isp_chain_bayer_inter_module_bypass r3 = {
		._register = val,
	};

	/* Default source is the input, then iterate of the ISP chain and set
	 * the corresponding source if the module is used. */
	u32 current_src = 1;

	/* In case the whole ISP is bypassed, there is a limitation in P7R2, we
	 * cannot simply connect the input to the output. Thus, we keep the
	 * 'Dead Pixel Correction' module in the chain with a pass-through
	 * configuration. */
	if (r3._register == 0xFF) {
		/* The mapping of ISP chain modules is a little bit 'tricky':
		 *   - Instance 0:
		 *       - isp_inter_0: 0x0200
		 *       - dpc_0:       0xe000
		 *   - Instance 1:
		 *       - isp_inter_1: 0x0300
		 *       - dpc_1:       0xf000
		 */
		int inst = (addr & 0x100) != 0;

		r2_src0.rip_src	    = 0x1;
		r2_src0.isp_out_src = 0x2;

		/* Dead Pixel Correction:
		 * the pixel at (2047, 2047) is marked as 'dead' and the
		 * correction method is 'Copy Right', which will result to a
		 * copy of itself since we are at the border of the image. */
		writel(0xFFFFFD, addr + 0xd00 + (inst * 0xF00));
		writel(0x0,      addr + 0xd00 + (inst * 0xF00) + 0x4);

		/* ISP inter */
		writel(r2_src0._register, addr);
		writel(r2_src1._register, addr + 0x4);

		return;
	}

	if (r3.rip_bypass) {
		r2_src0.rip_src = 0;
		r2_src0.lsc_src = current_src;
	} else {
		r2_src0.rip_src = current_src;
		current_src = 0x2;
	}

	if (r3.lsc_bypass) {
		r2_src0.lsc_src = 0;
		r2_src0.chroma_aber_src = current_src;
	} else {
		r2_src0.lsc_src = current_src;
		current_src = 0x3;
	}

	if (r3.chroma_aber_bypass) {
		r2_src0.chroma_aber_src = 0;
		r2_src1.denoise_src = current_src;
	} else {
		r2_src0.chroma_aber_src = current_src;
		current_src = 0x4;
	}

	if (r3.denoise_bypass) {
		r2_src1.denoise_src = 0;
		r2_src1.bayer_src = current_src;
	} else {
		r2_src1.denoise_src = current_src;
		current_src = 0x5;
	}

	if (r3.bayer_bypass) {
		r2_src1.bayer_src = 0;
	} else {
		r2_src1.bayer_src = current_src;
		current_src = 0x6;
	}

	r2_src0.isp_out_src = current_src;

	writel(r2_src0._register, addr);
	writel(r2_src1._register, addr + 0x4);
}

static u32 avi_compat_r2_isp_chain_bayer_read(u32 addr)
{
	u32 off  = addr &  0x0ffff;
	u32 base = addr & ~0x3ffff;

	/* There are 2 instances of isp_chain_bayer:
	 * 0x20000: Instance 0
	 * 0x30000: Instance 1
	 */
	u32 inst = (addr & 0x10000) != 0;

	/* Match each ISP block in ascending order of the ISP reg map */
	if (off < 0x1000) {
		/* Inter */
		u32 r2_addr = base + 0x200 + (inst * 0x100);

		if (off != 0)
			return 0;
		else
			return avi_compat_r2_isp_inter_read(r2_addr);

	} else if (off < 0x2000) {
		/* VL format 32 to 40: Not implemented in R2 */
		return 0;
	} else if (off < 0x4000) {
		/* Pedestal: Not implemented in R2 */
		return 0;
	} else if (off < 0x6000) {
		/* Green Imbalance: Not implemented in R2 */
		return 0;
	} else if (off < 0x7000) {
		/* Dead Pixel Correction */
		goto read;
	} else if (off < 0x8000) {
		/* Noise Reduction */
		goto read;
	} else if (off < 0xA000) {
		/* Stats Bayer: Not implemented in R2 */
		return 0;
	} else if (off < 0xC000) {
		/* Lens Shading Correction */
		goto read;
	} else if (off < 0xD000) {
		/* Chromatic Aberration */
		goto read;
	} else if (off < 0xE000) {
		/* Bayer */
		u32 r2_addr = base + 0x14000 + (inst * 0x100) + (addr & 0xff);

		return readl(r2_addr);
	} else if (off < 0xF000) {
		/* Color Correction: Not implemented in R2 */
		return 0;
	} else {
		/* VL format 40 to 32: Not implemented in R2 */
		return 0;
	}

read:
	return readl(addr);
}

static void avi_compat_r2_isp_chain_bayer_write(u32 val, u32 addr)
{
	u32 off  = addr &  0x0ffff;
	u32 base = addr & ~0x3ffff;

	/* There are 2 isp_chain_bayer:
	 * 0x20000: Instance 0
	 * 0x30000: Instance 1
	 */
	u32 inst = (addr & 0x10000) != 0;

	/* Match each ISP block in ascending order of the ISP reg map */
	if (off < 0x1000) {
		/* Inter */
		u32 r2_addr = base + 0x200 + (inst * 0x100);

		if (off != 0)
			return;
		else
			avi_compat_r2_isp_inter_write(val, r2_addr);

	} else if (off < 0x2000) {
		/* VL format 32 to 40: Not implemented in R2 */
	} else if (off < 0x4000) {
		/* Pedestal: Not implemented in R2 */
	} else if (off < 0x6000) {
		/* Green Imbalance: Not implemented in R2 */
	} else if (off < 0x7000) {
		/* Dead Pixel Correction */
		BUG_ON(val);
	} else if (off < 0x8000) {
		/* Noise Reduction */
		BUG_ON(val);
	} else if (off < 0xA000) {
		/* Stats Bayer: Not implemented in R2 */
	} else if (off < 0xC000) {
		/* Lens Shading Correction */
		BUG_ON(val);
	} else if (off < 0xD000) {
		/* Chromatic Aberration */
		BUG_ON(val);
	} else if (off < 0xE000) {
		/* Bayer */
		u32 r2_addr = base + 0x14000 + (inst * 0x100) + (addr & 0xff);

		writel(val, r2_addr);
	} else if (off < 0xF000) {
		/* Color Correction: Not implemented in R2 */
	} else {
		/* VL format 40 to 32: Not implemented in R2 */
	}
}

static u32 avi_compat_r2_isp_chain_yuv_read(u32 addr)
{
	u32 off  = addr &  0x0ffff;
	u32 base = addr & ~0x3ffff;

	/* There are 2 instance of isp_chain_yuv:
	 * 0x40000: Instance 0
	 * 0x50000: Instance 1
	 */
	u32 inst = (addr & 0x10000) != 0;

	/* Match each ISP block in ascending order */
	if (off < 0x1000) {
		/* Inter: Not implemented in R2 */
		return 0;
	} else if (off < 0x2000) {
		/* Edge enhancement: Not implemented in R2 */
		return 0;
	} else if (off < 0x3000) {
		/* i3D LUT */
		u32 r2_addr = base + 0x16000 + (inst * 0x1000) + (addr & 0xfff);

		return readl(r2_addr);
	} else {
		/* Drop */
		u32 r2_addr = base + 0x15000 + (inst * 0x100) + (addr & 0xff);

		return readl(r2_addr);
	}
}

static void avi_compat_r2_isp_chain_yuv_write(u32 val, u32 addr)
{
	u32 off  = addr &  0x0ffff;
	u32 base = addr & ~0x3ffff;

	/* There are 2 instance of isp_chain_yuv:
	 * 0x40000: Instance 0
	 * 0x50000: Instance 1
	 */
	u32 inst = (addr & 0x10000) != 0;

	/* Match each ISP block in ascending order */
	if (off < 0x1000) {
		/* Inter: Not implemented in R2 */
	} else if (off < 0x2000) {
		/* Edge enhancement: Not implemented in R2 */
	} else if (off < 0x3000) {
		/* i3D LUT */
		u32 r2_addr = base + 0x16000 + (inst * 0x1000) + (addr & 0xfff);

		writel(val, r2_addr);
	} else {
		/* Drop */
		u32 r2_addr = base + 0x15000 + (inst * 0x100) + (addr & 0xff);

		writel(val, r2_addr);
	}
}

/* There's a bug in R2 that can cause the AVI to drop a transaction within a
 * burst. To work around that we simply forbid bursts by adding memory barriers
 * after each access (that is, the non-__raw functions) */
void avi_compat_r2_write(u32 val, u32 addr)
{
	u32 block = addr & 0x7ff00;

	/* Match each block in ascending order of the AVI reg map */
	if (block < 0x100)
		/* CFG */
		avi_compat_r2_cfg_write(val, addr);
	else if (block < 0x200)
		/* INTER */
		avi_compat_r2_inter_write(val, addr);
	else if (block < 0x3000)
		/* FIFO */
		goto write;
	else if (block < 0x4000)
		/* CONV */
		avi_compat_r2_conv_write(val, addr);
	else if (block < 0x5000)
		/* BLEND */
		goto write;
	else if (block < 0x6000)
		/* LCD */
		goto write;
	else if (block < 0x7000)
		/* CAM */
		goto write;
	else if (block < 0x8000)
		/* SCALROT */
		goto write;
	else if (block < 0x12000)
		/* GAM */
		avi_compat_r1_gam_write(val, addr);
	else if (block < 0x20000)
		/* STATS YUV */
		goto write;
	else if (block < 0x40000)
		/* ISP CHAIN BAYER */
		avi_compat_r2_isp_chain_bayer_write(val, addr);
	else if (block < 0x60000)
		/* ISP CHAIN YUV */
		avi_compat_r2_isp_chain_yuv_write(val, addr);
	else
		goto write;

	return;
write:
	writel(val, addr);
}
EXPORT_SYMBOL(avi_compat_r2_write);

u32 avi_compat_r2_read(u32 addr)
{
	u32 block = addr & 0x7ff00;

	/* Match each block in ascending order of the AVI reg map */
	if (block < 0x100)
		/* CFG */
		return avi_compat_r2_cfg_read(addr);
	else if (block < 0x200)
		/* INTER */
		return avi_compat_r2_inter_read(addr);
	else if (block < 0x3000)
		/* FIFO */
		goto read;
	else if (block < 0x4000)
		/* CONV */
		return avi_compat_r2_conv_read(addr);
	else if (block < 0x5000)
		/* BLEND */
		goto read;
	else if (block < 0x6000)
		/* LCD */
		goto read;
	else if (block < 0x7000)
		/* CAM */
		goto read;
	else if (block < 0x8000)
		/* SCALROT */
		goto read;
	else if (block < 0x12000)
		/* GAM */
		return avi_compat_r1_gam_read(addr);
	else if (block < 0x20000)
		/* STATS YUV */
		goto read;
	else if (block < 0x40000)
		/* ISP CHAIN BAYER */
		return avi_compat_r2_isp_chain_bayer_read(addr);
	else if (block < 0x60000)
		/* ISP CHAIN YUV */
		return avi_compat_r2_isp_chain_yuv_read(addr);
	else
		goto read;
read:

	return readl(addr);
}
EXPORT_SYMBOL(avi_compat_r2_read);
