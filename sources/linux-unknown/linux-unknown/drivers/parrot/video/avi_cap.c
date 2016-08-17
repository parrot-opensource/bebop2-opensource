#include <linux/export.h>
#include <linux/seq_file.h>
#include "avi_compat.h"
#include "avi_cap.h"

/*******************************************************************************
 * Helpers
 *******************************************************************************/

/* Check that caps does not contain more than one capability set in exclusive.
 *
 * Returns 1 if there's a conflict, 0 otherwise.
 */
static inline int avi_cap_conflicts(unsigned long caps,
                                    unsigned long exclusive)
{
	unsigned long c = caps & exclusive;

	/* Here we have a conflict if more than one bit of c is set. That means
	 * the Hamming weight of c must be <= 1 but we can do simpler: if only
	 * one bit is set then c is a power of two and then we can use the
	 * simple (c & (c - 1)) == 0 test to check that. This power of two test
	 * has one false positive when c == 0, but since 0 is also acceptable in
	 * this case (not a conflict) we're all set */
	return (c & (c - 1)) != 0;
}

/*******************************************************************************
 * Getters
 *******************************************************************************/

/* Iterate over an inclusive range of node and return the first unused node.
 *
 * If end < start we iterate backwards.
 *
 * If no available node is found NULL is returned.
 *
 * Needs to be called with avi_ctrl.cfg_lck held
 */
struct avi_node *avi_cap_find_node(enum avi_node_id start,
                                   enum avi_node_id end)
{
	int     incr = (end >= start) ? 1 : -1;
	int     i    = start;

	for (;;) {
		struct avi_node *node = avi_get_node(i);

		if (!node->busy)
			return node;

		if (i == end)
			/* No node found */
			return NULL;

		i += incr;
	}
}

inline static struct avi_node * avi_cap_find_single(enum avi_node_id id)
{
	return avi_cap_find_node(id,id);
}

/* Return a scaler with both entry available or NULL is no scaler found
*/
static struct avi_node * avi_cap_get_scaler(void)
{
	struct avi_node *scal, *scal_planar;
	enum avi_node_id id = AVI_SCAL00_NODE;

	while (id <= AVI_SCAL10_NODE) {
		scal            = avi_get_node(id);
		scal_planar     = avi_get_node(id+1);

		if ((scal->busy) || (scal_planar->busy))
			id += (AVI_SCAL10_NODE-AVI_SCAL00_NODE);
		else
			return scal;
	}
	return NULL;
}

/* FIFO */

#define FIFO_NR (AVI_FIFO11_NODE - AVI_FIFO00_NODE + 1)

#define FIFO_00 (1 << 0)
#define FIFO_01 (1 << 1)
#define FIFO_02 (1 << 2)
#define FIFO_03 (1 << 3)
#define FIFO_04 (1 << 4)
#define FIFO_05 (1 << 5)
#define FIFO_06 (1 << 6)
#define FIFO_07 (1 << 7)
#define FIFO_08 (1 << 8)
#define FIFO_09 (1 << 9)
#define FIFO_10 (1 << 10)
#define FIFO_11 (1 << 11)

/* P7 FIFOs have different capabilities with chip revision. */

#define AVI_FIFO_CAPS_VL        (FIFO_00 | FIFO_01 | FIFO_02 | FIFO_03 | \
                                 FIFO_04 | FIFO_05 | FIFO_06 | FIFO_07 | \
                                 FIFO_08 | FIFO_09 | FIFO_10 | FIFO_11)

#define AVI_FIFO_CAPS_IN        (FIFO_00 | FIFO_01 | FIFO_02 | FIFO_03 | \
		                 FIFO_04 | FIFO_05 | FIFO_06 | FIFO_07 | \
                                 FIFO_08)

#define AVI_FIFO_CAPS_OUT_R1    (FIFO_00 | FIFO_01 | FIFO_02 | FIFO_03 | \
                                 FIFO_04 | FIFO_09)

#define AVI_FIFO_CAPS_OUT_R2    (AVI_FIFO_CAPS_OUT_R1 | (FIFO_10))

#define AVI_FIFO_CAPS_OUT_R3    (AVI_FIFO_CAPS_OUT_R2 | (FIFO_05 | FIFO_11))

/* Search for an available FIFO with specified capacity.
 *
 * The exclusion list allow to find several different FIFO before using them.
 *
 * Search is from top to bottom because :
 *      - We should use in priority FIFO with less features
 *      - Bottom FIFO 0 and 1 have 8ko instead of 4ko, we may want to request
 *        them appart.
 */
static struct avi_node * avi_cap_get_fifo(unsigned long caps)
{
	int i;
	struct avi_node *node;

	/* Starting from top, check adapted FIFO busy state */
	for ( i = (FIFO_NR -1); i >= 0; i--) {
		if (!((1 << i) & caps))
			continue;

		node = avi_get_node(AVI_FIFO00_NODE + i);
		if (!node->busy)
			return node;
	}
	return NULL;
}

/* Return the node of a FIFO able to perform DMA out
 * NULL is returned if no FIFO found
 */
#ifdef AVI_BACKWARD_COMPAT
static struct avi_node * avi_cap_get_dmaout(void)
{

	switch(avi_get_revision()) {
	case AVI_REVISION_1:
		return avi_cap_get_fifo(AVI_FIFO_CAPS_OUT_R1);
	case AVI_REVISION_2:
		return avi_cap_get_fifo(AVI_FIFO_CAPS_OUT_R2);
	case AVI_REVISION_3:
		return avi_cap_get_fifo(AVI_FIFO_CAPS_OUT_R3);
	default:
		BUG_ON(1);
	}
	return NULL;
}
#else /* AVI_BACKWARD_COMPAT */
static inline struct avi_node * avi_cap_get_dmaout(void)
{
	return avi_cap_get_fifo(AVI_FIFO_CAPS_OUT_R3);
}
#endif /* AVI_BACKWARD_COMPAT */

/*******************************************************************************
 * API
 *******************************************************************************/

int avi_cap_check(unsigned long caps)
{
	/* A segment can have only one input */
	if (avi_cap_conflicts(caps, AVI_CAPS_CAM_ALL | AVI_CAP_DMA_IN))
		return -EINVAL;

	/* and one output */
	if (avi_cap_conflicts(caps, AVI_CAPS_LCD_ALL | AVI_CAP_DMA_OUT))
		return -EINVAL;

	/* planar only makes sense for DMA input and scaler */
	if ((caps & AVI_CAP_PLANAR) &&
	    (!(caps & AVI_CAP_DMA_IN) && !(caps & AVI_CAP_SCAL)))
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(avi_cap_check);

/* Returns a non-busy node with given capability.
 * Returns NULL if there is no node available with given capability, if many
 * capabilities are specified or for unknown capabilities.
 *
 *
 */
struct avi_node * avi_cap_get_node(unsigned long cap)
{
	switch(cap) {
	case AVI_CAP_BUFFER:
		return avi_cap_get_fifo(AVI_FIFO_CAPS_VL);
	case AVI_CAP_PLANAR:
	case AVI_CAP_DMA_IN:
		return avi_cap_get_fifo(AVI_FIFO_CAPS_IN);
	case AVI_CAP_DMA_OUT:
		return avi_cap_get_dmaout();
	case AVI_CAP_SCAL:
		return avi_cap_get_scaler();
	case AVI_CAP_CONV:
		return avi_cap_find_node(AVI_CONV0_NODE,AVI_CONV3_NODE);
	case AVI_CAP_GAM:
		return avi_cap_find_node(AVI_GAM0_NODE,AVI_GAM1_NODE);
	case AVI_CAP_CAM_0:
		return avi_cap_find_single(AVI_CAM0_NODE);
	case AVI_CAP_CAM_1:
		return avi_cap_find_single(AVI_CAM1_NODE);
	case AVI_CAP_CAM_2:
		return avi_cap_find_single(AVI_CAM2_NODE);
	case AVI_CAP_CAM_3:
		return avi_cap_find_single(AVI_CAM3_NODE);
	case AVI_CAP_CAM_4:
		return avi_cap_find_single(AVI_CAM4_NODE);
	case AVI_CAP_CAM_5:
		return avi_cap_find_single(AVI_CAM5_NODE);
	case AVI_CAP_LCD_0:
		return avi_cap_find_single(AVI_LCD0_NODE);
	case AVI_CAP_LCD_1:
		return avi_cap_find_single(AVI_LCD1_NODE);
	case AVI_CAP_ISP_CHAIN_BAYER:
		return avi_cap_find_node(AVI_ISP_CHAIN_BAYER0_NODE,
					 AVI_ISP_CHAIN_BAYER1_NODE);
	case AVI_CAP_STATS_YUV:
		return avi_cap_find_node(AVI_STATS_YUV0_NODE,
					 AVI_STATS_YUV1_NODE);
	case AVI_CAP_ISP_CHAIN_YUV:
		return avi_cap_find_node(AVI_ISP_CHAIN_YUV0_NODE,
					 AVI_ISP_CHAIN_YUV1_NODE);
	case AVI_CAP_STATS_BAYER_0:
		return avi_cap_find_single(AVI_STATS_BAYER0_NODE);
	case AVI_CAP_STATS_BAYER_1:
		return avi_cap_find_single(AVI_STATS_BAYER1_NODE);
	default:
		return NULL;
	}
}
EXPORT_SYMBOL(avi_cap_get_node);

inline struct avi_node *avi_cap_get_blender(void)
{
	return avi_cap_find_node(AVI_BLEND0_NODE,AVI_BLEND1_NODE);
}
EXPORT_SYMBOL(avi_cap_get_blender);
