#ifndef _AVI_SEGMENT_H_
#define _AVI_SEGMENT_H_

#include "avi.h"
#include "avi_isp.h"
#include "avi_cap.h"
#include "avi_dma.h"
#include "avi_chroma.h"

struct avi_segment;

/* The interrupt callbacks are called directly from interrupt context with the
 * segment's lock held */
typedef unsigned (*avi_segment_irq_handler_t)(struct avi_segment *,
                                              enum avi_field *);

/* A segment can have up to 7 sources (2 blenders in series) */
#define AVI_SEGMENT_MAX_SRC  7
/* A segment can have up to 5 sinks (4 forks in series). */
/* XXX not supported yet, change to 5 when adding fork support */
#define AVI_SEGMENT_MAX_SINK 1

/* The segment's input and output formats */
struct avi_segment_format {
	unsigned		width;
	unsigned		height;

	int                     interlaced;

	enum avi_colorspace     colorspace;

	/* For DMA IN/OUT segments */
	struct avi_dma_pixfmt	pix_fmt;

	struct {
		/* constraint: line_size must be multiple of 8 */
		unsigned	line_size; /* in bytes */
	} plane0, plane1;
};

/* A segment's layout within a blender */
struct avi_segment_layout {
	unsigned x;
	unsigned y;
	int      hidden;
	int      alpha;
};

enum avi_segment_activation {
	AVI_SEGMENT_DISABLED = 0,
	AVI_SEGMENT_ACTIVE,
	/* Segment disabled during interrupt */
	AVI_SEGMENT_ACTIVE_ONESHOT,
};

struct avi_segment {
	struct avi_segment              *source[AVI_SEGMENT_MAX_SRC];
	struct avi_segment              *sink[AVI_SEGMENT_MAX_SINK];
	/* pointers to the blenders used by this segment */
	struct avi_node                 *blender[2];
	/* pointers to the forks used by this segment */
	struct avi_node                 *fork[4];
	/* A bitmask of the segment's capabilities */
	unsigned long                    caps;
	enum avi_segment_activation      active;
	/* Input and output formats */
	struct avi_segment_format        input_format;
	struct avi_segment_format        output_format;
	/* Output offset */
	struct avi_segment_layout        layout;
	/* Input blender background colour. 24bit 888, no alpha */
	u32                              background;
	/* Frame period (if applicable, 0 otherwise). It might make sense to put
	 * that in the segment format but I'm not sure if that's necessary and I
	 * don't want to increase the structure size needlessly. */
	long                             stream_period_us;
	long                             period_us;
	/* Frame dropped (if applicable, 0 otherwise). */
	unsigned long                    frame_dropped;
	/* A list of the segment's nodes ordered the way they're interconnected
	 * in hardware */
	struct avi_node                **nodes;
	unsigned                         nodes_nr;
	/* For DMA nodes we store the base address of both planes */
	dma_addr_t                       plane0_base;
	dma_addr_t                       plane1_base;
	enum avi_field                   cur_field;
	/* DMA IN node used by special case planar : 2xDMA_IN > SCALER */
	struct avi_node                 *dma_in_planar;

	char                             id[16];
	struct list_head                 list;
	/* Lock protecting segment access when (dis)connecting or destroying the
	 * segment */
	struct mutex                     lock;
	/* The device owning this segment */
	struct device                   *owner;
	/* Private data for use by the owner. Can be useful in the interrupt
	 * handler. */
	void                            *priv;
	/* The interrupt callback */
	avi_segment_irq_handler_t        irq_handler_f;
	struct avi_node                 *irq_node;
};

/* Length of the segment id. Last byte is forced to \0 for convenience. */
#define AVI_SEGMENT_ID_LEN   (sizeof(((struct avi_segment *)0)->id))


/* Attempt to create a segment with capabilities "cap".
 *
 * Use IS_ERR and PTR_ERR to check for success and retreive the errno.
 *
 * If errno is ENODEV it means the AVI couldn't allocate the nodes to satisfy
 * the requested capabilities. In this case caps is updated to contain the
 * unsatisfied capabilities.
 *
 * If errno is EBUSY it means there's already a segment with that
 * type/major/minor combination.
 *
 * If owner is not NULL it becomes the segment's owner, otherwise the segment's
 * created as an orphan and avi_take_segment must be called to take ownership.
 *
 * The id of the resulting segment is "avi-<type>.<major>[:minor]"
 *
 * If minor is -1 it's ignored and not part of the id.
 */
extern struct avi_segment *avi_segment_build(unsigned long	*caps,
					     const char		*type,
					     int		 major,
					     int		 minor,
					     struct device	*owner);

/* Generate a new unique id based on "base_id" and "nonce". The result is stored
 * in gen_id. This is usefull for drivers that need to create segments on the
 * fly. */
extern void avi_segment_gen_id(const char *base_id,
                               const char *nonce,
                               char gen_id[AVI_SEGMENT_ID_LEN]);

/* Return 1 if the segment has at least one source or one sink. 0 otherwise. */
extern int avi_segment_connected(struct avi_segment *segment);

/* Destroys a segment.
 *
 * The segment must not be connected to an other segment at the moment this
 * function is called (otherwise -EBUSY is returned).
 *
 * Returns negative errno on error, 0 otherwise.
 *
 * /!\ Locks the segment's mutex
 */
extern int avi_segment_teardown(struct avi_segment *segment);

/* Retreive an existing segment by id.
 *
 * If no segment matching id is found this returns NULL
 */
extern struct avi_segment *avi_segment_find(const char *id);

/* Enables the segment's interrupt.
 *
 * A segment has an interrupt if it contains an LCD, CAM or DMA_OUT capability.
 *
 * It's an error to call this function on a segment that does not have any such
 * capability.
 *
 * If the segment's IRQ is already enabled this does nothing.
 */
extern void avi_segment_enable_irq(struct avi_segment *segment);

/* Disables the segment's interrupt.
 *
 * Same remarks as avi_segment_enable_irq.
 */
extern void avi_segment_disable_irq(struct avi_segment *segment);

/* Registers this segment's interrupt handler.
 *
 * You have to own the segment before you attempt to register an interrupt
 * handler.
 */
extern void avi_segment_register_irq(struct avi_segment *segment,
                                     avi_segment_irq_handler_t handler);

/* Request the ownership of a segment.
 *
 * This is mandatory before attempting to modify the configuration of the
 * segment.
 *
 * owner cannot be NULL.
 *
 * This returns -EBUSY if the segment is already owned, 0 otherwise.
 *
 * /!\ Locks the segment's mutex
 */
extern int avi_take_segment(struct avi_segment *segment, struct device *owner);

/* Release the ownership of a segment.
 *
 * Returns negative errno on error, 0 otherwise.
 *
 * This does not destroy the segment, just remove the ownership. Use
 * avi_teardown_segment to permanently destroy a segment.
 */
extern int avi_release_segment(struct avi_segment *segment);

/* Connect two segments together.
 *
 * If the source segment already has a sink a fork is added to share the output
 * with the new destination.
 *
 * If the destination segment already has a source or the zorder parameter is
 * greater than 0 a blender is added to mix the input. In this case the zorder
 * parameter dictates the layer the source will occupy in the blender. Layer 0
 * is the top-most layer.
 *
 * If zorder is -1 the first available layer is used (starting from 1 upwards).
 *
 * Returns -ENODEV if the connection could not be achieved by lack of fork or
 * blender inputs available (if zorder is >= 0).
 *
 * Returns -EBUSY if the requested zorder is already in use.
 *
 * /!\ Locks both segments' mutexes
 */
extern int avi_segment_connect(struct avi_segment *src,
                               struct avi_segment *dst,
                               int                 zorder);

/* Disconnect two segments
 *
 * Returns -EINVAL if src and dst are not currently connected together.
 *
 * /!\ Locks both segments' mutexes
 */
extern int avi_segment_disconnect(struct avi_segment *src,
                                  struct avi_segment *dst);

/* Set a segment layout on the screen (configures the blender) */
extern int avi_segment_set_position(struct avi_segment *s,
                                    unsigned x,
                                    unsigned y);

/* Set a segment alpha (configures the blender) */
extern int avi_segment_set_alpha(struct avi_segment *s, int alpha);


/* Set the segment background color when no video is displayed (blender
 * background).
 *
 * Color is expressed as RGB and converted according to segment colorspace */
extern void avi_segment_set_background(struct avi_segment *segment, u32 rgb);

/* Return the background color converted to 24bit RGB */
u32 avi_segment_get_background(struct avi_segment *segment);

/* Enable and activate the segment in order to start processing. Depending on
 * the capabilities of the segment that might mean enabling the pixelclock or
 * other capability-specific actions
 */
extern int avi_segment_activate(struct avi_segment *segment);

/*
 * Same as above but it configures the FIFOs for a single frame and then stops
 * the processing.
 */
extern int avi_segment_activate_oneshot(struct avi_segment *segment);

/* Disable the segment */
extern int avi_segment_deactivate(struct avi_segment *segment);

int avi_segment_try_format(struct avi_segment *s,
                           const struct avi_segment_format *in,
                           const struct avi_segment_format *out,
                           const struct avi_segment_layout *layout);

/* Configure a segment's input and output format to the same value (i.e. the
 * segment does not modify the source or destination format). */
extern int avi_segment_set_format(struct avi_segment *segment,
                                  const struct avi_segment_format *in_fmt,
                                  const struct avi_segment_format *out_fmt);

/* Same as above but sets the layout as well. */
extern int avi_segment_set_format_and_layout(struct avi_segment *segment,
                                             const struct avi_segment_format *i,
                                             const struct avi_segment_format *o,
                                             const struct avi_segment_layout *l);

/* Configure a segment's input format (i.e. the
 * segment does not modify the source or destination format). */
extern int avi_segment_set_input_format(struct avi_segment *segment,
                                        const struct avi_segment_format *fmt);

/* Configure a segment's output format (i.e. the
 * segment does not modify the source or destination format). */
extern int avi_segment_set_output_format(struct avi_segment *segment,
                                         const struct avi_segment_format *fmt);

/* Hide a segment (move it outside of the active frame). Requires a blender.*/
extern int avi_segment_hide(struct avi_segment *segment);

/* Put the segment back to its previous layout */
extern int avi_segment_unhide(struct avi_segment *segment);

/* Retrieve a copy of segment's input format */
extern void avi_segment_get_input_format(struct avi_segment *segment,
                                         struct avi_segment_format *format);

/* Retrieve a copy of segment's output format */
extern void avi_segment_get_output_format(struct avi_segment *segment,
                                          struct avi_segment_format *format);


extern void avi_segment_get_layout(struct avi_segment		*segment,
                                   struct avi_segment_layout	*layout);

/* Retrieve a node by its capability. */
extern struct avi_node *avi_segment_get_node(struct avi_segment *s,
                                             const unsigned long cap);

/* Configure the DMA input buffer for an AVI_CAP_DMA_IN segment */
void avi_segment_set_input_buffer(struct avi_segment *segment,
                                  const struct avi_dma_buffer *buff);

/* Configure the DMA output buffer for an AVI_CAP_DMA_OUT segment */
void avi_segment_set_output_buffer(struct avi_segment *segment,
                                   const struct avi_dma_buffer *buff);
/********************************
 * Public segment index function
 ********************************/

/* Iterate over all existing segments, calling "cback" on each.
 *
 * WARNING: The callback is called with the index lock held
 */
extern int avi_segment_foreach(int (*cback)(struct avi_segment *, void *),
                               void *priv);

/* Return the total number of segments currently registered */
extern unsigned avi_segment_count(void);

#endif /* _AVI_SEGMENT_H_ */
