#ifndef _AVI_DEBUG_H_
#define _AVI_DEBUG_H_

#include "avi_segment.h"
#include "avi_pixfmt.h"

extern const char *avi_debug_colorspace_to_str(enum avi_colorspace cspace);
extern const char *avi_debug_pixfmt_to_str(struct avi_dma_pixfmt fmt);
extern const char *avi_debug_activation_to_str(enum avi_segment_activation active);
extern const char *avi_debug_force_clear_to_str(unsigned clear);

/* For printing capabilities: returns the name of a capability in cap, update
 * cap to remove the capability returned. Returns NULL when no capability
 * remains.
 *
 * Basically when you want to display a set of capabilities you iterate over
 * this function until it returns NULL */
extern const char *avi_debug_caps_to_str(unsigned long *caps);

extern char *avi_debug_format_caps(unsigned long caps, char *buf, size_t buf_size);

#endif /* _AVI_DEBUG_H_ */

