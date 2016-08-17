#ifndef _AVI_V4L2_DEV_H_
#define _AVI_V4L2_DEV_H_

#include <asm/ioctl.h>

#include "video/avi_cap.h"

/*
 * Userland interface
 */

struct avi_dev_segment {
	unsigned long	caps;
	char            id[16];
};

#define AVI_BUILD_DEV   _IOW('A', 0x0, struct avi_dev_segment *)

#endif /* _AVI_V4L2_DEV_H_ */
