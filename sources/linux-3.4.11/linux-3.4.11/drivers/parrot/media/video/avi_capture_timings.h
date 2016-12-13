#ifndef _AVI_CAPTURE_TIMINGS_H_
#define _AVI_CAPTURE_TIMINGS_H_

#include "video/avi.h"

struct avi_capture_timings {
	union avi_cam_h_timing ht;
	union avi_cam_v_timing vt;
};

typedef void (*avi_capture_get_timings)(struct avi_capture_timings  *t,
                                        struct avi_cam_measure_regs *m);

#endif /* _AVI_CAPTURE_TIMINGS_H_ */
