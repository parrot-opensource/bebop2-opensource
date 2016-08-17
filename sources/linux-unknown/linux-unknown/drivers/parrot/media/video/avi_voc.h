#ifndef _AVI_VOC_H
#define _AVI_VOC_H

#include <linux/videodev2.h>
#include <video/avi_cap.h>

#define V4L2_CID_VOC_BASE       (V4L2_CID_USER_BASE | 0x1000)
#define V4L2_CID_VOC_NO_SCALER  (V4L2_CID_VOC_BASE + 0)
#define V4L2_CID_VOC_ZORDER     (V4L2_CID_VOC_BASE + 1)
#define V4L2_CID_VOC_HIDE       (V4L2_CID_VOC_BASE + 2)

/**
 * struct avi_voc_plat_data - AVI voc specific data
 */
struct avi_voc_plat_data {
	char            *display;
};

#endif /* _AVI_VOC_H */
