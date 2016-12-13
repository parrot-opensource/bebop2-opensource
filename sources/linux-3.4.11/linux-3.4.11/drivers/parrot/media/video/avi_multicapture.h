/**
 * Avi multicapture configuration
 */
#ifndef _AVI_MULTICAPTURE_H__
#define _AVI_MULTICAPTURE_H__

struct avimulti_platform_data {
	/* 
	 * The number of cameras that will be set in full-framerate (2 frames instead of 1).
	 *
	 * For example, if you have 3 cameras @30 and you set this parameter to 1, you will get
	 * the following stream running at @15 :
	 * 
	 * CAM_0 (1st frame)
	 * CAM_0 (2nd frame)
	 * CAM_1
	 * CAM_2
	 *
	 * Leave to 0 if the system can handle all the cameras at full framerate.
	 */
	unsigned int nb_full_framerate;

	/*
	 * All the cameras that will be merged into the multicapture driver
	 * The order is important if you set nb_full_framerate to something other than 0
	 */
	struct avicam_platform_data *cam_interfaces;

	/*
	 * The number of cameras in cam_interfaces
	 */
	unsigned int nb_cameras;

	/**
	 * The width/height of the cameras.
	 * This implies that all the cameras must have the same dimensions.
	 * XXX: probe the cameras' V4L2 drivers and get rid of those values
	 */
	unsigned int width;
	unsigned int height;

	/**
	 * Mosaics will contain one additional "struct avimulti_metadata" for each frame.
	 * These structs will be stored at the end of the buffer, adding as many "lines"
	 * as necessary.
	 */
	unsigned int enable_metadata;

	/**
	 * How long is judged enough to trigger a timeout on one of the cameras
	 * This is used to disable specific cameras on the fly if they stop responding
	 * to not freeze the mosaic.
	 * 
	 * "HZ" = 1s
	 */
	unsigned int timeout;
	/**
	 * Videobuf2 cpu caching flags
	 */
	enum vb2_cache_flags vb2_cache_flags;
};

/**
 * Contains metadata about one frame in the mosaic
 */
struct avimulti_metadata {
	int enabled;
	s64 timestamp;
} __attribute__((packed));

#endif
