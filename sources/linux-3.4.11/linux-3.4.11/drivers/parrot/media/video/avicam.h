#ifndef _AVICAM_H_
#define _AVICAM_H_

#include <linux/i2c.h>
#include <media/videobuf2-core.h>

#include "avi_v4l2.h"
#include "avi_capture_timings.h"

enum avicam_bus_width {
	AVICAM_BUS_WIDTH_8,
	AVICAM_BUS_WIDTH_16,
};

/*	this name should correspond to the module file name,
	not to the module.driver.name as declared in the driver code */
#define AVICAM_DUMMY_NAME "avicam_dummy_dev"

struct avicam_subdevs {
	struct i2c_board_info	*board_info;
	int			 i2c_adapter_id;
	struct avicam_subdevs	 *subdevs;/*subdevices connected to a video input of the current subdev */
};

struct avicam_platform_data {
	/* We need at least one camera */
	unsigned long                    cam_cap;

	/* True if we want to create a video device for stat capture (only
	 * available for raw format capture)*/
	int                              enable_stats;

	/* Custom measure-to-timings conversion function, if necessary */
	avi_capture_get_timings          measure_to_timings;

	union avi_cam_interface          interface;
	unsigned int			 bus_config;
	char				 bus_width;
	struct avicam_subdevs		*subdevs;
	struct avicam_dummy_info	*dummy_driver_info;
	enum vb2_cache_flags             vb2_cache_flags;
};

#endif /* _AVICAM_H_ */
