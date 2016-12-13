#ifndef _FC7100_VARIANT_H_
#define _FC7100_VARIANT_H_

#include <linux/init.h>

struct fc7100_variant {
	const char	*name;
	void           (*init_lcd)(void);
	void           (*reserve_lcd_mem)(void);
};

extern const struct fc7100_variant __initdata *fc7100_variant;

#endif /* _FC7100_VARIANT_H_ */

