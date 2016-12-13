#include "board-fc7100-workbench-variant.h"

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/string.h>

#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>

#include "common.h"
#include "fc7100-module.h"

extern void __init yoshi_lcd_mem(void);
extern void __init yoshi_init_lcd(void);

static const struct fc7100_variant fc7100_variants[] =
{
	{
		.name		 = "yoshi",
		.reserve_lcd_mem = &yoshi_lcd_mem,
		.init_lcd	 = &yoshi_init_lcd,
	},
};

const struct fc7100_variant __initdata *fc7100_variant = NULL;

static int __init fc7100_wb_variant_setup(char *value)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(fc7100_variants); i++)
		if (!strcmp(fc7100_variants[i].name, value)) {
			fc7100_variant = &fc7100_variants[i];
			printk(KERN_INFO
			       "FC7100 Workbench: enabling variant %s\n",
				fc7100_variant->name);
			return 0;
		}

	printk(KERN_ERR "FC7100 Workbench: unknown variant %s\n", value);

	BUG();

	return 0;
}
early_param("fc7100_wb_variant", fc7100_wb_variant_setup);
