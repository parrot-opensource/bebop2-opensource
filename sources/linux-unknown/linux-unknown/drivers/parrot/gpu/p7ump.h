/**
 * linux/driver/parrot/gpuo/p7ump.h - Parrot 7 Mali UMP API
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    2013-12-04
 *
 * This file is released under the GPL
 */


#ifndef _P7UMP_H
#define _P7UMP_H

#define P7UMP_IO_MAGIC 'k'

struct p7ump_token {
	int	bus;
	int	size;
	int	id;
};

#define P7UMP_GET_ID _IOWR(P7UMP_IO_MAGIC, 1, struct p7ump_token)
#define P7UMP_REL_ID _IOWR(P7UMP_IO_MAGIC, 2, int)


#if defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE)
extern int p7ump_add_map(u32 addr, size_t size);
extern int p7ump_remove_map(u32 addr, size_t size);
#else /* defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE) */
static inline int p7ump_add_map(u32 addr, size_t size)
{
	return -ENOSYS;
}

static inline int p7ump_remove_map(u32 addr, size_t size)
{
	return -ENOSYS;
}
#endif /* defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE) */


#endif
