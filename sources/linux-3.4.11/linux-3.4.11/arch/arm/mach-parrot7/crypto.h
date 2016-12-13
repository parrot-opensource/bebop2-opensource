/**
 * linux/arch/arm/mach-parrot7/crypto.h - P7 crypto IP module platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Karl Leplat <karl.leplat@parrot.com>
 * date:    14-Feb-2014
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_CRYPTO_H
#define _ARCH_PARROT7_CRYPTO_H

#if defined(CONFIG_CRYPTO_DEV_P7) || \
    defined(CONFIG_CRYPTO_DEV_P7_MODULE)

#include <linux/init.h>

extern void p7_init_crypto(void) __init;

#else  /* defined(CONFIG_CRYPTO_DEV_P7) || \
          defined(CONFIG_CRYPTO_DEV_P7_MODULE) */

static inline void p7_init_crypto(void) {}

#endif /* defined(CONFIG_CRYPTO_DEV_P7) || \
          defined(CONFIG_CRYPTO_DEV_P7_MODULE) */

#endif
