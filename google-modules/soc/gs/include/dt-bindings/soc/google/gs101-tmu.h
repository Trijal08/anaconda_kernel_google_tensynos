/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Google, LLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for GS101
 */

#ifndef _DT_BINDINGS_GS_101_TMU_H
#define _DT_BINDINGS_GS_101_TMU_H

/* NUMBER FOR TMU TYPE*/
#define TMU_TYPE_CPU	0
#define TMU_TYPE_GPU	1
#define TMU_TYPE_ISP	2
#define TMU_TYPE_TPU	3
/* AUR exists on gs201/zuma only; see enum tmu_type_t in gs_tmu_v3.c */
#define TMU_TYPE_AUR	4
#define TMU_TYPE_END	5

#endif /* _DT_BINDINGS_GS_101_TMU_H */
