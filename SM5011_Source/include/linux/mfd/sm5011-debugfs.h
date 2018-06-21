/*
 * sm5011-debugfs.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5011_DEBUGFS_H
#define __LINUX_MFD_SM5011_DEBUGFS_H

#include <linux/device.h>
#include <linux/mutex.h>

/*
 * struct sm5011_pmic_dev - sm5011's master device for sub-drivers
 * @ ops_lock : mutex for debugfs driver
 * @ dev : device of master device
 */
struct sm5011_debugfs_device {
	struct mutex ops_lock;
	struct device dev;
};

/*
 * struct sm5011_debugfs_reg32 - Array of debugfs file
 * @ name : debug filesystem name
 * @ reg : debug filesystem register
 */
struct sm5011_debugfs_reg32 {
	char *name;
	u32 reg;
};

#define sm5011_dump_register(nm)				\
{										\
	.name	= __stringify(nm),			\
	.reg	= SM5011_REG_##nm,			\
}

#define to_sm5011_debugfs_device(obj) container_of(obj, struct sm5011_debugfs_device, dev)

/* 
  * [ External Function ]
  */
extern int sm5011_debugfs_create_attrs(struct device *dev);

#endif /* __LINUX_MFD_SM5011_DEBUGFS_H */
