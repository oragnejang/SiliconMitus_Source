/*
 * sm5011-irq.c
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>

#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-irq.h>

#define DEV_NAME "sm5011"

#define IRQ_NAME(irq) [irq] = irq##_NAME
static const char *sm5011_irq_names[] = {
	IRQ_NAME(SM5011_IRQ1_MICDET),
	IRQ_NAME(SM5011_IRQ1_VBAT_VALID),
	IRQ_NAME(SM5011_IRQ1_MANUALRST),
	IRQ_NAME(SM5011_IRQ1_LONGKEY_MRSTB),
	IRQ_NAME(SM5011_IRQ1_LONGKEY_CHGON),
	IRQ_NAME(SM5011_IRQ1_LONGKEY_nONKEY),
	IRQ_NAME(SM5011_IRQ1_SHORTKEY),
	
	IRQ_NAME(SM5011_IRQ2_ALARM2_ON),
	IRQ_NAME(SM5011_IRQ2_ALARM1_ON),
	IRQ_NAME(SM5011_IRQ2_WDTMEROUT),
};

enum SM5011_INT_MASK_GROUP {
	SM5011_INT1_MASK_GROUP = 0,
	SM5011_INT2_MASK_GROUP
};

enum SM5011_INT_GROUP {
	SM5011_INT1_GROUP = 0,
	SM5011_INT2_GROUP
};

static const u8 sm5011_mask_reg[] = {
	[SM5011_INT1_MASK_GROUP]     = SM5011_REG_INTMSK1,
	[SM5011_INT2_MASK_GROUP]     = SM5011_REG_INTMSK2,
};

struct sm5011_irq_data {
	int mask;
	int group;
};

#define DECLARE_IRQ(idx, _group, _mask)     [(idx)] = { .group = (_group), .mask = (_mask) }

static const struct sm5011_irq_data sm5011_irq_datas[] = {
	DECLARE_IRQ(SM5011_IRQ1_MICDET,	        SM5011_INT1_GROUP,  1 << 6),
	DECLARE_IRQ(SM5011_IRQ1_VBAT_VALID,	    SM5011_INT1_GROUP,  1 << 5),
	DECLARE_IRQ(SM5011_IRQ1_MANUALRST,      SM5011_INT1_GROUP,  1 << 4),
	DECLARE_IRQ(SM5011_IRQ1_LONGKEY_MRSTB,	SM5011_INT1_GROUP,  1 << 3),
	DECLARE_IRQ(SM5011_IRQ1_LONGKEY_CHGON,  SM5011_INT1_GROUP,  1 << 2),
	DECLARE_IRQ(SM5011_IRQ1_LONGKEY_nONKEY, SM5011_INT1_GROUP,  1 << 1),
	DECLARE_IRQ(SM5011_IRQ1_SHORTKEY,       SM5011_INT1_GROUP,  1 << 0),
	
	DECLARE_IRQ(SM5011_IRQ2_ALARM2_ON,	    SM5011_INT2_GROUP,  1 << 2),
	DECLARE_IRQ(SM5011_IRQ2_ALARM1_ON,      SM5011_INT2_GROUP,  1 << 1),
	DECLARE_IRQ(SM5011_IRQ2_WDTMEROUT,      SM5011_INT2_GROUP,  1 << 0),
};

static void sm5011_irq_lock(struct irq_data *data)
{
	struct sm5011_pmic_dev *sm5011 = irq_get_chip_data(data->irq);

	mutex_lock(&sm5011->irqlock);
}

static void sm5011_irq_sync_unlock(struct irq_data *data)
{
	struct sm5011_pmic_dev *sm5011 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < NUM_IRQ_REGS; i++) {
		struct i2c_client *i2c = sm5011->i2c;

		if (IS_ERR_OR_NULL(i2c))
			continue;

		sm5011->irq_masks_cache[i] = sm5011->irq_masks_cur[i];

        sm5011_reg_write(sm5011, sm5011_mask_reg[i], sm5011->irq_masks_cur[i]);
	}

	mutex_unlock(&sm5011->irqlock);
}

	static const inline struct sm5011_irq_data *
irq_to_sm5011_irq(struct sm5011_pmic_dev *sm5011, int irq)
{
	return &sm5011_irq_datas[irq - sm5011->irq_base];
}

static void sm5011_irq_mask(struct irq_data *data)
{
	struct sm5011_pmic_dev *sm5011 = irq_get_chip_data(data->irq);
	const struct sm5011_irq_data *irq_data = irq_to_sm5011_irq(sm5011, data->irq);

	if (irq_data->group >= NUM_IRQ_REGS)
		return;

	sm5011->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void sm5011_irq_unmask(struct irq_data *data)
{
	struct sm5011_pmic_dev *sm5011 = irq_get_chip_data(data->irq);
	const struct sm5011_irq_data *irq_data = irq_to_sm5011_irq(sm5011, data->irq);

	if (irq_data->group >= NUM_IRQ_REGS)
		return;

	sm5011->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip sm5011_irq_chip = {
	.name			        = DEV_NAME,
	.irq_bus_lock		    = sm5011_irq_lock,
	.irq_bus_sync_unlock	= sm5011_irq_sync_unlock,
	.irq_mask		        = sm5011_irq_mask,
	.irq_unmask		        = sm5011_irq_unmask,
};

const char *sm5011_get_irq_name_by_index(int index)
{
	return sm5011_irq_names[index];
}
EXPORT_SYMBOL(sm5011_get_irq_name_by_index);


static irqreturn_t sm5011_irq_thread(int irq, void *data)
{
	struct sm5011_pmic_dev *sm5011 = data;
	u8 irq_reg[NUM_IRQ_REGS] = {0};
	int i, ret;

	ret = sm5011_bulk_read(sm5011, SM5011_REG_INT1, NUM_IRQ_REGS, &irq_reg[SM5011_INT1_GROUP]);
	if (ret) {
		pr_err("%s:%s fail to read: %d\n", "sm5011", __func__, ret);
		return IRQ_NONE;
	}
	for (i = 0; i < NUM_IRQ_REGS; i++) {
		pr_info("%s : %d = 0x%x\n", __func__, i, irq_reg[i]);
	}

	/* Apply masking */
	for (i = 0; i < NUM_IRQ_REGS; i++) {
		irq_reg[i] &= ~sm5011->irq_masks_cur[i];
	}

	/* Report */
	for (i = 0; i < SM5011_IRQ_NR; i++) {
		if (irq_reg[sm5011_irq_datas[i].group] & sm5011_irq_datas[i].mask) {
			handle_nested_irq(sm5011->irq_base + i);
		}
	}

	return IRQ_HANDLED;
}

int sm5011_irq_init(struct sm5011_pmic_dev *sm5011_pmic)
{
	int ret = 0;
	int i = 0;

	if (!sm5011_pmic->irq) {
		pr_err("No interrupt specified, no interrupts\n");
		sm5011_pmic->irq_base = 0;
		return 0;
	}

	mutex_init(&sm5011_pmic->irqlock);

	/* Mask individual interrupt sources */
	for (i = 0; i < NUM_IRQ_REGS; i++) {
        sm5011_pmic->irq_masks_cur[i] = 0xff;
        sm5011_pmic->irq_masks_cache[i] = 0xff;
		sm5011_reg_write(sm5011_pmic, sm5011_mask_reg[i], sm5011_pmic->irq_masks_cur[i]);
	}

	sm5011_pmic->irq_base = irq_alloc_descs(-1, 0, SM5011_IRQ_NR, 0);
	if (sm5011_pmic->irq_base < 0) {
		pr_err("Failed to allocate IRQs: %d\n",
			 sm5011_pmic->irq_base);
		return -EINVAL;
	}
	
	/* Register with genirq */
	for (i = 0; i < SM5011_IRQ_NR; i++) {
		int cur_irq;
		cur_irq = i + sm5011_pmic->irq_base;
		irq_set_chip_data(cur_irq, sm5011_pmic);
		irq_set_chip_and_handler(cur_irq, &sm5011_irq_chip, handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}
	irq_set_status_flags(sm5011_pmic->irq, IRQ_NOAUTOEN); 
	ret = request_threaded_irq(sm5011_pmic->irq, NULL, sm5011_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"sm5011-irq", sm5011_pmic);
	if (ret) {
		pr_err("Fail to request IRQ %d: %d\n", sm5011_pmic->irq, ret);
		return ret;
	}

	enable_irq(sm5011_pmic->irq);

	return 0;
}

void sm5011_irq_exit(struct sm5011_pmic_dev *sm5011_pmic)
{
	if (sm5011_pmic->irq)
		free_irq(sm5011_pmic->irq, sm5011_pmic);
}
