/*
 * sm5010-wdt.c
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/regmap.h>
#include <linux/mfd/sm5010-core.h>
#include <linux/mfd/sm5010-irq.h>
#include <linux/mfd/sm5010.h>

/* ENWATCHDOG */
#define	SM5010_WDT_EN_SHIFT		0
#define	SM5010_WDT_EN_MASK		(1 << SM5010_WDT_EN_SHIFT)
/* WATCHDOG_TMR */
#define	SM5010_WDT_TMR_SHIFT	1
#define	SM5010_WDT_TMR_MASK		(3 << SM5010_WDT_TMR_SHIFT)
/* WDTMER_RST */
#define	SM5010_WDT_RST_SHIFT	3
#define	SM5010_WDT_RST_MASK		(1 << SM5010_WDT_RST_SHIFT)

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct sm5010_wdt {
	struct device			*dev;
	struct sm5010_pmic_dev	*iodev;
	struct mutex			lock;
	int						irq_base;

	struct watchdog_device	wdd;
	struct clk		*clk;
	unsigned long		rate;
	struct notifier_block	restart_nb;
};

static struct {
	unsigned int time;  /* Seconds */
	int val;            /* Watchdog time value */
} sm5010_wdt_cfgs[] = {
	{  30, 0 },
	{  60, 1 },
	{  90, 2 },
	{  120, 3 },
};

static inline
struct sm5010_wdt *to_sm5010_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct sm5010_wdt, wdd);
}

static int sm5010_wdt_start(struct watchdog_device *wdd)
{
	struct sm5010_wdt *wdt = to_sm5010_wdt(wdd);	
	int ret;

	pr_info("%s\n", __func__);

	mutex_lock(&wdt->lock);

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (1 << SM5010_WDT_EN_SHIFT), SM5010_WDT_EN_MASK);
	if (ret < 0) {
		dev_err(wdt->dev, "%s: fail to update reg(%d)\n",
			__func__, ret);
		goto out;
	}

out:	
	mutex_unlock(&wdt->lock);

	return 0;
}

static int sm5010_wdt_stop(struct watchdog_device *wdd)
{
	struct sm5010_wdt *wdt = to_sm5010_wdt(wdd);	
	int ret;

	pr_info("%s\n", __func__);
	
	mutex_lock(&wdt->lock);

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (0 << SM5010_WDT_EN_SHIFT), SM5010_WDT_EN_MASK);
	if (ret < 0) {
		dev_err(wdt->dev, "%s: fail to update reg(%d)\n",
			__func__, ret);
		goto out;
	}

out:	
	mutex_unlock(&wdt->lock);

	return 0;
}

static int sm5010_wdt_ping(struct watchdog_device *wdd)
{
	struct sm5010_wdt *wdt = to_sm5010_wdt(wdd);	
	int ret;

	pr_info("%s\n", __func__);

	mutex_lock(&wdt->lock);
	
	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (1 << SM5010_WDT_RST_SHIFT), SM5010_WDT_RST_MASK);		
	if (ret < 0) {
		dev_err(wdt->dev, "%s: fail to update reg(%d)\n",
			__func__, ret);
		goto out;
	}

out:	
	mutex_unlock(&wdt->lock);

	return 0;

}

static int sm5010_wdt_set_timeout(struct watchdog_device *wdd,
				unsigned int timeout)
{
	struct sm5010_wdt *wdt = to_sm5010_wdt(wdd);	
	int ret, i;

	pr_info("%s\n", __func__);

	mutex_lock(&wdt->lock);

	for (i = 0; i < ARRAY_SIZE(sm5010_wdt_cfgs); i++)
		if (sm5010_wdt_cfgs[i].time == timeout)
			break;
	if (i == ARRAY_SIZE(sm5010_wdt_cfgs))
		goto out;

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, 
			(sm5010_wdt_cfgs[i].val << SM5010_WDT_TMR_SHIFT), SM5010_WDT_TMR_MASK);
	if (ret < 0) {
		dev_err(wdt->dev, "%s: fail to update reg(%d)\n",
			__func__, ret);
		goto out;
	}
	wdd->timeout = timeout;

out:	
	mutex_unlock(&wdt->lock);
	
	return sm5010_wdt_start(wdd);
}

static const struct watchdog_ops sm5010_wdt_ops = {
	.start			= sm5010_wdt_start,
	.stop			= sm5010_wdt_stop,
	.ping			= sm5010_wdt_ping,
	.set_timeout	= sm5010_wdt_set_timeout,
	.owner			= THIS_MODULE,
};

static const struct watchdog_info sm5010_wdt_info = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT,
	.identity	= "sm5010-wdt",
};

static int sm5010_wdt_restart(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct sm5010_wdt *wdt = container_of(nb, struct sm5010_wdt, restart_nb);
	struct sm5010_platform_data *pdata = dev_get_platdata(wdt->iodev->dev);
	int ret, i;
	
	pr_info("%s\n", __func__);
	
	mutex_lock(&wdt->lock);

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (0 << SM5010_WDT_EN_SHIFT), SM5010_WDT_EN_MASK);
	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (1 << SM5010_WDT_RST_SHIFT), SM5010_WDT_RST_MASK);

	for (i = 0; i < ARRAY_SIZE(sm5010_wdt_cfgs); i++)
		if (sm5010_wdt_cfgs[i].val == pdata->wdt_timer_val)
			break;
	if (i == ARRAY_SIZE(sm5010_wdt_cfgs))
		goto out;

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, 
			(sm5010_wdt_cfgs[i].val << SM5010_WDT_TMR_SHIFT), SM5010_WDT_TMR_MASK);

	ret = sm5010_reg_update(wdt->iodev, SM5010_REG_WDTCNTL, (pdata->wdt_en << SM5010_WDT_EN_SHIFT), SM5010_WDT_EN_MASK);

out:	
	mutex_unlock(&wdt->lock);

	return NOTIFY_DONE;
}

static irqreturn_t sm5010_wdt_timerout_irq_handler(int irq, void *data)
{
	struct sm5010_wdt *wdt = data;

	if (!wdt->dev)
		return IRQ_HANDLED;

	dev_info(wdt->dev, "%s:irq(%d)\n", __func__, irq);

	return IRQ_HANDLED;
}

const struct sm5010_irq_handler sm5010_wdt_irq_handlers[] = {
	{
		.name = "SM5010_WDTMEROUT",
		.irq_index = SM5010_IRQ2_WDTMEROUT,		
		.handler = sm5010_wdt_timerout_irq_handler,
	},
};

static int register_irq(struct platform_device *pdev,
		struct sm5010_wdt *wdt)
{
	int irq;
	int i, j;
	int ret;
	const struct sm5010_irq_handler *irq_handler = sm5010_wdt_irq_handlers;
	const char *irq_name;
    
	for (i = 0; i < ARRAY_SIZE(sm5010_wdt_irq_handlers); i++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		
		if(irq < 0) {
			pr_err("%s,(wdt) ERROR irq = [%d] \n", __func__, irq);
			goto err_irq;
		}
		irq = wdt->irq_base + irq;
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, 
			irq_handler[i].handler, 0, irq_name, wdt);
		if (ret < 0) {
			pr_err("%s :(wdt)  Failed to request IRQ (%s): #%d: %d\n",
					__func__, irq_name, irq, ret);
			goto err_irq;
		}

		pr_info("%s :(wdt) Register IRQ%d(%s) successfully\n",
				__func__, irq, irq_name);
	}

	return 0;
err_irq:
	for (j = 0; j < i; j++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[j].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, wdt);
	}

	return ret;
}

static void unregister_irq(struct platform_device *pdev,
		struct sm5010_wdt *wdt)
{
	int irq;
	int i;
	const char *irq_name;
	const struct sm5010_irq_handler *irq_handler = sm5010_wdt_irq_handlers;

	for (i = 0; i < ARRAY_SIZE(sm5010_wdt_irq_handlers); i++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, wdt);
	}
}

static int sm5010_wdt_probe(struct platform_device *pdev)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5010_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct sm5010_wdt *wdt;
	int ret = 0;
	int i;

	pr_info("%s: Start\n", __func__);

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	
	mutex_init(&wdt->lock);
	wdt->dev = &pdev->dev;
	wdt->iodev = iodev;
	wdt->wdd.dev = &pdev->dev;
	wdt->wdd.info = &sm5010_wdt_info;	
	wdt->wdd.ops = &sm5010_wdt_ops;
	wdt->wdd.min_timeout = sm5010_wdt_cfgs[0].time;
	wdt->wdd.max_timeout = sm5010_wdt_cfgs[3].time;

	for (i = 0; i < ARRAY_SIZE(sm5010_wdt_cfgs); i++)
		if (sm5010_wdt_cfgs[i].val == pdata->wdt_timer_val)
			break;
		
	if (i == ARRAY_SIZE(sm5010_wdt_cfgs))
		return -EINVAL;
	
	wdt->wdd.timeout = sm5010_wdt_cfgs[i].time;	
	
	ret = watchdog_init_timeout(&wdt->wdd, wdt->wdd.timeout, &pdev->dev);
	if (ret)
		goto err_wdt_init_timeout; 

	wdt->irq_base = iodev->irq_base;

	ret = register_irq(pdev, wdt);
	if (ret < 0)
        goto err_wdt_irq; 

	watchdog_set_nowayout(&wdt->wdd, nowayout);

	pr_info("%s: wdt->wdd.timeout = %d, wdt->wdd.status = 0x%lx, nowayout = %d\n", 
		__func__, wdt->wdd.timeout,wdt->wdd.status, nowayout);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register watchdog\n");
		goto err_clk_unprepare;
	}

	/*
	 * WDT restart notifier has priority 0 (use as a last resort)
	 */
	wdt->restart_nb.notifier_call = sm5010_wdt_restart;
	ret = register_restart_handler(&wdt->restart_nb);
	if (ret)
		dev_err(&pdev->dev, "failed to setup restart handler\n");

	platform_set_drvdata(pdev, wdt);

	pr_info("%s: Done\n", __func__);
	
	return 0;
	
err_clk_unprepare:	
	mutex_destroy(&wdt->lock);
err_wdt_irq:	
err_wdt_init_timeout:

	return ret;
}

static int sm5010_wdt_remove(struct platform_device *pdev)
{
	struct sm5010_wdt *wdt = platform_get_drvdata(pdev);

	unregister_irq(pdev, wdt);

	unregister_restart_handler(&wdt->restart_nb);
	watchdog_unregister_device(&wdt->wdd);

	return 0;
}

static void sm5010_wdt_shutdown(struct platform_device *pdev)
{
	struct sm5010_wdt *wdt = platform_get_drvdata(pdev);

	sm5010_wdt_stop(&wdt->wdd);
}

static const struct of_device_id sm5010_wdt_of_match[] = {
	{ .compatible = "sm,sm5010-wdt", },
	{ },
};
MODULE_DEVICE_TABLE(platform, sm5010_wdt_of_match);

static struct platform_driver sm5010_watchdog_driver = {
	.driver	= {
		.name		= "sm5010-wdt",
		.of_match_table	= sm5010_wdt_of_match,
		.owner		= THIS_MODULE,
	},
	.probe	= sm5010_wdt_probe,
	.remove	= sm5010_wdt_remove,
	.shutdown	= sm5010_wdt_shutdown,	
};

module_platform_driver(sm5010_watchdog_driver);

MODULE_DESCRIPTION("SM5010 Watchdong Driver");
MODULE_LICENSE("GPL");
