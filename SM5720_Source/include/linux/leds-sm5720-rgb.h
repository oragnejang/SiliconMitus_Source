/*
 *  leds-sm5720-rgb.h
 *  Samsung SM5720 RGB-LED Driver header file
 *
 *  Copyright (C) 2016 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LEDS_SM5720_RGB_H
#define __LEDS_SM5720_RGB_H __FILE__

enum sm5720_led_color {
    BLUE     = 0x0,
	RED,
	GREEN,
	WHITE,
    LED_MAX,
};

enum sm5720_led_mode {
    LED_DISABLE = 0x0,
    LED_ALWAYS_ON,
    LED_BLINK,
};

enum sm5720_led_pattern {
	PATTERN_OFF,
	CHARGING,
	CHARGING_ERR,
	MISSED_NOTI,
	LOW_BATTERY,
	FULLY_CHARGED,
	POWERING,
};

struct sm5720_rgb {
	struct device *dev;
    struct device *led_dev;
	struct led_classdev led[4];
	struct i2c_client *i2c;

    unsigned int delay_on_times_ms;
	unsigned int delay_off_times_ms;
};

struct sm5720_rgb_platform_data
{
	char *name[4];
};

#endif /* __LEDS_SM5720_RGB_H */

