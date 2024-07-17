/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "uarte_split_trans.h"

LOG_MODULE_REGISTER(app);

int main(void)
{

	LOG_INF("## uarte bug test sample ##");
	dk_leds_init();
	kbd_split_uarte_init();
	while(1) {
		k_msleep(1000);
	}
	return 0;
}
