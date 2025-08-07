/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include <zephyr/kernel.h>

int main(void)
{
	while (1) {
		k_msleep(1000);
	}
	return 0;
}
