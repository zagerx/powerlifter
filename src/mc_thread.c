/**
 * @file motor_thread.c
 * @brief BLDC motor control thread implementation
 *
 * Handles:
 * - Motor control thread creation
 * - Hardware initialization (GPIOs, watchdog)
 * - Main motor control loop
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include "fault_monitoring_module.h"
#include "statemachine.h"
#include "zephyr/device.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <lib/bldcmotor/motor.h>
#include <fault_monitoring_module.h>
/* Module logging setup */
LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);

/* Thread stack definition */
K_THREAD_STACK_DEFINE(motor_thread_stack, 2048);

/**
 * @struct motor_thread_data
 * @brief Motor thread control structure
 */
struct motor_thread_data {
	const struct device *motor_dev; ///< Motor device pointer
	struct k_thread thread;         ///< Thread control block
};
/**
 * @brief Motor control thread entry function
 * @param p1 Unused parameter
 * @param p2 Unused parameter
 * @param p3 Unused parameter
 *
 * Initializes hardware and runs main control loop:
 * 1. Configures all GPIO devices
 * 2. Starts motor control tasks
 * 3. Maintains watchdog timer
 */
extern void motor_set_vol(const struct device *motor, float *bus_vol);

static void motor_thread_entry(void *p1, void *p2, void *p3)
{
	const struct device *motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
	const struct motor_config *cfg = motor0->config;
	float bus_volcurur[2];
	fmm_init(cfg->fault, 60.0f, 48.0f, 5, 5, NULL);
	static int16_t fault_fsm = MOTOR_STATE_IDLE;
	/* Main control loop */
	while (1) {
		motor_getbus_vol_curr(motor0, &bus_volcurur[0], &bus_volcurur[1]);
		motor_set_vol(motor0, bus_volcurur);

		fmm_monitoring(cfg->fault, bus_volcurur[0]);
		switch (fault_fsm) {
		case MOTOR_STATE_FAULT:
			if (fmm_readstatus(cfg->fault) == FMM_NORMAL) {
				fault_fsm = MOTOR_STATE_IDLE;
				motor_set_state(motor0, MOTOR_STATE_IDLE);
			}
			break;
		case MOTOR_STATE_IDLE:
			if (fmm_readstatus(cfg->fault) == FMM_FAULT) {
				fault_fsm = MOTOR_STATE_FAULT;
				motor_set_state(motor0, MOTOR_STATE_FAULT);
			}
		}
		DISPATCH_FSM(cfg->fsm);
		k_msleep(1);
	}
}

/**
 * @brief Create motor control thread
 * @param dev Unused device pointer
 *
 * Creates high-priority cooperative thread for motor control.
 */
void creat_motor_thread(const struct device *dev)
{
	static struct motor_thread_data thread_data __aligned(4);

	k_thread_create(&thread_data.thread, motor_thread_stack,
			K_THREAD_STACK_SIZEOF(motor_thread_stack), motor_thread_entry, NULL, NULL,
			NULL,
			K_PRIO_COOP(4), // High priority cooperative thread
			0, K_NO_WAIT);
}
