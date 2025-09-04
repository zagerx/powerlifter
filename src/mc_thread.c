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
#include <lib/motor/motor.h>
#include <fault_monitoring_module.h>
#include <zephyr/sys/reboot.h>

/* Module logging setup */
#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_thread);

/* Thread stack definition */
K_THREAD_STACK_DEFINE(motor_thread_stack, 2048);
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC        DT_NODELABEL(encoder_vcc)
#define W_DOG              DT_NODELABEL(wdog)
/**
 * @struct motor_thread_data
 * @brief Motor thread control structure
 */

static struct k_thread mc_thread; ///< Thread control block

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
extern void motor_set_falutcode(const struct device *motor, enum motor_fault_code code);
extern enum motor_fault_code motor_get_falutcode(const struct device *motor);
static void motor_thread_entry(void *p1, void *p2, void *p3)
{
	const struct device *motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
	const struct motor_data *m_data = motor0->data;

	int ret;

	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
	ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure brake pin (err %d)", ret);
	}
	gpio_pin_set_dt(&mot12_brk, 0);
	/* Initialize watchdog pin */
	const struct gpio_dt_spec w_dog = GPIO_DT_SPEC_GET(W_DOG, gpios);
	ret = gpio_pin_configure_dt(&w_dog, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure watchdog pin (err %d)", ret);
	}

	/* Initialize encoder power */
	const struct gpio_dt_spec encoder_vcc = GPIO_DT_SPEC_GET(ENCODER_VCC, gpios);
	ret = gpio_pin_configure_dt(&encoder_vcc, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure encoder power (err %d)", ret);
	}

	float bus_volcurur[2];
	fmm_t *bus_vol_fmm = m_data->fault[0];
	fmm_t *buf_curr_fmm = m_data->fault[1];
	fmm_init(bus_vol_fmm, 60.0f, 48.0f, 5, 5, NULL);
	fmm_init(buf_curr_fmm, 5.0f, 0.0f, 5, 5, NULL);

	static int16_t fault_fsm = 0;
	while (1) {
		motor_get_bus_voltage_current(motor0, &bus_volcurur[0], &bus_volcurur[1]);
		motor_set_vol(motor0, bus_volcurur);

		fmm_monitoring(bus_vol_fmm, bus_volcurur[0]);
		fmm_monitoring(buf_curr_fmm, bus_volcurur[1]);

		switch (fault_fsm) {
		case 1:
			if (fmm_readstatus(bus_vol_fmm) == FMM_NORMAL &&
			    fmm_readstatus(buf_curr_fmm) == FMM_NORMAL) { // 判断是否恢复
				fault_fsm = 0;
				motor_set_falutcode(motor0, MOTOR_FAULTCODE_NOERR);
				motor_set_state(motor0, MOTOR_STATE_IDLE);
			}
			break;
		case 0: // 判断是否有故障
			if (fmm_readstatus(bus_vol_fmm) == FMM_FAULT) {
				fault_fsm = 1;
				motor_set_falutcode(motor0, MOTOR_FAULTCODE_UNDERVOL);
				motor_set_state(motor0, MOTOR_STATE_FAULT);
			} else if (fmm_readstatus(buf_curr_fmm) == FMM_FAULT) {
				fault_fsm = 1;
				motor_set_falutcode(motor0, MOTOR_FAULTCODE_OVERCURRMENT);
				motor_set_state(motor0, MOTOR_STATE_FAULT);
			} else {
			}
			break;
		default:
			break;
		}

		DISPATCH_FSM(m_data->mode_state_mec);
		gpio_pin_toggle_dt(&w_dog);

		k_msleep(1);
	}
}

/**
 * @brief Create motor control thread
 * @param dev Unused device pointer
 *
 * Creates high-priority cooperative thread for motor control.
 */
int creat_motor_thread(void)
{

	k_thread_create(&mc_thread, motor_thread_stack, K_THREAD_STACK_SIZEOF(motor_thread_stack),
			motor_thread_entry, NULL, NULL, NULL,
			K_PRIO_COOP(4), // High priority cooperative thread
			0, K_NO_WAIT);
	return 0;
}
SYS_INIT(creat_motor_thread, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);