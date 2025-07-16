#include <stdint.h>
#include <zephyr/kernel.h>
#include "statemachine.h"
#include "zephyr/device.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <lib/bldcmotor/motor.h>
/* Module logging setup */
LOG_MODULE_REGISTER(super_thread, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(super_thread_stack, 2048);

static fsm_cb_t elevator_handle = {
	.chState = 0,
};
uint8_t conctrl_cmd = 0;
#define RISING_DIS 3000.0f
enum {
	ELEVATOR_INIT = USER_STATUS,
	ELEVATOR_FINDZERO,
	ELEVATOR_ZERO,
	ELEVATOR_ISZERO,
	ELEVATOR_ISEND,
	ELEVATOR_END,
	ELEVATOR_STOP,
	ELEVATOR_MOTOR_FAULT,
};
/* Device tree node aliases */
#define LED0_NODE	   DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC	   DT_NODELABEL(encoder_vcc)
#define W_DOG		   DT_NODELABEL(wdog)
#define P_SWITCH	   DT_NODELABEL(proximity_switch)
#define STOP_BUTTON	   DT_NODELABEL(stopbutton)

/* GPIO device specification */
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static void super_elevator_task(void *obj)
{

	fsm_cb_t *elevator_fsm = &elevator_handle;
	const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
	/* Run state machine */
	int switch_state;
	int8_t stop_state;
	stop_state = gpio_pin_get_dt(&stop_bt);
	if (stop_state == 1 && elevator_fsm->chState != ELEVATOR_STOP) {
		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
		gpio_pin_set_dt(&mot12_brk, 0);
		elevator_fsm->chState = ELEVATOR_STOP;
	}

	if (elevator_fsm->chState == ELEVATOR_INIT || elevator_fsm->chState == ELEVATOR_FINDZERO ||
	    elevator_fsm->chState == ELEVATOR_ISEND || elevator_fsm->chState == ELEVATOR_ISZERO ||
	    elevator_fsm->chState == ELEVATOR_STOP) {
		conctrl_cmd = 0;
	}
	if (motor_get_state(motor) == MOTOR_STATE_FAULT) {
		elevator_fsm->chState = ELEVATOR_MOTOR_FAULT;
	}
	switch (elevator_fsm->chState) {
	case ENTER:
	case ELEVATOR_INIT: {
		switch_state = gpio_pin_get_dt(&prx_switch);
		if (switch_state < 0) {
			LOG_WRN("Failed to read proximity switch");
		} else { // 电机正转 找零点
			if (switch_state == 0) {
				if (motor_get_mode(motor) != MOTOR_MODE_SPEED) {
					motor_set_mode(motor, MOTOR_MODE_SPEED);
					LOG_INF("set speed mode");
				} else {
					if (motor_get_state(motor) != MOTOR_STATE_READY) {
						motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
						LOG_INF("motor enable");
						break;
					}
					motor_set_state(motor, MOTOR_CMD_SET_START);
					motor_set_target(motor, 2.5f);
					elevator_fsm->chState = ELEVATOR_FINDZERO;
				}
			} else {
				if (motor_get_mode(motor) != MOTOR_MODE_POSI) {
					motor_set_mode(motor, MOTOR_MODE_POSI);
				}
				elevator_fsm->chState = ELEVATOR_ZERO;
			}
			LOG_DBG("Proximity switch state: %d", switch_state);
		}
	} break;

	case ELEVATOR_FINDZERO: {
		switch_state = gpio_pin_get_dt(&prx_switch);
		if (switch_state == 1) // 找到零点
		{
			motor_set_target(motor, 0.0f);
			motor_set_mode(motor, MOTOR_MODE_POSI);
			elevator_fsm->chState = ELEVATOR_ZERO;
		}
	} break;
	case ELEVATOR_ZERO: // 零点处
	{
		if (motor_get_mode(motor) != MOTOR_MODE_POSI) {
			break;
		}
		gpio_pin_set_dt(&mot12_brk, 0);
		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
		// 添加对顶升命令的响应
		if (conctrl_cmd != 1) {
			break;
		}
		gpio_pin_set_dt(&mot12_brk, 1);
		if (motor_get_state(motor) != MOTOR_STATE_READY) {
			float posi = -RISING_DIS;
			motor_set_target(motor, posi);
			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
			break;
		}
		motor_set_state(motor, MOTOR_CMD_SET_START);
		conctrl_cmd = 0;
		elevator_fsm->chState = ELEVATOR_ISEND;
	} break;

	case ELEVATOR_ISEND: // 等待5s使其达到远端
	{
		{
			static uint16_t conut = 0;
			conut++;
			if (fabsf(motor_get_curposi(motor)) < RISING_DIS - 0.001f) // 已经到达位置
			{
				break;
			}

			if (conut > 4500) {
				conut = 0;
				gpio_pin_set_dt(&mot12_brk, 0);
				motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
				elevator_fsm->chState = ELEVATOR_END;
			}
		}
	} break;

	case ELEVATOR_END: {
		// 1、等待回零点指令
		if (conctrl_cmd != 2) {
			break;
		}
		gpio_pin_set_dt(&mot12_brk, 1);
		if (motor_get_state(motor) != MOTOR_STATE_READY) {
			float posi = RISING_DIS + 50;
			motor_set_target(motor, posi);
			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
			break;
		}
		motor_set_state(motor, MOTOR_CMD_SET_START);
		conctrl_cmd = 0;
		elevator_fsm->chState = ELEVATOR_ISZERO;
	} break;

	case ELEVATOR_ISZERO: // 是否回到零点
		switch_state = gpio_pin_get_dt(&prx_switch);
		if (switch_state != 1) {
			break;
		}
		elevator_fsm->chState = ELEVATOR_ZERO;
		break;
	case ELEVATOR_STOP:
		if (stop_state == 0) {
			gpio_pin_set_dt(&mot12_brk, 1);
			elevator_fsm->chState = ELEVATOR_INIT;
		}
		break;
	case ELEVATOR_MOTOR_FAULT:
		gpio_pin_set_dt(&mot12_brk, 0);
		if (motor_get_state(motor) == MOTOR_STATE_IDLE) {
			gpio_pin_set_dt(&mot12_brk, 1);
			elevator_fsm->chState = ELEVATOR_INIT;
		}
		break;
	case EXIT:
		break;
	}
}

/**
   uint8 INIT = 0
   uint8 NOT_READY = 1
   uint8 UNLOCK = 2
   uint8 LOCKING = 3
   uint8 LOCK = 4
   uint8 UNLOCKING = 5
   uint8 INTERMEDIATE = 6
   uint8 EXCEPTION = 255
*/
int8_t super_elevator_state(void)
{
	int16_t state = 0;
	if (elevator_handle.chState == ELEVATOR_INIT) {
		state = 0;
	} else if (elevator_handle.chState == ELEVATOR_FINDZERO) {
		state = 1;
	} else if (elevator_handle.chState == ELEVATOR_ZERO) {
		state = 2;
	} else if (elevator_handle.chState == ELEVATOR_ISEND) {
		state = 3;
	} else if (elevator_handle.chState == ELEVATOR_END) {
		state = 4;
	} else if (elevator_handle.chState == ELEVATOR_ISZERO) {
		state = 5;
	} else { // 急停状态，后续补充
		state = 6;
	}
	return state;
}
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
static void super_thread_entry(void *p1, void *p2, void *p3)
{
	/* Initialize LED indicator */
	if (!device_is_ready(led.port)) {
		LOG_ERR("LED device not ready");
		return;
	}
	k_msleep(1000);
	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED (err %d)", ret);
	}

	/* Initialize brake pin */
	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
	ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure brake pin (err %d)", ret);
	}

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

	const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
	ret = gpio_pin_configure_dt(&prx_switch, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure proximity switch (err %d)", ret);
	} else {
		LOG_INF("Proximity switch configured");
	}

	const struct gpio_dt_spec stop = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
	ret = gpio_pin_configure_dt(&stop, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure stop button (err %d)", ret);
	} else {
		LOG_INF("stop button configured");
	}

	/* Initial delay for hardware stabilization */
	k_msleep(10);

	/* Main control loop */
	while (1) {
		/* Toggle watchdog */
		gpio_pin_toggle_dt(&w_dog);
		/* Run motor control tasks */
		super_elevator_task(NULL);

		// if (motor_get_state(motor) == MOTOR_STATE_FAULT) {
		// 	gpio_pin_set_dt(&mot12_brk, 0);
		// } else {
		// 	gpio_pin_set_dt(&mot12_brk, 1);
		// }
		k_msleep(1);
	}
}

/**
 * @struct motor_thread_data
 * @brief Motor thread control structure
 */
struct motor_thread_data {
	const struct device *motor_dev; ///< Motor device pointer
	struct k_thread thread;		///< Thread control block
};

/**
 * @brief Create motor control thread
 * @param dev Unused device pointer
 *
 * Creates high-priority cooperative thread for motor control.
 */
void creat_super_thread(const struct device *dev)
{
	static struct motor_thread_data thread_data __aligned(4);

	k_thread_create(&thread_data.thread, super_thread_stack,
			K_THREAD_STACK_SIZEOF(super_thread_stack), super_thread_entry, NULL, NULL,
			NULL,
			K_PRIO_COOP(5), // High priority cooperative thread
			0, K_NO_WAIT);
}
