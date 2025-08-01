#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include "statemachine.h"
#include "zephyr/device.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <lib/motor/motor.h>
#include "s_posi_planning.h"
#include <lib/foc/foc.h>

/* Module logging setup */
LOG_MODULE_REGISTER(super_thread, LOG_LEVEL_DBG);

/* Device tree node aliases */
#define LED0_NODE	       DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE     DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC	       DT_NODELABEL(encoder_vcc)
#define W_DOG		       DT_NODELABEL(wdog)
#define P_SWITCH	       DT_NODELABEL(proximity_switch)
#define STOP_BUTTON	       DT_NODELABEL(stopbutton)
#define ZERO_POSI_IOSTATE      (1)
#define EMERGENCY_STOP_IOSTATE (1)
#define RISING_CMD	       (1)
#define FALLING_CMD	       (2)
#define RISING_DIS	       3000.0f
K_THREAD_STACK_DEFINE(super_thread_stack, 2048);

uint8_t conctrl_cmd = 0;
static fsm_cb_t elevator_handle = {
	.chState = 0,
};
static struct k_thread superlift_thread;
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// static fsm_rt_t superlift_NoReady(fsm_cb_t *obj);
// static fsm_rt_t superlift_ZeroPoint(fsm_cb_t *obj);
// static fsm_rt_t superlift_Rising(fsm_cb_t *obj);
// static fsm_rt_t superlift_HigPoint(fsm_cb_t *obj);
// static fsm_rt_t superlift_Falling(fsm_cb_t *obj);
// static fsm_rt_t superlift_EmergencyStop(fsm_cb_t *obj);
// static fsm_rt_t superlift_Motorfault(fsm_cb_t *obj);
static fsm_rt_t superlift_Idle(fsm_cb_t *obj);

/* GPIO device specification */
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

	// if (elevator_handle.current_state == superlift_NoReady) {
	// 	state = 1;
	// } else if (elevator_handle.current_state == superlift_ZeroPoint) {
	// 	state = 2;
	// } else if (elevator_handle.current_state == superlift_Rising) {
	// 	state = 3;
	// } else if (elevator_handle.current_state == superlift_HigPoint) {
	// 	state = 4;
	// } else if (elevator_handle.current_state == superlift_Falling) {
	// 	state = 5;
	// } else if (elevator_handle.current_state == superlift_EmergencyStop) {
	// 	if (elevator_handle.previous_state == superlift_ZeroPoint) {
	// 		state = 2;
	// 	} else if (elevator_handle.previous_state == superlift_HigPoint) {
	// 		state = 4;
	// 	} else if (elevator_handle.previous_state == superlift_Falling) {
	// 		state = 6;
	// 	} else if (elevator_handle.previous_state == superlift_Rising) {
	// 		state = 6;
	// 	} else if (elevator_handle.previous_state == superlift_NoReady) {
	// 		state = 1;
	// 	}
	// } else {
	// 	state = 255;
	// }
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
	statemachine_init(&elevator_handle, "superlift_fsm", superlift_Idle, NULL, NULL, 0);
	// const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));

	/* Main control loop */
	while (1) {
		/* Toggle watchdog */
		// gpio_pin_toggle_dt(&w_dog);
		// /* Run motor control tasks */
		// if (motor_get_state(motor) == MOTOR_STATE_FAULT &&
		//     elevator_handle.current_state != superlift_Motorfault) {
		// 	TRAN_STATE(&elevator_handle, superlift_Motorfault);
		// }
		DISPATCH_FSM(&elevator_handle);
		// conctrl_cmd = 0;
		k_msleep(1);
	}
}

// static fsm_rt_t superlift_NoReady(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		ELEVATOR_WAIT,
// 		ELEVATOR_FINDZERO,
// 		EMERGENCY_STOP,
// 	};
// 	const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
// 	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
// 	int switch_state;
// 	int8_t stop_state;
// 	switch_state = gpio_pin_get_dt(&prx_switch);
// 	stop_state = gpio_pin_get_dt(&stop_bt);
// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter NoReady");
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (switch_state != ZERO_POSI_IOSTATE) {
// 			if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 				TRAN_STATE(&elevator_handle, superlift_EmergencyStop);
// 				break;
// 			}
// 			if (conctrl_cmd == FALLING_CMD) {
// 				conctrl_cmd = 0;
// 				if (motor_get_mode(motor) != MOTOR_MODE_SPEED) {
// 					motor_set_mode(motor, MOTOR_MODE_SPEED);
// 				}
// 				obj->chState = ELEVATOR_WAIT;
// 			}
// 		} else {
// 			TRAN_STATE(&elevator_handle, superlift_ZeroPoint);
// 		}
// 		break;
// 	case ELEVATOR_WAIT:
// 		if (motor_get_state(motor) != MOTOR_STATE_READY) {
// 			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
// 			LOG_INF("motor enable");
// 			break;
// 		}
// 		gpio_pin_set_dt(&mot12_brk, 1);
// 		motor_set_state(motor, MOTOR_CMD_SET_START);
// 		motor_set_target(motor, 2.5f);
// 		obj->chState = ELEVATOR_FINDZERO;
// 		break;
// 	case ELEVATOR_FINDZERO:
// 		if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 			TRAN_STATE(&elevator_handle, superlift_EmergencyStop);
// 			break;
// 		}

// 		if (switch_state == 1) {
// 			// motor_set_target(motor, 0.0f);
// 			// motor_set_mode(motor, MOTOR_MODE_POSI);
// 			TRAN_STATE(&elevator_handle, superlift_ZeroPoint);
// 		}
// 		break;

// 	case EXIT:
// 		gpio_pin_set_dt(&mot12_brk, 0);
// 		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
// 		LOG_INF("exit NoReady");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static fsm_rt_t superlift_ZeroPoint(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		READY,
// 		READY1
// 	};
// 	// const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
// 	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
// 	int8_t stop_state;
// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter ZeroPoint motor_mode:%d", motor_get_mode(motor));

// 		if (motor_get_mode(motor) != MOTOR_MODE_POSI) {
// 			motor_set_mode(motor, MOTOR_MODE_POSI);
// 			break;
// 		}
// 		gpio_pin_set_dt(&mot12_brk, 0);
// 		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		stop_state = gpio_pin_get_dt(&stop_bt);
// 		if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 			TRAN_STATE(&elevator_handle, superlift_EmergencyStop);
// 			break;
// 		}
// 		if (conctrl_cmd != RISING_CMD) {
// 			break;
// 		}
// 		obj->chState = READY;
// 		break;
// 	case READY:
// 		motor_clear_realodom(motor, 0.0f);
// 		gpio_pin_set_dt(&mot12_brk, 1);
// 		if (motor_get_state(motor) != MOTOR_STATE_READY) {
// 			float posi = -RISING_DIS;
// 			motor_set_target_position(motor, posi, 0.0f);
// 			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
// 			break;
// 		}
// 		motor_set_state(motor, MOTOR_CMD_SET_START);
// 		obj->chState = READY1;
// 		break;
// 	case READY1:
// 		if (motor_get_state(motor) == MOTOR_STATE_CLOSED_LOOP) {
// 			TRAN_STATE(&elevator_handle, superlift_Rising);
// 		}
// 		break;
// 	case EXIT:
// 		LOG_INF("exit ZeroPoint");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static fsm_rt_t superlift_Rising(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		EMERGENCY_STOP,
// 		EMERGENCY_STOP_RISING,
// 		EMERGENCY_STOP_FALLING
// 	};
// 	static uint16_t conut = 0;

// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
// 	int8_t stop_state;
// 	stop_state = gpio_pin_get_dt(&stop_bt);

// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter Rising");
// 		conut = 0;
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 			TRAN_STATE(obj, superlift_EmergencyStop);
// 			break;
// 		}
// 		if (fabsf(motor_get_current_position(motor)) < RISING_DIS - 0.001f) // 已经到达位置
// 		{
// 			conut = 0;
// 			break;
// 		}
// 		conut++;
// 		if (conut > 2500) {
// 			conut = 0;
// 			TRAN_STATE(&elevator_handle, superlift_HigPoint);
// 		}
// 		break;
// 	case EXIT:
// 		LOG_INF("exit Rising");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static fsm_rt_t superlift_HigPoint(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		READY,
// 	};
// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
// 	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);

// 	int8_t stop_state;
// 	stop_state = gpio_pin_get_dt(&stop_bt);

// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter HigPoint");
// 		gpio_pin_set_dt(&mot12_brk, 0);
// 		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 			TRAN_STATE(&elevator_handle, superlift_EmergencyStop);
// 			break;
// 		}
// 		// 急停解除
// 		if (conctrl_cmd != FALLING_CMD) {
// 			break;
// 		}
// 		// 准备下降
// 		obj->chState = READY;
// 		break;
// 	case READY:
// 		gpio_pin_set_dt(&mot12_brk, 1);
// 		if (motor_get_state(motor) != MOTOR_STATE_READY) {
// 			float posi;
// 			posi = motor_get_current_position(motor);
// 			motor_set_target_position(motor, 0.0f, posi);
// 			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
// 			break;
// 		}
// 		motor_set_state(motor, MOTOR_CMD_SET_START);
// 		TRAN_STATE(&elevator_handle, superlift_Falling);
// 		break;
// 	case EXIT:
// 		LOG_INF("exit HigPoint");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static fsm_rt_t superlift_Falling(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		EMERGENCY_STOP,
// 		EMERGENCY_STOP_RISING,
// 		EMERGENCY_STOP_FALLING
// 	};
// 	const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);

// 	int switch_state;
// 	int8_t stop_state;
// 	stop_state = gpio_pin_get_dt(&stop_bt);

// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter Falling");
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (stop_state == EMERGENCY_STOP_IOSTATE) {
// 			TRAN_STATE(obj, superlift_EmergencyStop);
// 			break;
// 		}
// 		switch_state = gpio_pin_get_dt(&prx_switch);
// 		if (switch_state != 1) {
// 			break;
// 		}
// 		TRAN_STATE(&elevator_handle, superlift_ZeroPoint);
// 		break;
// 	case EXIT:
// 		LOG_INF("exit Falling");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static fsm_rt_t superlift_EmergencyStop(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 		SET_RISING_DIS,
// 		SET_FALLING_DIS,
// 		SET_RISING_MOTOR_ENABLE,
// 		SET_FALLING_MOTOR_ENABLE
// 	};
// 	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
// 	const struct gpio_dt_spec stop_bt = GPIO_DT_SPEC_GET(STOP_BUTTON, gpios);
// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));

// 	// int switch_state;
// 	int8_t stop_state;
// 	stop_state = gpio_pin_get_dt(&stop_bt);
// 	switch (obj->chState) {
// 	case ENTER:
// 		gpio_pin_set_dt(&mot12_brk, 0);
// 		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
// 		LOG_INF("enter EmergencyStop");
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (stop_state != EMERGENCY_STOP_IOSTATE) {
// 			if (obj->previous_state == superlift_NoReady) {
// 				TRAN_STATE(obj, superlift_NoReady);
// 				break;
// 			} else if (obj->previous_state == superlift_ZeroPoint) {
// 				TRAN_STATE(obj, superlift_ZeroPoint);
// 				break;
// 			} else if (obj->previous_state == superlift_HigPoint) {
// 				TRAN_STATE(obj, superlift_HigPoint);
// 				break;
// 			} else if (obj->previous_state == superlift_Rising ||
// 				   obj->previous_state == superlift_Falling) {
// 				if (conctrl_cmd == RISING_CMD) {
// 					gpio_pin_set_dt(&mot12_brk, 1);
// 					float posi;
// 					posi = motor_get_current_position(motor);
// 					motor_set_target_position(motor, -RISING_DIS, posi);
// 					motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
// 					obj->chState = SET_RISING_DIS;
// 				} else if (conctrl_cmd == FALLING_CMD) {
// 					gpio_pin_set_dt(&mot12_brk, 1);
// 					float posi;
// 					posi = motor_get_current_position(motor);
// 					motor_set_target_position(motor, 0.0f, posi);
// 					motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
// 					obj->chState = SET_FALLING_DIS;
// 				}
// 				break;
// 			}
// 		}
// 		break;
// 	case SET_RISING_DIS:
// 		motor_set_state(motor, MOTOR_CMD_SET_START);
// 		obj->chState = SET_RISING_MOTOR_ENABLE;
// 		break;
// 	case SET_FALLING_DIS:
// 		motor_set_state(motor, MOTOR_CMD_SET_START);
// 		obj->chState = SET_FALLING_MOTOR_ENABLE;
// 		break;
// 	case SET_RISING_MOTOR_ENABLE:
// 		if (motor_get_state(motor) == MOTOR_STATE_CLOSED_LOOP) {
// 			LOG_INF("motor enter close loop");
// 		}
// 		TRAN_STATE(obj, superlift_Rising);
// 		break;
// 	case SET_FALLING_MOTOR_ENABLE:
// 		TRAN_STATE(obj, superlift_Falling);
// 		break;
// 	case EXIT:
// 		LOG_INF("exit EmergencyStop");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }
// #include <zephyr/sys/reboot.h>
// static fsm_rt_t superlift_Motorfault(fsm_cb_t *obj)
// {
// 	enum {
// 		RUNING = USER_STATUS,
// 	};
// 	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
// 	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
// 	static uint16_t motor_count = 0;
// 	switch (obj->chState) {
// 	case ENTER:
// 		LOG_INF("enter Motorfault");
// 		gpio_pin_set_dt(&mot12_brk, 0);
// 		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
// 		motor_count = 0;
// 		obj->chState = RUNING;
// 		break;
// 	case RUNING:
// 		if (motor_get_state(motor) == MOTOR_STATE_IDLE) {
// 			if (motor_count++ > 3000) {
// 				// TRAN_STATE(obj, superlift_Idle);
// 				sys_reboot(SYS_REBOOT_COLD);
// 			}
// 		} else {
// 			motor_count = 0;
// 		}
// 		break;
// 	case EXIT:
// 		LOG_INF("exit Motorfault");
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }
static fsm_rt_t superlift_Idle(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
		RUNING1,
		RUNING2,
		RUNING3,
		RUNING4,
		IDLE,
	};
	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));
	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);

	switch (obj->chState) {
	case ENTER:

		LOG_INF("enter Superlift Idle");
		gpio_pin_set_dt(&mot12_brk, 1);
		if (motor_get_mode(motor) != MOTOR_MODE_POSI) {
			motor_set_mode(motor, MOTOR_MODE_POSI);
			break;
		}
		obj->chState = RUNING;
		break;
	case RUNING:
		if (motor_get_state(motor) != MOTOR_STATE_READY) {
			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
			obj->chState = RUNING1;
			break;
		}
		break;
	case RUNING1:
		if (motor_get_state(motor) != MOTOR_STATE_CLOSED_LOOP) {
			motor_set_state(motor, MOTOR_CMD_SET_START);
			break;
		}
		k_msleep(6000);
		motor_set_state(motor, MOTOR_CMD_SET_DISABLE);
		k_msleep(2000);
		obj->chState = RUNING2;
		break;
	case RUNING2:
		if (motor_get_mode(motor) != MOTOR_MODE_SPEED) {
			motor_set_mode(motor, MOTOR_MODE_SPEED);
			break;
		}
		if (motor_get_state(motor) != MOTOR_STATE_READY) {
			motor_set_state(motor, MOTOR_CMD_SET_ENABLE);
			obj->chState = RUNING3;
			break;
		}
		break;
	case RUNING3:
		if (motor_get_state(motor) != MOTOR_STATE_CLOSED_LOOP) {
			motor_set_state(motor, MOTOR_CMD_SET_START);
			break;
		}
		k_msleep(2000);
		obj->chState = RUNING4;
		break;
	case RUNING4:
		motor_set_target_speed(motor, -2.5f);
		k_msleep(2000);
		motor_set_target_speed(motor, 2.5f);
		k_msleep(3000);
		break;
	case EXIT:
		LOG_INF("exit Superlift Idle");
		break;
	default:
		break;
	}
	return 0;
}
static int init_super_thread(void)
{

	k_thread_create(&superlift_thread, super_thread_stack,
			K_THREAD_STACK_SIZEOF(super_thread_stack), super_thread_entry, NULL, NULL,
			NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);
	return 0;
}

SYS_INIT(init_super_thread, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);