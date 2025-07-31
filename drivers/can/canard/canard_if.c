
/*

uavcan协议的接口文件



*/

#include "canard.h"
#include "zephyr/posix/sys/stat.h"
#include "zephyr/sys/util.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <dinosaurs/actuator/wheel_motor/Enable_1_0.h>
#include <dinosaurs/actuator/wheel_motor/SetTargetValue_2_0.h>
#include <dinosaurs/actuator/wheel_motor/PidParameter_1_0.h>
#include <dinosaurs/PortId_1_0.h>
#include <dinosaurs/actuator/wheel_motor/SetMode_2_0.h>
#include <dinosaurs/peripheral/OperateRemoteDevice_1_0.h>
#include <dinosaurs/peripheral/MovableAddons_1_0.h> // 添加头文件包含
#include <dinosaurs/PortId_1_0.h>
LOG_MODULE_REGISTER(canard_if, LOG_LEVEL_INF);

#define CANARD_MEM_POOL_SIZE 4096
static uint8_t canard_mem_pool[CANARD_MEM_POOL_SIZE] __aligned(4); // 静态内存池定义
static struct k_heap canard_heap;
static CanardInstance canard;
static CanardTxQueue txQueue;
extern const struct device *const can_dev;
K_THREAD_STACK_DEFINE(canard_thread_stack, 2048);
static struct k_thread thread; ///< 线程控制块
extern struct k_msgq rx_msgq;
static uint8_t heartbeat_transfer_id = 0;
static uint8_t movable_addons_transfer_id = 0;
static uint64_t last_movable_pub = 0;
static const uint16_t MOVABLE_ADDONS_PUB_INTERVAL_MS = 100; // 1秒发布间隔
static const CanardPortID MOVABLE_ADDONS_PORT_ID = 1022;    // 为MovableAddons分配的端口ID

static void subscribe_services(void);
static void handle_motor_enable(CanardRxTransfer *transfer);
static void handle_set_targe(CanardRxTransfer *transfer);
static void handle_pid_parameter(CanardRxTransfer *transfer);
static void handle_set_mode(CanardRxTransfer *transfer);
static void handle_operate_remote_device(CanardRxTransfer *transfer); // 新增操作远程设备回调

typedef void (*canard_subscription_callback_t)(CanardRxTransfer *);

#define NODE_ID (25)
extern int can_init(void);

static void *memAllocate(CanardInstance *const ins, size_t amount)
{
	(void)ins;
	void *ptr = k_heap_alloc(&canard_heap, amount, K_NO_WAIT);

	return ptr; // 使用Zephyr内存分配
}

static void memFree(CanardInstance *const ins, void *const pointer)
{
	(void)ins;
	k_heap_free(&canard_heap, pointer);
}

int canard_if_init(uint8_t node_id)
{
	k_heap_init(&canard_heap, canard_mem_pool, sizeof(canard_mem_pool));
	canard = canardInit(&memAllocate, &memFree);
	canard.node_id = node_id;
	txQueue = canardTxInit(100, CANARD_MTU_CAN_CLASSIC);
	return 0;
}

static int32_t canard_transmit(const CanardTxQueueItem *ti)
{
	struct can_frame frame = {.id = ti->frame.extended_can_id,
				  .dlc = ti->frame.payload_size,
				  .flags = CAN_FRAME_IDE};
	memcpy(frame.data, ti->frame.payload, ti->frame.payload_size);
	return can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
}

void canard_publish_heartbeat(void)
{
	{
		{
			const uint64_t uptime_sec = k_uptime_get() / 1000;
			uint8_t heartbeat_payload[7] = {
				(uint8_t)(uptime_sec & 0xFF),
				(uint8_t)((uptime_sec >> 8) & 0xFF),
				(uint8_t)((uptime_sec >> 16) & 0xFF),
				(uint8_t)((uptime_sec >> 24) & 0xFF),
				0x01, // Health状态 (NOMINAL)
				0x3F, // Mode状态 (OPERATIONAL)
				0x7F  // Vendor-specific状态
			};

			const CanardTransferMetadata metadata = {
				.priority = CanardPriorityNominal,
				.transfer_kind = CanardTransferKindMessage,
				.port_id = 7509, // UAVCAN.node.Heartbeat标准端口
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id = heartbeat_transfer_id++,
			};

			canardTxPush(&txQueue, &canard, 0, &metadata, sizeof(heartbeat_payload),
				     heartbeat_payload);
		}
	}
}

void canard_publish_movable_addons(uint16_t device_id, const char *device_name, uint8_t state_value)
{
	// 初始化MovableAddons消息
	dinosaurs_peripheral_MovableAddons_1_0 msg = {
		.state = {.timestamp =
				  {
					  .microsecond = k_uptime_get() * 1000 // 当前时间戳（微秒）
				  },
			  .current_state = state_value},
		.device_id = device_id};

	// 设置设备名称
	size_t name_len = strlen(device_name);
	if (name_len > uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_) {
		name_len = uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_;
	}
	msg.name.value.count = name_len;
	memcpy(msg.name.value.elements, device_name, name_len);

	// 序列化消息
	uint8_t buffer[dinosaurs_peripheral_MovableAddons_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
	size_t buffer_size = sizeof(buffer);
	int8_t ret = dinosaurs_peripheral_MovableAddons_1_0_serialize_(&msg, buffer, &buffer_size);

	if (ret < 0) {
		LOG_ERR("MovableAddons serialization failed: %d", ret);
		return;
	}

	// 设置传输元数据
	const CanardTransferMetadata metadata = {.priority = CanardPriorityNominal,
						 .transfer_kind = CanardTransferKindMessage,
						 .port_id = MOVABLE_ADDONS_PORT_ID,
						 .remote_node_id = CANARD_NODE_ID_UNSET,
						 .transfer_id = movable_addons_transfer_id++};

	// 推送到发送队列
	canardTxPush(&txQueue, &canard, 0, &metadata, buffer_size, buffer);
}

extern int8_t super_elevator_state(void);

static void canard_thread(void *p1, void *p2, void *p3)
{
	can_init();
	canard_if_init(NODE_ID);
	subscribe_services(); // 新增服务订阅

	while (1) {
		// 处理发送队列
		for (const CanardTxQueueItem *ti = NULL; (ti = canardTxPeek(&txQueue)) != NULL;) {
			if (canard_transmit(ti) == 0) {
				canard.memory_free(&canard, canardTxPop(&txQueue, ti));
			} else {
				break;
			}
		}

		static uint64_t last_heartbeat = 0;
		if (k_uptime_get() - last_heartbeat > 1000) {
			canard_publish_heartbeat();
			last_heartbeat = k_uptime_get();
		}

		if (k_uptime_get() - last_movable_pub > MOVABLE_ADDONS_PUB_INTERVAL_MS) {
			canard_publish_movable_addons(1, "ieb_motor_lift",
						      super_elevator_state()); // LOCK状态
			last_movable_pub = k_uptime_get();
		}
		// 新增接收处理
		struct can_frame frame;
		if (k_msgq_get(&rx_msgq, &frame, K_NO_WAIT) == 0) {
			CanardFrame canard_frame = {.extended_can_id = frame.id,
						    .payload_size = frame.dlc,
						    .payload = frame.data};

			CanardRxTransfer transfer;
			CanardRxSubscription *subscription = NULL;

			if (canardRxAccept(&canard, k_uptime_get() * 1000, &canard_frame, 0,
					   &transfer, &subscription) > 0) {
				if (subscription && subscription->user_reference) {
					canard_subscription_callback_t callback =
						(canard_subscription_callback_t)
							subscription->user_reference;
					callback(&transfer);
				}
				canard.memory_free(&canard, transfer.payload);
			}
		}
		k_msleep(1);
	}
}

void creat_canard_thread(void)
{
	k_thread_create(&thread, canard_thread_stack, K_THREAD_STACK_SIZEOF(canard_thread_stack),
			canard_thread, NULL, NULL, NULL,
			K_PRIO_COOP(4), // 高优先级协作线程
			0, K_NO_WAIT);
}

// 订阅服务函数
static void subscribe_services(void)
{
	static CanardRxSubscription sub_enable;
	static CanardRxSubscription sub_setTar;

	sub_enable.user_reference = (void *)handle_motor_enable; // 显式类型转换
	canardRxSubscribe(&canard, CanardTransferKindRequest, 113,
			  dinosaurs_actuator_wheel_motor_Enable_Request_1_0_EXTENT_BYTES_,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &sub_enable);
	sub_setTar.user_reference = handle_set_targe;
	canardRxSubscribe(&canard, CanardTransferKindRequest, 117, 16,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &sub_setTar);

	static CanardRxSubscription sub_pid_param;
	canardRxSubscribe(
		&canard, CanardTransferKindRequest,
		dinosaurs_PortId_1_0_dinosaurs_actuator_wheel_motor_PidParameter_1_0_FIXED_PORT_ID_,
		dinosaurs_actuator_wheel_motor_PidParameter_Request_1_0_EXTENT_BYTES_,
		CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &sub_pid_param);
	sub_pid_param.user_reference = (void *)handle_pid_parameter;

	static CanardRxSubscription sub_mode;
	canardRxSubscribe(&canard, CanardTransferKindRequest,
			  dinosaurs_PortId_1_0_actuator_wheel_motor_SetMode_2_0_ID,
			  dinosaurs_actuator_wheel_motor_SetMode_Request_2_0_EXTENT_BYTES_,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &sub_mode);
	sub_mode.user_reference = (void *)handle_set_mode;

	static CanardRxSubscription sub_remote_device;
	canardRxSubscribe(&canard, CanardTransferKindRequest,
			  121, // 为OperateRemoteDevice分配端口ID
			  dinosaurs_peripheral_OperateRemoteDevice_Request_1_0_EXTENT_BYTES_,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &sub_remote_device);
	sub_remote_device.user_reference = (void *)handle_operate_remote_device;
}
#include <lib/motor/motor.h>
extern uint8_t conctrl_cmd;
// 远程设备操作处理函数
static void handle_operate_remote_device(CanardRxTransfer *transfer)
{
	dinosaurs_peripheral_OperateRemoteDevice_Request_1_0 req = {0};
	size_t inout_size = transfer->payload_size;
	if (dinosaurs_peripheral_OperateRemoteDevice_Request_1_0_deserialize_(
		    &req, transfer->payload, &inout_size) >= 0) {
		// 转换设备名和参数为字符串
		char device_name[32] = {0};
		char device_param[32] = {0};
		memcpy(device_name, req.name.elements, req.name.count);
		memcpy(device_param, req.param.elements, req.param.count);

		// LOG_INF("Remote device operation: method=%u, name='%s', param='%s'", req.method,
		// 	device_name, device_param);
		if (!strcmp(device_name, "ieb_motor_lift")) {
			if (req.method ==
			    dinosaurs_peripheral_OperateRemoteDevice_Request_1_0_OPEN) {
				if (!strcmp(device_param, "1")) {
					// LOG_INF("rising");
					conctrl_cmd = 1;

				} else if (!strcmp(device_param, "0")) {
					// LOG_INF("falling");
					conctrl_cmd = 2;

				} else {
				}
			} else if (req.method ==
				   dinosaurs_peripheral_OperateRemoteDevice_Request_1_0_CLOSE) {
			} else {
			}
		} else if (!strcmp(device_name, "m-brake")) {

		} else {
		}

		// 准备响应
		dinosaurs_peripheral_OperateRemoteDevice_Response_1_0 resp = {
			.result = dinosaurs_peripheral_OperateRemoteDevice_Response_1_0_SUCESS};

		// 可选: 填充返回值
		resp.value.count =
			snprintf((char *)resp.value.elements, sizeof(resp.value.elements),
				 "Operation %s executed", device_name);

		uint8_t buffer[64];
		size_t buffer_size = sizeof(buffer);
		dinosaurs_peripheral_OperateRemoteDevice_Response_1_0_serialize_(&resp, buffer,
										 &buffer_size);

		const CanardTransferMetadata meta = {.priority = CanardPriorityNominal,
						     .transfer_kind = CanardTransferKindResponse,
						     .port_id = transfer->metadata.port_id,
						     .remote_node_id =
							     transfer->metadata.remote_node_id,
						     .transfer_id = transfer->metadata.transfer_id};
		canardTxPush(&txQueue, &canard, 0, &meta, buffer_size, buffer);
	}
}
static void handle_set_mode(CanardRxTransfer *transfer)
{
	dinosaurs_actuator_wheel_motor_SetMode_Request_2_0 req = {0};
	size_t inout_size = transfer->payload_size;

	if (dinosaurs_actuator_wheel_motor_SetMode_Request_2_0_deserialize_(&req, transfer->payload,
									    &inout_size) >= 0) {
		// 模式转换逻辑
		switch (req.mode) {
		case dinosaurs_actuator_wheel_motor_SetMode_Request_2_0_SPEED_MODE:
			// motor_cmd_set(MOTOR_CMD_SET_SPEED_MODE, NULL, 0);
			break;
		case dinosaurs_actuator_wheel_motor_SetMode_Request_2_0_CURRENT_MODE:
			// motor_cmd_set(MOTOR_CMD_SET_TORQUE_MODE, NULL, 0);
			break;
		// 其他模式处理...
		case dinosaurs_actuator_wheel_motor_SetMode_Request_2_0_POSITION_MODE:
			// motor_cmd_set(MOTOR_CMD_SET_POSTION_MODE, NULL, 0);
			break;
		}

		// 准备响应
		dinosaurs_actuator_wheel_motor_SetMode_Response_2_0 resp = {
			.status = dinosaurs_actuator_wheel_motor_SetMode_Response_2_0_SET_SUCCESS};

		uint8_t buffer
			[dinosaurs_actuator_wheel_motor_SetMode_Response_2_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t buffer_size = sizeof(buffer);
		dinosaurs_actuator_wheel_motor_SetMode_Response_2_0_serialize_(&resp, buffer,
									       &buffer_size);

		const CanardTransferMetadata meta = {.priority = CanardPriorityNominal,
						     .transfer_kind = CanardTransferKindResponse,
						     .port_id = transfer->metadata.port_id,
						     .remote_node_id =
							     transfer->metadata.remote_node_id,
						     .transfer_id = transfer->metadata.transfer_id};
		canardTxPush(&txQueue, &canard, 0, &meta, buffer_size, buffer);
	}
}
// 电机使能处理函数
static void handle_motor_enable(CanardRxTransfer *transfer)
{
	dinosaurs_actuator_wheel_motor_Enable_Request_1_0 req = {0};
	size_t inout_size = transfer->payload_size;

	if (dinosaurs_actuator_wheel_motor_Enable_Request_1_0_deserialize_(&req, transfer->payload,
									   &inout_size) >= 0) {

		LOG_INF("Motor enable cmd from node %d: %d", transfer->metadata.remote_node_id,
			req.enable_state);

		// 执行电机控制
		if (req.enable_state == 0) {
			// motor_cmd_set(MOTOR_CMD_SET_ENABLE,0,0);
		} else {
			// motor_cmd_set(MOTOR_CMD_SET_DISABLE,0,0);
		}
		// 发送响应
		dinosaurs_actuator_wheel_motor_Enable_Response_1_0 resp = {
			.status = dinosaurs_actuator_wheel_motor_Enable_Response_1_0_SET_SUCCESS};

		uint8_t buffer
			[dinosaurs_actuator_wheel_motor_Enable_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t buffer_size = sizeof(buffer);
		dinosaurs_actuator_wheel_motor_Enable_Response_1_0_serialize_(&resp, buffer,
									      &buffer_size);

		const CanardTransferMetadata meta = {.priority = CanardPriorityNominal,
						     .transfer_kind = CanardTransferKindResponse,
						     .port_id = transfer->metadata.port_id,
						     .remote_node_id =
							     transfer->metadata.remote_node_id,
						     .transfer_id = transfer->metadata.transfer_id};
		canardTxPush(&txQueue, &canard, 0, &meta, buffer_size, buffer);
	}
}
static void handle_set_targe(CanardRxTransfer *transfer)
{
	const uint8_t *data;
	size_t len;
	CanardNodeID sender_id;
	CanardPortID port_id;
	data = transfer->payload;
	sender_id = transfer->metadata.remote_node_id;
	len = transfer->payload_size;
	port_id = transfer->metadata.port_id;
	dinosaurs_actuator_wheel_motor_SetTargetValue_Request_2_0 req;
	size_t inout_size = len;
	if (dinosaurs_actuator_wheel_motor_SetTargetValue_Request_2_0_deserialize_(
		    &req, data, &inout_size) >= 0) {
		// LOG_INF("Node %u set targe: %f  %f", sender_id,
		// (double)req.velocity.elements[0].meter_per_second,
		// (double)req.velocity.elements[1].meter_per_second);

		/*

		*/
		// motor_set_ref_param(0,req.velocity.elements[0].meter_per_second,0.0f);

		float buf[2];
		buf[0] = req.velocity.elements[0].meter_per_second;
		buf[1] = req.velocity.elements[1].meter_per_second;
		// motor_cmd_set(MOTOR_CMD_SET_SPEED,buf,ARRAY_SIZE(buf));
		// 创建响应
		dinosaurs_actuator_wheel_motor_SetTargetValue_Response_2_0 response = {
			.status =
				dinosaurs_actuator_wheel_motor_SetTargetValue_Response_2_0_SET_SUCCESS};

		uint8_t buffer
			[dinosaurs_actuator_wheel_motor_SetTargetValue_Response_2_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t buffer_size = sizeof(buffer);
		uint8_t result =
			dinosaurs_actuator_wheel_motor_SetTargetValue_Response_2_0_serialize_(
				&response, buffer, &buffer_size); // 使用Response类型的序列化函数
		if (result < 0) {
			LOG_INF("Error serializing response\n");
			return;
		}
		const CanardTransferMetadata meta = {.priority = CanardPriorityNominal,
						     .transfer_kind = CanardTransferKindResponse,
						     .port_id = transfer->metadata.port_id,
						     .remote_node_id =
							     transfer->metadata.remote_node_id,
						     .transfer_id = transfer->metadata.transfer_id};
		canardTxPush(&txQueue, &canard, 0, &meta, buffer_size, buffer);
	}
}
static void handle_pid_parameter(CanardRxTransfer *transfer)
{
	dinosaurs_actuator_wheel_motor_PidParameter_Request_1_0 req = {0};
	size_t inout_size = transfer->payload_size;

	if (dinosaurs_actuator_wheel_motor_PidParameter_Request_1_0_deserialize_(
		    &req, transfer->payload, &inout_size) >= 0) {
		// LOG_INF("Received PID params from node %d: [%.6f, %.6f, %.6f, %.6f]",
		//        transfer->metadata.remote_node_id,
		//        (double)req.pid_params[0], (double)req.pid_params[1],
		//        (double)req.pid_params[2], (double)req.pid_params[3]);

		// 这里添加实际PID参数处理逻辑
		// 例如: pid_set_parameters(req.pid_params[0], req.pid_params[1],
		//       req.pid_params[2], req.pid_params[3]);

		float buf[4];
		buf[0] = req.pid_params[0];
		buf[1] = req.pid_params[1];

		// motor_cmd_set(MOTOR_CMD_SET_PIDPARAM,buf,ARRAY_SIZE(buf));
		// 准备响应
		dinosaurs_actuator_wheel_motor_PidParameter_Response_1_0 resp = {
			.status =
				dinosaurs_actuator_wheel_motor_PidParameter_Response_1_0_SET_SUCCESS};

		uint8_t buffer
			[dinosaurs_actuator_wheel_motor_PidParameter_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t buffer_size = sizeof(buffer);
		dinosaurs_actuator_wheel_motor_PidParameter_Response_1_0_serialize_(&resp, buffer,
										    &buffer_size);

		const CanardTransferMetadata meta = {.priority = CanardPriorityNominal,
						     .transfer_kind = CanardTransferKindResponse,
						     .port_id = transfer->metadata.port_id,
						     .remote_node_id =
							     transfer->metadata.remote_node_id,
						     .transfer_id = transfer->metadata.transfer_id};
		canardTxPush(&txQueue, &canard, 0, &meta, buffer_size, buffer);
	}
}
