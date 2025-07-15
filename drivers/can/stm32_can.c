#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stm32_can, LOG_LEVEL_DBG);

const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

// 前置声明
static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data);

int can_init(void)
{
    if (!device_is_ready(can_dev)) {
        LOG_INF("dev no ready");
        return -ENODEV;
    }
    // 停止 CAN 设备
    can_stop(can_dev);
    k_msleep(100);  // 等待设备完全停止

    struct can_timing timing;
    int ret = can_calc_timing(can_dev, &timing, 1000000, 875); // 10kbps, 87.5%采样点
    if (ret < 0) {
        return ret;
    }

    ret = can_set_timing(can_dev, &timing);
    if (ret < 0) {
        return ret;
    }

    // 设置模式前确保控制器就绪
    while (can_set_mode(can_dev, CAN_MODE_NORMAL) == -EBUSY) {
        k_msleep(1);
    }

    // LOG_INF("can init finish");

    struct can_filter filter = {
        .id = 0x00000000,  // 匹配所有标准ID
        .mask = 0x00000000,  // 掩码全0表示不检查任何位
        .flags = CAN_FILTER_IDE  // 明确指定标准帧        
    };
    int filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
    LOG_INF("Added filter %d (id=0x%x mask=0x%x)", filter_id, filter.id, filter.mask);    

    // 打印硬件状态
    uint32_t core_clock;
    can_get_core_clock(can_dev, &core_clock);
    // LOG_INF("CAN core clock: %u Hz", core_clock);

    can_start(can_dev); 
    return 0;
}
K_MSGQ_DEFINE(rx_msgq, sizeof(struct can_frame), 10, 4);

static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    k_msgq_put(&rx_msgq, frame, K_NO_WAIT); // 非阻塞入队
    // LOG_INF("RX ID:%03x DLC:%d Data:", frame->id, frame->dlc);
    // for (int i = 0; i < frame->dlc; i++) {
    //     LOG_INF("%02x ", frame->data[i]);
    // }
    // LOG_INF("");
}

