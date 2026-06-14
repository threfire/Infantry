/**
  * @file       comm_app.h
  * @brief      设备侧通信应用对外接口
  * @details    建议先包含 comm_app_config.h 调整参数，再包含本头文件。
  */
#ifndef COMM_APP_H
#define COMM_APP_H

#include <stdint.h>
/* 可选：在工程中先包含此配置头以覆盖默认参数 */
#include "comm_app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief      启动通信应用任务（创建 FreeRTOS 任务）
  */
void comm_app_start(void);

/**
  * @brief      任务入口（如需由外部统一调度创建任务时使用）
  */
void comm_app_task(void *argument);

/**
  * @brief      EXTI 钩子：当相机触发 GPIO 有效沿到来时调用
  */
void comm_camera_trigger_pulse(void);

/* 可选 GPIO 轮询触发（当 CAM_TRIGGER_ENABLE=1 时启用）。
 * 注意：GPIO 初始化由 CubeMX 负责，这里只进行状态读取。 */
#if defined(CAM_TRIGGER_ENABLE) && (CAM_TRIGGER_ENABLE==1)
void comm_camera_trigger_poll(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMM_APP_H */
