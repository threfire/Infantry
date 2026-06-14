#ifndef CAMERA_CHANNEL_H
#define CAMERA_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#include "../../core/comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declare for use in hooks */
typedef struct camera_channel camera_channel_t;

/* Camera service SIDs */
#define CAM_SID_EVENT 0x0001u
#define CAM_SID_RESET 0x0002u


/**
 * @brief           摄像头通道回调钩子
 * @details         定义摄像头通道的事件通知接口
 */
typedef struct {
    void (*on_event)(camera_channel_t *cam, uint32_t frame_id, uint64_t ts_us, void *user); /**< 图像事件回调：新帧到达时触发 */
    void (*on_reset)(camera_channel_t *cam, void *user);                                    /**< 通道重置回调：通道状态重置时触发 */
    void *user;                                                                              /**< 用户上下文，透传给所有钩子函数 */
} camera_channel_hooks_t;

/**
 * @brief           摄像头内部事件记录
 * @details         用于诊断的摄像头事件和FIFO大小跟踪
 * 
 * @note            存储帧ID和时间戳，用于事件溯源和性能分析
 */
typedef struct {
    uint32_t frame_id;                  /**< 帧标识号 */
    uint64_t ts_us;                     /**< 事件发生时间戳（微秒） */
} cam_evt_t;


#ifndef CAM_FIFO_CAP
#define CAM_FIFO_CAP 10u
#endif

/**
 * @brief               摄像头数据通道实例
 * @details             管理摄像头图像流、事件上报和流控状态
 * 
 * @note                当前设计为全量发送模式，未设置速率限制字段
 */
struct camera_channel {
    ch_uproto_bind_t *bind;              /**< uproto传输绑定上下文 */
    uint8_t ch_id;                       /**< 逻辑通道ID */
    
    channel_t ch;                        /**< 基础通道对象 */
    channel_manager_t *mgr;              /**< 所属通道管理器 */
    
    uint64_t (*now_us)(void *user);      /**< 获取当前时间戳（微秒） */
    void *time_user;                     /**< 时间回调的用户上下文 */
    
    camera_channel_hooks_t hooks;        /**< 摄像头专用回调钩子 */
    
    uint32_t next_frame_id;              /**< 下一帧ID（递增） */
    uint8_t ack_pending;                 /**< 等待ACK标志：1=有未确认帧，0=空闲 */
    
    cam_evt_t fifo[CAM_FIFO_CAP];        /**< 摄像头事件FIFO缓冲 */
    uint8_t fifo_count;                  /**< FIFO当前事件数量 */
};

/* typedef kept above to satisfy older compilers' requirement */

/**
 * @brief                   摄像头通道配置
 * @details                 配置优先的API设计，使通道参数调优明确且便捷。
 *                          默认参数通过 camera_config.h 中的宏定义，可通过本结构体进行覆盖。
 * 
 * @see camera_config.h
 */
typedef struct {
    uint8_t ch_id;          /**< 逻辑通道 ID */
    uint8_t priority;       /**< 通道仲裁优先级 */
} camera_channel_cfg_t;


/**
  * @brief                  初始化并注册摄像头通道（扩展版，推荐）
  * @param[in] cam          摄像头通道对象
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] cfg          通道配置参数
  * @param[in] now_us       时间戳获取函数（可为NULL）
  * @param[in] time_user    时间戳函数的用户上下文
  * @param[in] hooks        事件回调钩子（可为NULL）
  * @retval                 none
  */
void camera_channel_init_ex(camera_channel_t *cam,
                            ch_uproto_bind_t *bind,
                            channel_manager_t *mgr,
                            const camera_channel_cfg_t *cfg,
                            uint64_t (*now_us)(void *user),
                            void *time_user,
                            const camera_channel_hooks_t *hooks);

/**
  * @brief                  初始化并注册摄像头通道（兼容版）
  * @param[in] cam          摄像头通道对象
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] ch_id        逻辑通道ID
  * @param[in] now_us       时间戳获取函数（可为NULL）
  * @param[in] time_user    时间戳函数的用户上下文
  * @param[in] hooks        事件回调钩子（可为NULL）
  * @retval                 none
  */
void camera_channel_init(camera_channel_t *cam,
                         ch_uproto_bind_t *bind,
                         channel_manager_t *mgr,
                         uint8_t ch_id,
                         uint64_t (*now_us)(void *user),
                         void *time_user,
                         const camera_channel_hooks_t *hooks);

/**
  * @brief                  摄像头触发边沿调用
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_trigger(camera_channel_t *cam);

/**
  * @brief                  周期性处理（发送延迟ACK等）
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_tick(camera_channel_t *cam);

/**
  * @brief                  显式复位（本地操作）
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_reset(camera_channel_t *cam);

/**
  * @brief                  获取下一帧ID
  * @param[in] cam          摄像头通道对象
  * @retval                 下一帧ID
  */
uint32_t camera_channel_next_frame_id(const camera_channel_t *cam);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_CHANNEL_H */
