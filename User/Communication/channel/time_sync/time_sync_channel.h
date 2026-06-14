#ifndef TIME_SYNC_CHANNEL_H
#define TIME_SYNC_CHANNEL_H

#include <stdint.h>
#include <stdbool.h>
#include "../../core/comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SIDs for time sync over MUX */
#define TS_SID_REQ   0x1001u
#define TS_SID_RESP  0x1002u

/**
 * @brief       时间源操作接口（微秒级）
 * @details     可配置的时间获取API，便于参数调优和平台适配
 */
typedef struct {
    uint64_t (*now_us)(void *user); /**< 获取当前本地时间戳（微秒） */
    void *user;                     /**< 用户上下文，传递给 now_us 回调 */
} ts_time_ops_t;

/**
 * @brief       时间同步通道实例
 * @details     实现基于REQ/RESP模式的时间同步协议，支持主动发起方和响应方两种角色
 * 
 * @note        使用32槽历史记录环缓冲进行RTT估计和时钟偏移滤波
 */
struct time_sync_channel {
    ch_uproto_bind_t *bind;              /**< uproto传输绑定上下文 */
    channel_manager_t *mgr;              /**< 所属通道管理器 */
    channel_t ch;                        /**< 基础通道对象 */
    uint8_t ch_id;                       /**< 逻辑通道ID */

    ts_time_ops_t time;                  /**< 时间操作接口（获取系统时间等） */

    /* 配置参数 */
    uint32_t period_ms;                  /**< 同步周期（毫秒） */
    uint8_t initiator;                   /**< 角色：1=主动发起方（周期性发送REQ），0=被动响应方 */
    uint32_t max_rtt_us;                 /**< 最大允许RTT（微秒），0表示接受所有 */

    /* 运行时状态 */
    uint64_t last_req_us;                /**< 上次发送REQ的时间戳（微秒） */
    uint16_t seq;                        /**< 当前序列号 */

    /** @brief REQ发送时间历史记录环缓冲（用于RTT计算和异常检测） */
    struct { uint16_t seq; uint64_t t0; } hist[32]; /**< seq=序列号, t0=发送时刻 */
    uint8_t hist_pos;                    /**< 历史记录环缓冲当前位置 */

    /* 时钟映射状态 */
    uint32_t rtt_us_last;                /**< 最近一次成功同步的RTT */
    int64_t  offset_us;                  /**< 设备相对于主机的时钟偏移（滤波后，微秒） */
    int64_t  offset_origin_us;           /**< 基准时钟偏移（微秒） */
    uint8_t  offset_origin_valid;        /**< 基准偏移是否有效：1=已建立，0=未建立 */
    int64_t  offset_display_us;          /**< 显示用偏移量（相对于基准，微秒） */
    uint64_t last_device_time_us;        /**< 最近一次同步的设备时间t2（微秒） */
    uint64_t last_host_time_us;          /**< 最近一次同步的主机时间t3（微秒） */
    uint8_t  mapping_valid;              /**< 时钟映射是否有效：1=有效，0=无效 */
    uint32_t mapping_version;            /**< 映射版本号（每次成功同步递增） */
};

/**
 * @brief       时间同步通道配置
 * @details     定义时间同步通道的工作参数和仲裁行为
 */
typedef struct {
    uint8_t ch_id;        /**< 逻辑通道 ID */
    uint32_t period_ms;   /**< 同步周期（毫秒） */
    uint8_t initiator;    /**< 角色：1=主动发起方（周期性发送REQ），0=被动响应方 */
    uint8_t priority;     /**< 仲裁优先级，值越大优先级越高 */
    uint32_t max_rtt_us;  /**< 最大允许RTT（微秒），0=接受所有 */
} ts_channel_cfg_t;

typedef struct time_sync_channel time_sync_channel_t;

/**
  * @brief                  初始化时间同步通道
  * @param[in] ts           时间同步通道对象
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] cfg          时间同步通道配置
  * @param[in] time_ops     时间获取接口
  * @retval                 none
  */
void time_sync_channel_init_ex(time_sync_channel_t *ts,
                               ch_uproto_bind_t *bind,
                               channel_manager_t *mgr,
                               const ts_channel_cfg_t *cfg,
                               const ts_time_ops_t *time_ops);

/**
  * @brief                  初始化时间同步通道
  * @param[in] ts           时间同步通道对象
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] ch_id        逻辑通道 ID
  * @param[in] time_ops     时间获取接口
  * @param[in] period_ms    同步周期（毫秒）
  * @param[in] initiator    角色：1=主动发起方（周期性发送REQ），0=被动响应方
  * @param[in] priority     仲裁优先级，值越大优先级越高
  * @retval                 none
  */
void time_sync_channel_init(time_sync_channel_t *ts,
                            ch_uproto_bind_t *bind,
                            channel_manager_t *mgr,
                            uint8_t ch_id,
                            const ts_time_ops_t *time_ops,
                            uint32_t period_ms,
                            uint8_t initiator,
                            uint8_t priority);

/**
  * @brief                  时间同步通道定时任务
  * @param[in] ts           时间同步通道对象
  * @retval                 none
  */
void time_sync_channel_tick(time_sync_channel_t *ts);


/**
  * @brief                  获取当前时间戳（微秒）
  * @param[in] ts           时间同步通道对象
  * @retval                 时间戳（微秒）
  */
uint64_t time_sync_channel_now_us(const time_sync_channel_t *ts);

/**
  * @brief                  设置最大允许RTT（微秒）
  * @param[in] ts           时间同步通道对象
  * @param[in] max_rtt_us   最大允许RTT（微秒）
  * @retval                 none
  */
void time_sync_channel_set_max_rtt_us(time_sync_channel_t *ts, uint32_t max_rtt_us);

#ifdef __cplusplus
}
#endif

#endif /* TIME_SYNC_CHANNEL_H */
