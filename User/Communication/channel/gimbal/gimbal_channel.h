#ifndef GIMBAL_CHANNEL_H
#define GIMBAL_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#include "../../core/comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SIDs for gimbal over MUX */
#define GIMBAL_SID_STATE 0x0201u /* device -> host */
#define GIMBAL_SID_DELTA 0x0202u /* host -> device */
#define GIMBAL_SID_TFMINI 0x0203u /* device -> host */
#define GIMBAL_SID_REFEREE 0x0204u /* device -> host */
#define GIMBAL_SID_FIRE 0x0205u /* host -> device */
#define GIMBAL_SID_CHASSIS 0x0206u /* host -> device */
#define GIMBAL_SID_REFEREE_QUERY 0x0207u /* host -> device */

#define GIMBAL_TFMINI_STATUS_VALID 0x0001u
#define GIMBAL_REFEREE_STATUS_VALID 0x0001u
#define GIMBAL_REFEREE_STATUS_TIMEOUT 0x0002u
#define GIMBAL_REFEREE_STATUS_ERROR 0x0004u
#define GIMBAL_REFEREE_STATUS_NOT_READY 0x0008u

/**
 * @brief 云台通道前向声明
 */
typedef struct gimbal_channel gimbal_channel_t;

/**
 * @brief 云台状态数据
 * @details 包含编码器读数、IMU姿态角度（微度）和设备逻辑时间戳
 */
typedef struct {
    int32_t enc_yaw;   /**< 编码器读数或单位 */
    int32_t enc_pitch;
    int32_t yaw_udeg;   /**< IMU偏航角，单位微度 */
    int32_t pitch_udeg; /**< IMU俯仰角，单位微度 */
    int32_t roll_udeg;  /**< IMU横滚角，单位微度 */
    uint64_t ts_us;     /**< 设备逻辑时间戳（微秒） */
} gimbal_state_t;

/**
 * @brief 云台控制增量命令
 * @details 包含期望的偏航/俯仰变化量和主机时间戳
 */
typedef struct {
    int32_t delta_yaw_udeg;   /**< 期望的偏航角变化量，单位微度 */
    int32_t delta_pitch_udeg; /**< 期望的俯仰角变化量，单位微度 */
    uint16_t status;          /**< 保留位域 */
    uint64_t ts_us;           /**< 主机时间戳（微秒） */
} gimbal_delta_t;

/**
 * @brief TFmini range data
 * @details Published via gimbal channel, see units in fields
 */
/**
 * @brief TFmini测距数据
 * @details 通过gimbal通道发布，单位见字段说明
 */
typedef struct {
    uint16_t distance_cm; /**< 距离值，单位cm */
    uint16_t strength;    /**< 信号强度 */
    int16_t temp_cdeg;    /**< 温度值，单位0.01℃ */
    uint16_t status;      /**< 状态位（bit0=有效数据） */
    uint64_t ts_us;       /**< 设备逻辑时间戳（微秒） */
} gimbal_tfmini_t;

/**
 * @brief 裁判系统关键状态（下位机上报）
 */
typedef struct {
    int32_t enemy_team;   /**< 0=未知, 1=红方, 2=蓝方 */
    int32_t fire_allowed; /**< 0=禁止开火, 1=允许开火 */
    int32_t robot_id;     /**< 本机机器人ID */
    int32_t game_stage;   /**< 比赛阶段 */
    uint16_t status;      /**< 状态位（GIMBAL_REFEREE_STATUS_*） */
    uint64_t ts_us;       /**< 设备逻辑时间戳（微秒） */
} gimbal_referee_t;

/**
 * @brief 上位机下发开火指令
 */
typedef struct {
    int32_t fire_on;      /**< 0=不开火, 1=开火 */
    int32_t fire_mode;    /**< 0=默认, 1=单发, 2=连发 */
    int32_t burst_count;  /**< 连发数量 */
    uint16_t status;      /**< 保留位域 */
    uint64_t ts_us;       /**< 主机映射到设备侧时间戳（微秒） */
} gimbal_fire_cmd_t;

/**
 * @brief 上位机下发底盘指令（经云台板转发）
 */
typedef struct {
    int32_t vx_mm_s;      /**< 前向速度 mm/s */
    int32_t vy_mm_s;      /**< 侧向速度 mm/s */
    int32_t wz_mdeg_s;    /**< 角速度 mdeg/s */
    int32_t mode;         /**< 底盘模式 */
    uint16_t status;      /**< 保留位域 */
    uint64_t ts_us;       /**< 主机映射到设备侧时间戳（微秒） */
} gimbal_chassis_cmd_t;

/**
 * @brief 云台状态源操作接口
 * @details 提供拉取当前云台状态的回调函数
 */
typedef struct {
    /** @brief 拉取当前云台状态，返回false表示状态不可用 */
    bool (*get_state)(gimbal_state_t *out, void *user);
    void *user;           /**< 用户上下文 */
    uint32_t period_ms;   /**< 状态发布周期（毫秒） */
} gimbal_source_ops_t;

/**
 * @brief 云台通道事件钩子
 */
typedef struct {
    /** @brief 接收到主机增量命令时调用 */
    void (*on_delta)(const gimbal_delta_t *delta, void *user);
    /** @brief 接收到主机开火命令时调用 */
    void (*on_fire)(const gimbal_fire_cmd_t *cmd, void *user);
    /** @brief 接收到主机底盘命令时调用 */
    void (*on_chassis)(const gimbal_chassis_cmd_t *cmd, void *user);
    /** @brief 接收到主机裁判查询命令时调用 */
    void (*on_referee_query)(void *user);
    void *user; /**< 用户上下文 */
} gimbal_hooks_t;

/**
 * @brief 云台通道实例
 * @details 管理云台状态发布和控制命令接收的完整上下文
 */
struct gimbal_channel {
    ch_uproto_bind_t *bind;              /**< uproto传输绑定上下文 */
    channel_manager_t *mgr;              /**< 所属通道管理器 */
    channel_t ch;                        /**< 基础通道对象 */
    uint8_t ch_id;                       /**< 逻辑通道ID */

    gimbal_source_ops_t src;             /**< 状态源操作接口 */
    gimbal_hooks_t hooks;                /**< 事件回调钩子 */

    uint64_t last_pub_us;                /**< 上次发布状态的时间戳（微秒） */
    uint64_t (*now_us)(void *user);      /**< 获取当前时间戳（微秒） */
    void *time_user;                     /**< 时间回调的用户上下文 */
};

/* Config-first API: prefer explicit structs over macros for tuning. */
/**
 * @brief 云台通道配置
 * @details 配置优先的API：使用显式结构体而非宏，便于参数调优
 */
typedef struct {
    uint8_t ch_id;      /**< 逻辑通道ID */
    uint8_t priority;   /**< 仲裁优先级 */
    uint32_t period_ms; /**< 状态发布周期（非零时覆盖src->period_ms） */
} gimbal_channel_cfg_t;

/**
 * @brief 云台控制命令
 * @details 包含偏航/俯仰增量、状态码、时间戳和版本号
 */
typedef struct {
    int32_t delta_yaw_udeg;   /**< 偏航角增量，单位微度 */
    int32_t delta_pitch_udeg; /**< 俯仰角增量，单位微度 */
    uint16_t status;          /**< 状态码 */
    uint64_t ts_us;           /**< 时间戳，单位微秒 */
    uint32_t version;         /**< 版本号 */
} gimbal_cmd_t;

/**
  * @brief                  使用配置结构体初始化云台通道
  * @param[in] gc           云台通道实例
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] cfg          通道配置参数
  * @param[in] src          状态源操作接口
  * @param[in] hooks        事件回调钩子
  * @param[in] now_us       时间戳获取函数
  * @param[in] time_user    时间回调的用户上下文
  * @retval                 none
  */
void gimbal_channel_init_ex(gimbal_channel_t *gc,
                            ch_uproto_bind_t *bind,
                            channel_manager_t *mgr,
                            const gimbal_channel_cfg_t *cfg,
                            const gimbal_source_ops_t *src,
                            const gimbal_hooks_t *hooks,
                            uint64_t (*now_us)(void *user),
                            void *time_user);

/**
  * @brief                  初始化云台通道（兼容版参数）
  * @param[in] gc           云台通道实例
  * @param[in] bind         uproto传输绑定上下文
  * @param[in] mgr          所属通道管理器
  * @param[in] ch_id        逻辑通道ID
  * @param[in] src          状态源操作接口
  * @param[in] hooks        事件回调钩子
  * @param[in] now_us       时间戳获取函数
  * @param[in] time_user    时间回调的用户上下文
  * @param[in] priority     仲裁优先级
  * @retval                 none
  */
void gimbal_channel_init(gimbal_channel_t *gc,
                         ch_uproto_bind_t *bind,
                         channel_manager_t *mgr,
                         uint8_t ch_id,
                         const gimbal_source_ops_t *src,
                         const gimbal_hooks_t *hooks,
                         uint64_t (*now_us)(void *user),
                         void *time_user,
                         uint8_t priority);

/**
  * @brief                  立即发布云台状态快照
  * @param[in] gc           云台通道实例
  * @param[in] out          待发布的状态数据（NULL时使用src获取）
  * @retval                 true=成功，false=失败
  */
bool gimbal_channel_publish(gimbal_channel_t *gc, const gimbal_state_t *out);

/**
  * @brief                  Publish TFmini range data
  * @param[in] gc           gimbal channel instance
  * @param[in] data         TFmini range data
  * @retval                 true=ok, false=failed
  */
/**
  * @brief                  发布TFmini测距数据
  * @param[in] gc           云台通道实例
  * @param[in] data         TFmini测距数据
  * @retval                 true=成功，false=失败
  */
bool gimbal_channel_publish_tfmini(gimbal_channel_t *gc, const gimbal_tfmini_t *data);

/**
  * @brief                  发布裁判状态
  * @param[in] gc           云台通道实例
  * @param[in] data         裁判状态数据
  * @retval                 true=成功，false=失败
  */
bool gimbal_channel_publish_referee(gimbal_channel_t *gc, const gimbal_referee_t *data);

/**
  * @brief                    设置最新的云台控制命令（来自主机）
  * @param[in] cmd            云台控制命令指针
  * @retval                   none
  * @note                     线程安全，适用于Cortex-M单写多读场景
  */
void gimbal_mailbox_set(const gimbal_cmd_t *cmd);

/**
  * @brief                    读取最新的云台控制命令
  * @param[out] out           输出命令缓冲区
  * @param[in,out] io_version 输入当前版本号，输出新命令版本号
  * @retval                   true=有新命令，false=无新命令或参数无效
  */
bool gimbal_mailbox_get(gimbal_cmd_t *out, uint32_t *io_version);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CHANNEL_H */
