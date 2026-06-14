/**
  * @file       comm.h
  * @brief      通信模块
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2077-01-01      xxxxx           1. Created this file
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */
 
#ifndef COMM_H
#define COMM_H

#include "config.h"
#include "comm_utils.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declare to avoid leaking full uproto API from public comm.h */
typedef struct uproto_context uproto_context_t;

/* Forward declare channel type so it can be used in structs below */
typedef struct channel channel_t;

/************************** Types Declaration *************************************/

/**
  * @brief       通道管理器
  *
  * @details     管理一组通道指针及其数量统计
  */
typedef struct {
    channel_t   *slots[CH_MAX];     /**< 通道指针数组 */
    uint8_t     count;              /**< 当前通道数量 */
} channel_manager_t;

/**
  * @brief       通道工作模式
  */
typedef enum {
    CH_MODE_QA      = 0,            /**< 请求/响应模式 */
    CH_MODE_PUSH    = 1             /**< 周期性或事件驱动推送模式 */
} ch_mode_t;

/**
  * @brief       通道配置
  * @details     完整描述通道的工作参数和行为特性
  *
  * @note        根据 mode 不同，部分字段的语义会发生变化
  */
typedef struct {
    ch_mode_t   mode;               /**< 通道工作模式（CH_MODE_QA 或 CH_MODE_PUSH） */
    uint8_t     reliable;           /**< 传输可靠性：1=需要ACK确认，0=无需确认 */
    uint8_t     expect_reply;       /**< QA模式专用：1=期待响应帧，0=发后不管 */
    uint8_t     priority;           /**< 调度器优先级提示，值越大优先级越高 */
    uint32_t    period_ms;          /**< PUSH模式专用：推送周期，单位毫秒 */
} channel_cfg_t;

/**
  * @brief       通道回调函数表（C 语言虚函数表模拟）
  * @details     定义通道事件处理接口，通过函数指针实现类似面向对象的多态机制
  *
  * @note        所有回调函数均包含 @p user 参数，用于传递用户私有上下文指针
  */
typedef struct {
    /**
      * @brief          通道初始化回调
      * @param[in]      ch：通道对象
      * @param[in]      payload：payload数据
      * @param[in]      len：payload长度
      * @param[in]      user：用户私有上下文指针
      * @retval         none
      */
    void (*on_rx)(channel_t *ch, const uint8_t *payload, uint32_t len, void *user); /**< 数据接收事件回调 */

    /**
      * @brief          频道定时器回调
      * @param[in]      ch：通道对象
      * @param[in]      user：用户私有上下文指针
      * @retval         none
      */
    void (*on_tick)(channel_t *ch, void *user);                                     /**< 定时器心跳回调（周期性调用） */

    /**
      * @brief          频道配置变更回调
      * @param[in]      ch：通道对象
      * @param[in]      cfg：新的配置参数
      * @param[in]      user：用户私有上下文指针
      * @retval         none
      */
    void (*on_cfg_change)(channel_t *ch, const channel_cfg_t *cfg, void *user);     /**< 配置变更事件回调 */
} channel_hooks_t;


/**
  * @brief       通道实例
  * @details     完整描述一个通道对象，包含配置、回调、传输绑定及运行状态
  */
struct channel {
    uint8_t         id;                     /**< 逻辑通道ID */
    channel_cfg_t   cfg;                    /**< 通道配置参数 */
    channel_hooks_t hooks;                  /**< 事件回调函数表 */
    void            *user;                  /**< 用户私有上下文，透传给所有钩子函数 */
    void            *transport;             /**< 传输层绑定句柄（不透明指针，如 uproto ctx） */
    uint8_t         msg_type;               /**< 传输层消息类型标识 */
    uint8_t         handshake_done;         /**< 握手完成标志：1=已完成，0=未完成 */
};

typedef struct channel channel_t;

/**
  * @brief       MUX 协议帧头（紧凑格式）
  * @details     定义 MUX 协议的数据帧头结构，使用 packed 属性确保无填充字节
  * 
  * @note        总长度为 16 字节，符合 __attribute__((packed)) 紧凑布局
  */
typedef struct __attribute__((packed)) {
    uint8_t     sof;                        /**< 起始帧标志：固定值 0xA5 */
    uint8_t     ver;                        /**< 协议版本：当前为 0x01 */
    uint8_t     channel;                    /**< 逻辑通道 ID */
    uint8_t     flags;                      /**< 控制标志：bit0=REL(可靠传输), bit1=RESP(响应帧), bit2=FRAG(分片) */
    uint16_t    len;                        /**< 有效载荷长度（字节数） */
    uint16_t    sid;                        /**< 子命令 ID */
    uint32_t    seq;                        /**< 通道内序列号，每帧递增 */
    uint16_t    rsv;                        /**< 保留字段，必须填 0 */
} mux_hdr_t;


enum { 
    MUX_SOF = 0xA5,
    MUX_VER = 0x01 
};

/**
  * @brief       uproto 传输层绑定上下文
  * @details     将 MUX 协议绑定到 uproto 传输层的完整上下文，管理序列号、待发队列和调度状态
  */
typedef struct {
    uproto_context_t    *ctx;                       /**< uproto 协议栈上下文句柄 */
    uint8_t             msg_type;                   /**< 承载 MUX 帧的 uproto 消息类型 */
    channel_manager_t   *mgr;                       /**< 通道管理器实例 */
    uint32_t            seq_counters[CH_MAX];       /**< 各通道当前序列号计数器 */
    
    /** 
      * @brief 通道待发队列（保持最新策略） 
      * 
      */
    struct {
        uint8_t         used;                       /**< 该队列项是否有效（1=有效，0=空） */
        uint8_t         ch_id;                      /**< 所属逻辑通道 ID */
        uint16_t        sid;                        /**< 子命令 ID */
        uint8_t         flags;                      /**< MUX 帧标志 */
        uint16_t        len;                        /**< 有效载荷长度 */
        uint8_t         payload[COMM_CHANNEL_PENDING_PAYLOAD_SIZE]; /**< 载荷缓冲区（摄像头和控制类消息足够） */
        uint16_t        age;                        /**< 仲裁老化计数，防止饿死 */
    } pending[CH_MAX];                              /**< 各通道独立的待发队列 */
    
    uint8_t             rr_cursor;                  /**< 轮询调度游标，用于平局仲裁 */
} ch_uproto_bind_t;


/************************** Functions Declaration *************************************/

/**
  * @brief          通道管理器实例
  * @details        管理所有通道实例，提供注册、查找和调度功能
  * @param[in]      ch：通道对象
  * @retval         none
  */
void chmgr_init(channel_manager_t *m);

/**
  * @brief          注册通道实例
  * @details        将通道实例注册到管理器，并返回成功与否
  * @param[in]      m：通道管理器实例
  * @param[in]      ch：通道对象
  * @retval         true：注册成功
  * @retval         false：注册失败
  */
bool chmgr_register(channel_manager_t *m, channel_t *ch);

/**
  * @brief          查找通道实例
  * @details        根据逻辑通道 ID 查找通道实例
  * @param[in]      m：通道管理器实例
  * @param[in]      id：逻辑通道 ID
  * @retval         通道对象指针
  */
channel_t *chmgr_find(channel_manager_t *m, uint8_t id);

/**
  * @brief          调度通道接收数据
  * @details        根据通道 ID 调度通道接收数据
  * @param[in]      m：通道管理器实例
  * @param[in]      id：逻辑通道 ID
  * @param[in]      payload：接收数据指针
  * @param[in]      len：接收数据长度
  * @retval         none
  */
void chmgr_dispatch_rx(channel_manager_t *m, uint8_t id, const uint8_t *payload, uint32_t len);

/**
  * @brief          通道调度
  * @details        调度所有通道，处理通道状态、定时任务和数据发送
  * @param[in]      m：通道管理器实例
  * @retval         none
  */
void chmgr_tick(channel_manager_t *m);

/**
  * @brief          通道初始化
  * @details        初始化通道对象，并绑定传输层
  * @param[in]      ch：通道对象
  * @param[in]      id：逻辑通道 ID
  * @param[in]      cfg：通道配置参数
  * @param[in]      hooks：通道事件回调函数表
  * @param[in]      user：用户私有上下文
  * @retval         none
  */
void channel_init(channel_t *ch,
                  uint8_t id,
                  const channel_cfg_t *cfg,
                  const channel_hooks_t *hooks,
                  void *user);

/**
  * @brief          通道绑定传输层
  * @details        将通道对象绑定到传输层，并指定消息类型
  * @param[in]      ch：通道对象
  * @param[in]      transport_ctx：传输层上下文指针
  * @param[in]      msg_type：传输层消息类型
  * @retval         none
  */
void channel_bind_transport(channel_t *ch, void *transport_ctx, uint8_t msg_type);

/**
  * @brief          通道配置
  * @details        设置通道配置参数
  * @param[in]      ch：通道对象
  * @param[in]      cfg：通道配置参数
  * @retval         none
  */
void channel_set_cfg(channel_t *ch, const channel_cfg_t *cfg);

/**
  * @brief          通道就绪状态
  * @details        判断通道是否就绪
  * @param[in]      ch：通道对象
  * @retval         true：通道就绪
  * @retval         false：通道未就绪
  */
bool channel_is_ready(const channel_t *ch);

/**
  * @brief          通道握手完成
  * @details        设置通道握手完成标志
  * @param[in]      ch：通道对象
  * @retval         none
  * @note           握手是特定于通道的；调用者实现实际的字节
  */
void channel_mark_handshake_done(channel_t *ch);

/**
  * @brief          MUX 编码
  * @details        将 MUX 头、有效载荷和 CRC16 编码为 uproto 消息
  * @param[out]     out：输出缓冲区指针
  * @param[in]      out_cap：输出缓冲区容量
  * @param[in]      hdr：MUX 头指针
  * @param[in]      payload：有效载荷指针
  * @retval         编码后的总字节数
  */
uint32_t mux_encode(uint8_t *out,
                    uint32_t out_cap,
                    const mux_hdr_t *hdr,
                    const uint8_t *payload);

/**
  * @brief          MUX 解码
  * @details        将 uproto 消息解码为 MUX 头和有效载荷
  * @param[in]      data：输入缓冲区指针
  * @param[in]      len：输入缓冲区长度
  * @param[out]     out_hdr：MUX 头指针
  * @param[out]     out_payload：有效载荷指针
  * @retval         true：解码成功
  * @retval         false：解码失败
  */
bool mux_decode(const uint8_t *data,
                uint32_t len,
                mux_hdr_t *out_hdr,
                const uint8_t **out_payload);

/**
  * @brief          UPROTO 绑定
  * @details        将通道对象绑定到 uproto 上下文，并指定消息类型
  * @param[in]      b：通道对象
  * @param[in]      ctx：uproto 上下文指针
  * @param[in]      msg_type：uproto 消息类型
  * @param[in]      mgr：通道管理器实例
  * @retval         none
  */
void ch_uproto_bind(ch_uproto_bind_t *b,
                    uproto_context_t *ctx,
                    uint8_t msg_type,
                    channel_manager_t *mgr);

/**
  * @brief          UPROTO 注册接收
  * @details        注册通道对象接收数据处理函数
  * @param[in]      b：通道对象
  * @retval         none
  */
void ch_uproto_register_rx(ch_uproto_bind_t *b);

/**
  * @brief          UPROTO 发送通知
  * @details        发送通知帧
  * @param[in]      b：通道对象
  * @param[in]      ch_id：逻辑通道 ID
  * @param[in]      sid：服务 ID
  * @param[in]      flags：帧标志
  * @param[in]      payload：有效载荷指针
  * @param[in]      len：有效载荷长度
  * @retval         true：发送成功
  * @retval         false：发送失败
  */
bool ch_uproto_send_notify(ch_uproto_bind_t *b,
                           uint8_t ch_id,
                           uint16_t sid,
                           uint8_t flags,
                           const uint8_t *payload,
                           uint16_t len);

/**
  * @brief          UPROTO 队列发送通知
  * @details        将通知帧加入发送队列
  * @param[in]      b：通道对象
  * @param[in]      ch_id：逻辑通道 ID
  * @param[in]      sid：服务 ID
  * @param[in]      flags：帧标志
  * @param[in]      payload：有效载荷指针
  * @param[in]      len：有效载荷长度
  * @retval         true：加入队列成功
  * @retval         false：加入队列失败
  */
bool ch_uproto_queue_notify(ch_uproto_bind_t *b,
                            uint8_t ch_id,
                            uint16_t sid,
                            uint8_t flags,
                            const uint8_t *payload,
                            uint16_t len);

/**
  * @brief          UPROTO 仲裁器定时处理
  * @details        处理通道对象中的仲裁器
  * @param[in]      b：通道对象
  * @retval         none
  */
void ch_uproto_arbiter_tick(ch_uproto_bind_t *b);

#ifdef __cplusplus
}
#endif

#endif // COMM_H
