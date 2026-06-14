#ifndef UPROTO_H
#define UPROTO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============ 配置参数 ============
#ifndef UPROTO_MAX_PAYLOAD
#define UPROTO_MAX_PAYLOAD      256
#endif
#ifndef UPROTO_MAX_TRANSACTIONS
#define UPROTO_MAX_TRANSACTIONS 8
#endif
#ifndef UPROTO_RX_BUFFER_SIZE
#define UPROTO_RX_BUFFER_SIZE   512
#endif
#ifndef UPROTO_MAX_HANDLERS
#define UPROTO_MAX_HANDLERS     16
#endif

// ============ 协议常量 ============
#define UPROTO_MAGIC_0          0xAA
#define UPROTO_MAGIC_1          0x55
#define UPROTO_VERSION          0x01
#define UPROTO_HEADER_SIZE      16

// Flags
#define UPROTO_FLAG_REQUEST     0x01
#define UPROTO_FLAG_RESPONSE    0x02
#define UPROTO_FLAG_ACK         0x04
#define UPROTO_FLAG_FRAG        0x10
#define UPROTO_FLAG_LAST_FRAG   0x20

// 系统消息类型 (0x00-0x0F 保留)
#define UPROTO_MSG_HELLO        0x01
#define UPROTO_MSG_HELLO_ACK    0x02
#define UPROTO_MSG_PING         0x03
#define UPROTO_MSG_PONG         0x04
#define UPROTO_MSG_ERROR        0x0F
#define UPROTO_MSG_CONTROL      0x11

// 用户消息类型从 0x10 开始
#define UPROTO_MSG_USER_BASE    0x10

// ============ 错误码/状态 ============
/**
 * @brief uproto错误码定义
 * @details 协议操作返回的状态码
 */
typedef enum {
    UPROTO_OK = 0,                  /**< 成功，无错误 */
    UPROTO_ERR_INVALID_PARAM,       /**< 无效参数 */
    UPROTO_ERR_NO_MEMORY,           /**< 内存不足 */
    UPROTO_ERR_TIMEOUT,             /**< 操作超时 */
    UPROTO_ERR_CRC,                 /**< CRC校验失败 */
    UPROTO_ERR_LENGTH,              /**< 数据长度错误 */
    UPROTO_ERR_VERSION,             /**< 版本不匹配 */
    UPROTO_ERR_NO_HANDLER,          /**< 无对应消息处理函数 */
    UPROTO_ERR_TX_FULL,             /**< 发送缓冲区满 */
    UPROTO_ERR_BUSY,                /**< 资源忙 */
} uproto_error_t;

/**
 * @brief uproto协议状态
 */
typedef enum {
    UPROTO_STATE_IDLE,          /**< 空闲状态 */
    UPROTO_STATE_HANDSHAKING,   /**< 握手进行中 */
    UPROTO_STATE_ESTABLISHED,   /**< 连接已建立 */
} uproto_state_t;

/**
 * @brief 事务状态
 */
typedef enum {
    TXN_STATE_FREE,       /**< 事务空闲 */
    TXN_STATE_WAITING,    /**< 等待响应 */
    TXN_STATE_COMPLETED,  /**< 事务完成 */
    TXN_STATE_TIMEOUT,    /**< 事务超时 */
} txn_state_t;

// 前置声明
/**
 * @brief uproto协议上下文前向声明
 */
typedef struct uproto_context uproto_context_t;

/**
 * @brief uproto帧结构前向声明
 */
typedef struct uproto_frame uproto_frame_t;

/**
 * @brief uproto事务结构前向声明
 */
typedef struct uproto_transaction uproto_transaction_t;

// ============ 回调/接口类型 ============
/**
 * @brief 端口写函数原型
 * @param[in] user   用户上下文
 * @param[in] data   待发送数据缓冲区
 * @param[in] len    数据长度（字节）
 * @retval           实际写入字节数
 */
typedef uint32_t (*uproto_port_write_fn)(void *user, const uint8_t *data, uint32_t len);

/**
 * @brief 端口刷新函数原型
 * @param[in] user   用户上下文
 * @retval           none
 */
typedef void (*uproto_port_flush_fn)(void *user);

/**
 * @brief 获取端口MTU函数原型
 * @param[in] user   用户上下文
 * @retval           最大传输单元（字节）
 */
typedef uint16_t (*uproto_port_get_mtu_fn)(void *user);

/**
 * @brief 获取当前时间函数原型（毫秒）
 * @param[in] user   用户上下文
 * @retval           当前时间戳（毫秒）
 */
typedef uint32_t (*uproto_time_now_fn)(void *user);

/**
 * @brief 事务完成回调函数原型
 * @param[in] user       用户上下文
 * @param[in] err        错误码
 * @param[in] rsp_data   响应数据缓冲区（可为NULL）
 * @param[in] rsp_len    响应数据长度
 * @retval               none
 */
typedef void (*uproto_txn_callback_fn)(void *user, uproto_error_t err,
                                       const uint8_t *rsp_data, uint32_t rsp_len);

/**
 * @brief 消息处理函数原型
 * @param[in] ctx      协议上下文
 * @param[in] txn_id   事务ID
 * @param[in] data     消息数据缓冲区
 * @param[in] len      数据长度
 * @param[in] user     用户上下文
 * @retval             none
 */
typedef void (*uproto_msg_handler_fn)(uproto_context_t *ctx, uint16_t txn_id,
                                      const uint8_t *data, uint32_t len, void *user);

/**
 * @brief 协议事件回调函数原型
 * @param[in] ctx    协议上下文
 * @param[in] event  事件名称字符串
 * @param[in] user   用户上下文
 * @retval           none
 */
typedef void (*uproto_event_callback_fn)(uproto_context_t *ctx, const char *event, void *user);

// ============ 数据结构 ============
/**
 * @brief uproto帧格式定义
 * @details 完整的协议帧结构，包含头部、载荷和CRC
 */
struct uproto_frame {
    uint8_t magic[2];                     /**< 帧魔数 */
    uint8_t version;                      /**< 协议版本 */
    uint8_t flags;                        /**< 控制标志 */
    uint8_t type;                         /**< 消息类型 */
    uint8_t reserved;                     /**< 保留字段 */
    uint16_t txn_id;                      /**< 事务ID */
    uint16_t seq;                         /**< 序列号 */
    uint16_t ack;                         /**< 确认号 */
    uint16_t len;                         /**< 载荷长度 */
    uint16_t hdr_crc;                     /**< 头部CRC校验 */
    uint8_t payload[UPROTO_MAX_PAYLOAD];  /**< 数据载荷 */
    uint16_t frame_crc;                   /**< 帧CRC校验 */
};

/**
 * @brief 协议事务上下文
 * @details 跟踪单个请求-响应事务的状态和参数
 */
struct uproto_transaction {
    txn_state_t state;         /**< 事务状态 */
    uint16_t txn_id;           /**< 事务ID */
    uint8_t type;              /**< 消息类型 */
    uint8_t retry_count;       /**< 当前重试次数 */
    uint8_t max_retries;       /**< 最大重试次数 */
    uint32_t timeout_ms;       /**< 超时时间（毫秒） */
    uint32_t start_time;       /**< 事务开始时间 */
    uint32_t last_tx_time;     /**< 最后发送时间 */
    uproto_txn_callback_fn callback; /**< 完成回调函数 */
    void *user;                /**< 用户上下文 */
    uint8_t tx_data[UPROTO_MAX_PAYLOAD + UPROTO_HEADER_SIZE + 2]; /**< 待发送数据缓冲 */
    uint32_t tx_len;           /**< 待发送数据长度 */
};

/**
 * @brief 消息处理入口
 * @details 将消息类型与处理函数关联的表项
 */
typedef struct {
    uint8_t type;                  /**< 消息类型 */
    uproto_msg_handler_fn handler; /**< 处理函数 */
    void *user;                    /**< 用户上下文 */
} uproto_handler_entry_t;

/**
 * @brief 端口操作接口
 * @details 硬件端口抽象层，提供读写和MTU查询功能
 */
typedef struct {
    uproto_port_write_fn write;   /**< 写函数指针 */
    uproto_port_flush_fn flush;   /**< 刷新函数指针 */
    uproto_port_get_mtu_fn get_mtu; /**< 获取MTU函数指针 */
    void *user;                   /**< 用户上下文 */
} uproto_port_ops_t;

/**
 * @brief 时间操作接口
 * @details 提供系统时间获取功能的抽象
 */
typedef struct {
    uproto_time_now_fn now_ms; /**< 获取毫秒时间戳函数指针 */
    void *user;                /**< 用户上下文 */
} uproto_time_ops_t;

/**
 * @brief uproto协议配置
 * @details 协议行为参数配置结构体
 */
typedef struct {
    uint32_t handshake_timeout_ms;     /**< 握手超时时间（毫秒） */
    uint32_t heartbeat_interval_ms;    /**< 心跳间隔（毫秒） */
    uint32_t default_timeout_ms;       /**< 默认事务超时（毫秒） */
    uint8_t default_retries;           /**< 默认重试次数 */
    bool enable_auto_handshake;        /**< 是否启用自动握手 */
    uproto_event_callback_fn event_cb; /**< 事件回调函数 */
    void *event_user;                  /**< 事件回调用户上下文 */
} uproto_config_t;

/**
 * @brief uproto协议主上下文
 * @details 包含协议状态机、事务管理、消息处理等完整状态
 */
struct uproto_context {
    uproto_state_t state;                /**< 协议状态 */
    uint16_t local_seq;                  /**< 本地序列号 */
    uint16_t remote_seq;                 /**< 远端序列号 */
    uint32_t last_rx_time;               /**< 最后接收时间 */
    uint32_t last_tx_time;               /**< 最后发送时间 */
    uint32_t handshake_start_time;       /**< 握手开始时间 */

    uproto_config_t config;              /**< 协议配置 */
    uproto_port_ops_t port;              /**< 端口操作接口 */
    uproto_time_ops_t time;              /**< 时间操作接口 */

    uproto_transaction_t transactions[UPROTO_MAX_TRANSACTIONS]; /**< 事务数组 */
    uint16_t next_txn_id;                /**< 下一个事务ID */

    uproto_handler_entry_t handlers[UPROTO_MAX_HANDLERS]; /**< 消息处理器表 */
    uint32_t handler_count;              /**< 处理器数量 */

    uint8_t rx_buffer[UPROTO_RX_BUFFER_SIZE]; /**< 接收缓冲区 */
    uint32_t rx_pos;                     /**< 接收缓冲区当前位置 */

    /** @brief 协议统计信息 */
    struct {
        uint32_t tx_frames;  /**< 发送帧数 */
        uint32_t rx_frames;  /**< 接收帧数 */
        uint32_t crc_errors; /**< CRC错误数 */
        uint32_t timeouts;   /**< 超时次数 */
        uint32_t retries;    /**< 重试次数 */
    } stats;
};

// ============ API ============
/**
  * @brief                  初始化uproto协议上下文
  * @param[in] ctx          协议上下文对象
  * @param[in] port_ops     端口操作接口
  * @param[in] time_ops     时间操作接口
  * @param[in] config       协议配置参数
  * @retval                 none
  */
void uproto_init(uproto_context_t *ctx,
                 const uproto_port_ops_t *port_ops,
                 const uproto_time_ops_t *time_ops,
                 const uproto_config_t *config);

/**
  * @brief                  启动握手流程
  * @param[in] ctx          协议上下文对象
  * @retval                 错误码（UPROTO_OK表示成功）
  */
uproto_error_t uproto_start_handshake(uproto_context_t *ctx);

/**
  * @brief                  检查协议是否已建立连接
  * @param[in] ctx          协议上下文对象
  * @retval                 true=已建立，false=未建立
  */
bool uproto_is_established(const uproto_context_t *ctx);

/**
  * @brief                  协议主循环处理（需周期性调用）
  * @param[in] ctx          协议上下文对象
  * @retval                 none
  */
void uproto_tick(uproto_context_t *ctx);

/**
  * @brief                  接收字节流处理
  * @param[in] ctx          协议上下文对象
  * @param[in] data         接收数据缓冲区
  * @param[in] len          数据长度（字节）
  * @retval                 none
  */
void uproto_on_rx_bytes(uproto_context_t *ctx, const uint8_t *data, uint32_t len);

/**
  * @brief                  发送请求消息
  * @param[in] ctx          协议上下文对象
  * @param[in] type         消息类型
  * @param[in] payload      载荷数据缓冲区（可为NULL）
  * @param[in] len          载荷长度
  * @param[in] timeout_ms   超时时间（毫秒）
  * @param[in] retries      重试次数
  * @param[in] callback     完成回调函数（可为NULL）
  * @param[in] user         用户上下文
  * @retval                 错误码（UPROTO_OK表示成功）
  */
uproto_error_t uproto_send_request(uproto_context_t *ctx,
                                   uint8_t type,
                                   const uint8_t *payload,
                                   uint32_t len,
                                   uint32_t timeout_ms,
                                   uint8_t retries,
                                   uproto_txn_callback_fn callback,
                                   void *user);

/**
  * @brief                  发送响应消息
  * @param[in] ctx          协议上下文对象
  * @param[in] txn_id       对应请求的事务ID
  * @param[in] type         消息类型
  * @param[in] payload      载荷数据缓冲区（可为NULL）
  * @param[in] len          载荷长度
  * @retval                 错误码（UPROTO_OK表示成功）
  */
uproto_error_t uproto_send_response(uproto_context_t *ctx,
                                    uint16_t txn_id,
                                    uint8_t type,
                                    const uint8_t *payload,
                                    uint32_t len);

/**
  * @brief                  发送通知消息（无响应）
  * @param[in] ctx          协议上下文对象
  * @param[in] type         消息类型
  * @param[in] payload      载荷数据缓冲区（可为NULL）
  * @param[in] len          载荷长度
  * @retval                 错误码（UPROTO_OK表示成功）
  */
uproto_error_t uproto_send_notify(uproto_context_t *ctx,
                                  uint8_t type,
                                  const uint8_t *payload,
                                  uint32_t len);

/**
  * @brief                  注册消息类型处理器
  * @param[in] ctx          协议上下文对象
  * @param[in] type         消息类型
  * @param[in] handler      处理函数
  * @param[in] user         用户上下文
  * @retval                 错误码（UPROTO_OK表示成功）
  */
uproto_error_t uproto_register_handler(uproto_context_t *ctx,
                                       uint8_t type,
                                       uproto_msg_handler_fn handler,
                                       void *user);

/**
  * @brief                  获取协议统计信息
  * @param[in] ctx          协议上下文对象
  * @param[out] tx_frames   发送帧数（可为NULL）
  * @param[out] rx_frames   接收帧数（可为NULL）
  * @param[out] crc_errors  CRC错误数（可为NULL）
  * @param[out] timeouts    超时次数（可为NULL）
  * @retval                 none
  */
void uproto_get_stats(const uproto_context_t *ctx,
                      uint32_t *tx_frames,
                      uint32_t *rx_frames,
                      uint32_t *crc_errors,
                      uint32_t *timeouts);

// Backward-safe: ensure user base msg id is available if earlier line was commented
#ifndef UPROTO_MSG_USER_BASE
/**
 * @brief 用户消息类型基址
 * @details 确保用户基础消息ID可用，防止早期版本被注释导致的问题
 */
#define UPROTO_MSG_USER_BASE    0x10
#endif

#ifdef __cplusplus
}
#endif

#endif // UPROTO_H
