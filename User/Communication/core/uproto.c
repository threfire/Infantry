/**
 * @file uproto.c
 * @brief 通用协议核心实现。
 * 
 * 提供帧编解码、事务管理、超时重试以及心跳/握手等核心逻辑。
 * 所有接口均搭配 @ref uproto.h 暴露的 API 使用。
 * 
 **/
#include "uproto.h"
#include "platform.h"
#include "CRC8_CRC16.h"

/* Map to project CRC implementation (DJI style) */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length) {
    return get_CRC16_check_sum((uint8_t*)data, length, 0xFFFF);
}

/* ============ 辅助函数 ============ */
/**
 * @brief 安全获取当前毫秒时间戳。
 */
static uint32_t uproto_now(const uproto_context_t *ctx) {
    if (ctx->time.now_ms) {
        return ctx->time.now_ms(ctx->time.user);
    }
    return 0;
}

/**
 * @brief 触发事件回调。
 */
static void uproto_fire_event(uproto_context_t *ctx, const char *event) {
    if (ctx->config.event_cb) {
        ctx->config.event_cb(ctx, event, ctx->config.event_user);
    }
}

/**
 * @brief 以小端字节序读取 16 位整数。
 */
static uint16_t read_u16_le(const uint8_t *p) {
    return ((uint16_t)p[0]) | ((uint16_t)p[1] << 8);
}

/**
 * @brief 写入 16 位小端整数。
 */
static void write_u16_le(uint8_t *p, uint16_t val) {
    p[0] = val & 0xFF;
    p[1] = (val >> 8) & 0xFF;
}

/* ============ 帧编码 ============ */
/**
 * @brief 将数据打包为协议帧。
 *
 * @param out         输出缓冲区。
 * @param flags       帧标志。
 * @param type        消息类型。
 * @param txn_id      事务 ID。
 * @param seq         序列号。
 * @param ack         确认号。
 * @param payload     数据载荷。
 * @param payload_len 载荷长度。
 * @return 编码后的总长度。
 */
static uint32_t uproto_encode_frame(uint8_t *out,
                                  uint8_t flags,
                                  uint8_t type,
                                  uint16_t txn_id,
                                  uint16_t seq,
                                  uint16_t ack,
                                  const uint8_t *payload,
                                  uint32_t payload_len) {
    uint32_t pos = 0;
    
    // Magic
    out[pos++] = UPROTO_MAGIC_0;
    out[pos++] = UPROTO_MAGIC_1;
    
    // Header
    out[pos++] = UPROTO_VERSION;
    out[pos++] = flags;
    out[pos++] = type;
    out[pos++] = 0;  // reserved
    
    write_u16_le(&out[pos], txn_id); pos += 2;
    write_u16_le(&out[pos], seq);    pos += 2;
    write_u16_le(&out[pos], ack);    pos += 2;
    write_u16_le(&out[pos], payload_len); pos += 2;
    
    // Header CRC (从 version 到 len)
    uint16_t hdr_crc = crc16_ccitt(&out[2], 12);
    write_u16_le(&out[pos], hdr_crc); pos += 2;
    
    // Payload
    if (payload && payload_len > 0) {
        platform_memcpy(&out[pos], payload, payload_len);
        pos += payload_len;
    }
    
    // Frame CRC (从 version 到 payload)
    uint16_t frame_crc = crc16_ccitt(&out[2], pos - 2);
    write_u16_le(&out[pos], frame_crc); pos += 2;
    
    return pos;
}

/* ============ 帧解码 ============ */
/**
 * @brief 校验并解析输入字节流。
 */
static uproto_error_t uproto_decode_frame(const uint8_t *data,
                                          uint32_t len,
                                          uproto_frame_t *frame) {
    if (len < UPROTO_HEADER_SIZE + 2) {
        return UPROTO_ERR_LENGTH;
    }
    
    uint32_t pos = 0;
    
    // 检查 Magic
    if (data[pos++] != UPROTO_MAGIC_0 || data[pos++] != UPROTO_MAGIC_1) {
        return UPROTO_ERR_LENGTH;
    }
    
    // 解析头部
    frame->version = data[pos++];
    frame->flags = data[pos++];
    frame->type = data[pos++];
    frame->reserved = data[pos++];
    
    frame->txn_id = read_u16_le(&data[pos]); pos += 2;
    frame->seq = read_u16_le(&data[pos]); pos += 2;
    frame->ack = read_u16_le(&data[pos]); pos += 2;
    frame->len = read_u16_le(&data[pos]); pos += 2;
    
    uint16_t hdr_crc_rx = read_u16_le(&data[pos]); pos += 2;
    
    // 验证头部 CRC
    uint16_t hdr_crc_calc = crc16_ccitt(&data[2], 12);
    if (hdr_crc_rx != hdr_crc_calc) {
        return UPROTO_ERR_CRC;
    }
    
    // 检查长度
    if (frame->len > UPROTO_MAX_PAYLOAD) {
        return UPROTO_ERR_LENGTH;
    }
    
    if (len < UPROTO_HEADER_SIZE + frame->len + 2) {
        return UPROTO_ERR_LENGTH;
    }
    
    // 复制 payload
    if (frame->len > 0) {
        platform_memcpy(frame->payload, &data[pos], frame->len);
        pos += frame->len;
    }
    
    // 验证帧 CRC
    uint16_t frame_crc_rx = read_u16_le(&data[pos]);
    uint16_t frame_crc_calc = crc16_ccitt(&data[2], UPROTO_HEADER_SIZE - 2 + frame->len);
    if (frame_crc_rx != frame_crc_calc) {
        return UPROTO_ERR_CRC;
    }
    
    return UPROTO_OK;
}

/* ============ 事务管理 ============ */
/**
 * @brief 获取一个空闲事务槽位。
 */
static uproto_transaction_t* uproto_alloc_transaction(uproto_context_t *ctx) {
    for (uint32_t i = 0; i < UPROTO_MAX_TRANSACTIONS; i++) {
        if (ctx->transactions[i].state == TXN_STATE_FREE) {
            return &ctx->transactions[i];
        }
    }
    return NULL;
}

/**
 * @brief 按事务 ID 查找事务。
 */
static uproto_transaction_t* uproto_find_transaction(uproto_context_t *ctx, uint16_t txn_id) {
    for (uint32_t i = 0; i < UPROTO_MAX_TRANSACTIONS; i++) {
        if (ctx->transactions[i].state != TXN_STATE_FREE &&
            ctx->transactions[i].txn_id == txn_id) {
            return &ctx->transactions[i];
        }
    }
    return NULL;
}

/**
 * @brief 完成事务并释放资源。
 */
static void uproto_complete_transaction(uproto_context_t *ctx,
                                        uproto_transaction_t *txn,
                                        uproto_error_t err,
                                        const uint8_t *rsp_data,
                                        uint32_t rsp_len) {
    if (txn->callback) {
        txn->callback(txn->user, err, rsp_data, rsp_len);
    }
    txn->state = TXN_STATE_FREE;
}

/* ============ 发送帧 ============ */
static uproto_error_t uproto_send_frame(uproto_context_t *ctx,
                                       uint8_t flags,
                                       uint8_t type,
                                       uint16_t txn_id,
                                       const uint8_t *payload,
                                       uint32_t payload_len) {
    if (!ctx->port.write) {
        return UPROTO_ERR_INVALID_PARAM;
    }
    
    uint8_t buffer[UPROTO_MAX_PAYLOAD + UPROTO_HEADER_SIZE + 2];
    uint32_t frame_len = uproto_encode_frame(buffer, flags, type, txn_id,
                                          ctx->local_seq++, 0,
                                          payload, payload_len);
    
    uint32_t sent = ctx->port.write(ctx->port.user, buffer, frame_len);
    if (sent == frame_len) {
        if (ctx->port.flush) {
            ctx->port.flush(ctx->port.user);
        }
        ctx->stats.tx_frames++;
        ctx->last_tx_time = uproto_now(ctx);
        return UPROTO_OK;
    }
    
    return UPROTO_ERR_TX_FULL;
}

/* ============ 处理接收帧 ============ */
/**
 * @brief 解析并路由已解码的帧。
 */
static void uproto_handle_frame(uproto_context_t *ctx, const uproto_frame_t *frame) {
    ctx->stats.rx_frames++;
    ctx->last_rx_time = uproto_now(ctx);
    ctx->remote_seq = frame->seq;
    
    // 处理系统消息
    if (frame->type == UPROTO_MSG_HELLO) {
        // 收到握手请求，发送响应
        uproto_send_frame(ctx, UPROTO_FLAG_RESPONSE, UPROTO_MSG_HELLO_ACK,
                         frame->txn_id, NULL, 0);
        if (ctx->state == UPROTO_STATE_IDLE || ctx->state == UPROTO_STATE_HANDSHAKING) {
            ctx->state = UPROTO_STATE_ESTABLISHED;
            uproto_fire_event(ctx, "handshake_completed");
        }
        return;
    }
    
    if (frame->type == UPROTO_MSG_HELLO_ACK) {
        // 收到握手响应
        if (ctx->state == UPROTO_STATE_HANDSHAKING) {
            ctx->state = UPROTO_STATE_ESTABLISHED;
            uproto_fire_event(ctx, "handshake_completed");
        }
        // 完成握手事务
        uproto_transaction_t *txn = uproto_find_transaction(ctx, frame->txn_id);
        if (txn) {
            uproto_complete_transaction(ctx, txn, UPROTO_OK, frame->payload, frame->len);
        }
        return;
    }
    
    if (frame->type == UPROTO_MSG_PING) {
        // 心跳请求，发送响应
        uproto_send_frame(ctx, UPROTO_FLAG_RESPONSE, UPROTO_MSG_PONG,
                         frame->txn_id, NULL, 0);
        return;
    }
    
    if (frame->type == UPROTO_MSG_PONG) {
        // 心跳响应
        uproto_transaction_t *txn = uproto_find_transaction(ctx, frame->txn_id);
        if (txn) {
            uproto_complete_transaction(ctx, txn, UPROTO_OK, NULL, 0);
        }
        return;
    }
    
    // 处理响应
    if (frame->flags & UPROTO_FLAG_RESPONSE) {
        uproto_transaction_t *txn = uproto_find_transaction(ctx, frame->txn_id);
        if (txn) {
            uproto_complete_transaction(ctx, txn, UPROTO_OK, frame->payload, frame->len);
        }
        return;
    }

    // 处理通知
    if (!(frame->flags & UPROTO_FLAG_REQUEST)) {
        for (uint32_t i = 0; i < ctx->handler_count; i++) {
            if (ctx->handlers[i].type == frame->type) {
                ctx->handlers[i].handler(ctx, frame->txn_id,
                                        frame->payload, frame->len,
                                        ctx->handlers[i].user);
                return;
            }
        }
        return;
    }
    
    // 处理请求
    if (frame->flags & UPROTO_FLAG_REQUEST) {
        // 查找处理器
        for (uint32_t i = 0; i < ctx->handler_count; i++) {
            if (ctx->handlers[i].type == frame->type) {
                ctx->handlers[i].handler(ctx, frame->txn_id,
                                        frame->payload, frame->len,
                                        ctx->handlers[i].user);
                return;
            }
        }
        // 未找到处理器，发送错误响应
        uproto_send_frame(ctx, UPROTO_FLAG_RESPONSE, UPROTO_MSG_ERROR,
                         frame->txn_id, NULL, 0);
        return;
    }
}

/* ============ 流式解码器 ============ */
/**
 * @brief 从接收缓冲区提取完整帧。
 */
static void uproto_process_rx_buffer(uproto_context_t *ctx) {
    while (ctx->rx_pos >= UPROTO_HEADER_SIZE + 2) {
        // 查找帧头
        uint32_t frame_start = 0;
        bool found_magic = false;
        
        for (uint32_t i = 0; i < ctx->rx_pos - 1; i++) {
            if (ctx->rx_buffer[i] == UPROTO_MAGIC_0 &&
                ctx->rx_buffer[i + 1] == UPROTO_MAGIC_1) {
                frame_start = i;
                found_magic = true;
                break;
            }
        }
        
        if (!found_magic) {
            // 未找到帧头，清空缓冲区
            ctx->rx_pos = 0;
            break;
        }
        
        // 丢弃帧头前的数据
        if (frame_start > 0) {
            platform_memmove(ctx->rx_buffer, &ctx->rx_buffer[frame_start],
                   ctx->rx_pos - frame_start);
            ctx->rx_pos -= frame_start;
        }
        
        // 检查是否有完整帧
        if (ctx->rx_pos < UPROTO_HEADER_SIZE) {
            break;  // 等待更多数据
        }
        
        // 读取载荷长度
        uint16_t payload_len = read_u16_le(&ctx->rx_buffer[12]);
        uint32_t total_len = UPROTO_HEADER_SIZE + payload_len + 2;
        
        if (payload_len > UPROTO_MAX_PAYLOAD) {
            // 长度异常，丢弃此帧头，继续查找
            platform_memmove(ctx->rx_buffer, &ctx->rx_buffer[2], ctx->rx_pos - 2);
            ctx->rx_pos -= 2;
            continue;
        }
        
        if (ctx->rx_pos < total_len) {
            break;  // 等待更多数据
        }
        
        // 解码帧
        uproto_frame_t frame;
        uproto_error_t err = uproto_decode_frame(ctx->rx_buffer, total_len, &frame);
        
        if (err == UPROTO_OK) {
            // 处理帧
            uproto_handle_frame(ctx, &frame);
        } else if (err == UPROTO_ERR_CRC) {
            ctx->stats.crc_errors++;
            uproto_fire_event(ctx, "crc_error");
        }
        
        // 移除已处理的帧
        platform_memmove(ctx->rx_buffer, &ctx->rx_buffer[total_len],
               ctx->rx_pos - total_len);
        ctx->rx_pos -= total_len;
    }
}

/* ============ 公共 API ============ */

/**
 * @brief 初始化协议上下文。
 *
 * 拷贝端口/时间配置，并重置内部状态。
 */
void uproto_init(uproto_context_t *ctx,
                 const uproto_port_ops_t *port_ops,
                 const uproto_time_ops_t *time_ops,
                 const uproto_config_t *config) {
    platform_memset(ctx, 0, sizeof(*ctx));
    
    ctx->state = UPROTO_STATE_IDLE;
    ctx->port = *port_ops;
    ctx->time = *time_ops;
    
    if (config) {
        ctx->config = *config;
    } else {
        // 默认配置
        ctx->config.handshake_timeout_ms = 0;  // 禁用握手
        ctx->config.heartbeat_interval_ms = 0;  // 禁用心跳
        ctx->config.default_timeout_ms = 3000;
        ctx->config.default_retries = 3;
        ctx->config.enable_auto_handshake = false;
    }
    
    ctx->next_txn_id = 1;
}

/**
 * @brief 主动触发握手流程。
 *
 * 仅在空闲状态可调用。
 */
uproto_error_t uproto_start_handshake(uproto_context_t *ctx) {
    if (ctx->state != UPROTO_STATE_IDLE) {
        return UPROTO_ERR_BUSY;
    }
    
    ctx->state = UPROTO_STATE_HANDSHAKING;
    ctx->handshake_start_time = uproto_now(ctx);
    
    return uproto_send_request(ctx, UPROTO_MSG_HELLO, NULL, 0,
                              ctx->config.handshake_timeout_ms,
                              ctx->config.default_retries,
                              NULL, NULL);
}

/**
 * @brief 判断当前是否处于已建立状态。
 *
 * @param ctx 协议上下文。
 * @return 若已握手成功返回 true，否则返回 false。
 */
bool uproto_is_established(const uproto_context_t *ctx) {
    return ctx->state == UPROTO_STATE_ESTABLISHED;
}

/**
 * @brief 协议周期逻辑入口（需定期调用）。
 *
 * @param ctx 协议上下文。
 *
 * 负责检测超时、触发重试以及周期性心跳。
 */
void uproto_tick(uproto_context_t *ctx) {
    uint32_t now = uproto_now(ctx);
    
    // 检查握手超时
    if (ctx->state == UPROTO_STATE_HANDSHAKING &&
        ctx->config.handshake_timeout_ms > 0) {
        if (now - ctx->handshake_start_time > ctx->config.handshake_timeout_ms) {
            ctx->state = UPROTO_STATE_IDLE;
            uproto_fire_event(ctx, "handshake_timeout");
        }
    }
    
    // 处理事务超时和重传
    for (uint32_t i = 0; i < UPROTO_MAX_TRANSACTIONS; i++) {
        uproto_transaction_t *txn = &ctx->transactions[i];
        
        if (txn->state != TXN_STATE_WAITING) {
            continue;
        }
        
        uint32_t elapsed = now - txn->last_tx_time;
        
        if (elapsed >= txn->timeout_ms) {
            // 超时
            if (txn->retry_count < txn->max_retries) {
                // 重传
                txn->retry_count++;
                txn->last_tx_time = now;
                ctx->port.write(ctx->port.user, txn->tx_data, txn->tx_len);
                if (ctx->port.flush) {
                    ctx->port.flush(ctx->port.user);
                }
                ctx->stats.retries++;
                uproto_fire_event(ctx, "retry");
            } else {
                // 超时失败
                ctx->stats.timeouts++;
                uproto_fire_event(ctx, "timeout");
                uproto_complete_transaction(ctx, txn, UPROTO_ERR_TIMEOUT, NULL, 0);
            }
        }
    }
    
    // 心跳发送
    if (ctx->config.heartbeat_interval_ms > 0 &&
        ctx->state == UPROTO_STATE_ESTABLISHED) {
        if (now - ctx->last_tx_time >= ctx->config.heartbeat_interval_ms) {
            uproto_send_request(ctx, UPROTO_MSG_PING, NULL, 0,
                              ctx->config.default_timeout_ms,
                              0, NULL, NULL);
        }
    }
}

/**
 * @brief 注入接收数据并尝试解析完整帧。
 *
 * @param ctx  协议上下文。
 * @param data 新接收的字节流。
 * @param len  字节数量。
 */
void uproto_on_rx_bytes(uproto_context_t *ctx, const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (ctx->rx_pos < UPROTO_RX_BUFFER_SIZE) {
            ctx->rx_buffer[ctx->rx_pos++] = data[i];
        } else {
            // 缓冲区溢出，清空重新开始
            ctx->rx_pos = 0;
        }
    }
    
    uproto_process_rx_buffer(ctx);
}

/**
 * @brief 发送带期望响应的请求。
 *
 * @param ctx        协议上下文。
 * @param type       消息类型。
 * @param payload    请求数据指针，可为 NULL。
 * @param len        请求数据长度。
 * @param timeout_ms 超时时间，0 时使用默认值。
 * @param retries    最大重试次数。
 * @param callback   事务完成回调，可为 NULL。
 * @param user       回调用户数据。
 *
 * @return 发送结果。
 */
uproto_error_t uproto_send_request(uproto_context_t *ctx,
                                   uint8_t type,
                                   const uint8_t *payload,
                                   uint32_t len,
                                   uint32_t timeout_ms,
                                   uint8_t retries,
                                   uproto_txn_callback_fn callback,
                                   void *user) {
    if (len > UPROTO_MAX_PAYLOAD) {
        return UPROTO_ERR_LENGTH;
    }
    
    // 分配事务
    uproto_transaction_t *txn = uproto_alloc_transaction(ctx);
    if (!txn) {
        return UPROTO_ERR_NO_MEMORY;
    }
    
    uint16_t txn_id = ctx->next_txn_id++;
    
    // 编码帧
    txn->tx_len = uproto_encode_frame(txn->tx_data,
                                     UPROTO_FLAG_REQUEST,
                                     type, txn_id,
                                     ctx->local_seq++, 0,
                                     payload, len);
    
    // 发送
    uint32_t sent = ctx->port.write(ctx->port.user, txn->tx_data, txn->tx_len);
    if (sent != txn->tx_len) {
        txn->state = TXN_STATE_FREE;
        return UPROTO_ERR_TX_FULL;
    }
    
    if (ctx->port.flush) {
        ctx->port.flush(ctx->port.user);
    }
    
    // 设置事务状态
    txn->state = TXN_STATE_WAITING;
    txn->txn_id = txn_id;
    txn->type = type;
    txn->retry_count = 0;
    txn->max_retries = retries;
    txn->timeout_ms = timeout_ms > 0 ? timeout_ms : ctx->config.default_timeout_ms;
    txn->start_time = uproto_now(ctx);
    txn->last_tx_time = txn->start_time;
    txn->callback = callback;
    txn->user = user;
    
    ctx->stats.tx_frames++;
    ctx->last_tx_time = txn->start_time;
    
    return UPROTO_OK;
}

/**
 * @brief 回复请求方发送的响应帧。
 *
 * @param ctx     协议上下文。
 * @param txn_id  对应的事务 ID。
 * @param type    响应类型。
 * @param payload 响应载荷。
 * @param len     载荷长度。
 *
 * @return 发送结果。
 */
uproto_error_t uproto_send_response(uproto_context_t *ctx,
                                    uint16_t txn_id,
                                    uint8_t type,
                                    const uint8_t *payload,
                                    uint32_t len) {
    if (len > UPROTO_MAX_PAYLOAD) {
        return UPROTO_ERR_LENGTH;
    }
    
    return uproto_send_frame(ctx, UPROTO_FLAG_RESPONSE, type, txn_id, payload, len);
}

/**
 * @brief 发送无需响应的通知消息。
 *
 * @param ctx     协议上下文。
 * @param type    消息类型。
 * @param payload 消息载荷，可为 NULL。
 * @param len     载荷长度。
 *
 * @return 发送结果。
 */
uproto_error_t uproto_send_notify(uproto_context_t *ctx,
                                  uint8_t type,
                                  const uint8_t *payload,
                                  uint32_t len) {
    if (len > UPROTO_MAX_PAYLOAD) {
        return UPROTO_ERR_LENGTH;
    }
    
    return uproto_send_frame(ctx, 0, type, 0, payload, len);
}

/**
 * @brief 注册消息处理器。
 *
 * @param ctx     协议上下文。
 * @param type    绑定的消息类型。
 * @param handler 回调函数。
 * @param user    回调上下文。
 *
 * @return 注册结果。
 */
uproto_error_t uproto_register_handler(uproto_context_t *ctx,
                                       uint8_t type,
                                       uproto_msg_handler_fn handler,
                                       void *user) {
    if (ctx->handler_count >= UPROTO_MAX_HANDLERS) {
        return UPROTO_ERR_NO_MEMORY;
    }
    
    ctx->handlers[ctx->handler_count].type = type;
    ctx->handlers[ctx->handler_count].handler = handler;
    ctx->handlers[ctx->handler_count].user = user;
    ctx->handler_count++;
    
    return UPROTO_OK;
}

/**
 * @brief 获取统计计数。
 *
 * @param ctx        协议上下文。
 * @param tx_frames  若非 NULL，返回发送帧数。
 * @param rx_frames  若非 NULL，返回接收帧数。
 * @param crc_errors 若非 NULL，返回 CRC 错误数。
 * @param timeouts   若非 NULL，返回超时次数。
 */
void uproto_get_stats(const uproto_context_t *ctx,
                      uint32_t *tx_frames,
                      uint32_t *rx_frames,
                      uint32_t *crc_errors,
                      uint32_t *timeouts) {
    if (tx_frames) *tx_frames = ctx->stats.tx_frames;
    if (rx_frames) *rx_frames = ctx->stats.rx_frames;
    if (crc_errors) *crc_errors = ctx->stats.crc_errors;
    if (timeouts) *timeouts = ctx->stats.timeouts;
}












