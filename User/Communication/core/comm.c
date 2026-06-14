#include "comm.h"
#include "uproto.h"
#include <string.h>
#include "CRC8_CRC16.h"
#include "config.h"

static void channel_default_hooks_on_cfg_change(channel_t *ch, const channel_cfg_t *cfg, void *user) {
    (void)ch;
    (void)cfg;
    (void)user;
}

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
                  void *user) {
    if(!ch)
        return;
    memset(ch, 0, sizeof(*ch));
    ch->id = id;
    if(cfg)
        ch->cfg = *cfg;
    if(hooks)
        ch->hooks = *hooks;
    ch->user = user;
    if(!ch->hooks.on_cfg_change) {
        ch->hooks.on_cfg_change = channel_default_hooks_on_cfg_change;
    }
}

/**
  * @brief          通道绑定传输层
  * @details        将通道对象绑定到传输层，并指定消息类型
  * @param[in]      ch：通道对象
  * @param[in]      transport_ctx：传输层上下文指针
  * @param[in]      msg_type：传输层消息类型
  * @retval         none
  */
void channel_bind_transport(channel_t *ch, void *transport_ctx, uint8_t msg_type) {
    if(!ch)
        return;
    ch->transport = transport_ctx;
    ch->msg_type = msg_type;
}

/**
  * @brief          通道配置
  * @details        设置通道配置参数
  * @param[in]      ch：通道对象
  * @param[in]      cfg：通道配置参数
  * @retval         none
  */
void channel_set_cfg(channel_t *ch, const channel_cfg_t *cfg) {
    if(!ch || !cfg)
        return;
    ch->cfg = *cfg;
    if(ch->hooks.on_cfg_change)
        ch->hooks.on_cfg_change(ch, &ch->cfg, ch->user);
}

/**
  * @brief          通道就绪状态
  * @details        判断通道是否就绪
  * @param[in]      ch：通道对象
  * @retval         true：通道就绪
  * @retval         false：通道未就绪
  */
bool channel_is_ready(const channel_t *ch) {
    return ch && ch->handshake_done;
}

/**
  * @brief          通道握手完成
  * @details        设置通道握手完成标志
  * @param[in]      ch：通道对象
  * @retval         none
  * @note           握手是特定于通道的；调用者实现实际的字节
  */
void channel_mark_handshake_done(channel_t *ch) {
    if(!ch)
        return;
    ch->handshake_done = 1;
}

/**
  * @brief          通道管理器实例
  * @details        管理所有通道实例，提供注册、查找和调度功能
  * @param[in]      ch：通道对象
  * @retval         none
  */
void chmgr_init(channel_manager_t *m) {
    if (!m) return;
    memset(m, 0, sizeof(*m));
}

/**
  * @brief          注册通道实例
  * @details        将通道实例注册到管理器，并返回成功与否
  * @param[in]      m：通道管理器实例
  * @param[in]      ch：通道对象
  * @retval         true：注册成功
  * @retval         false：注册失败
  */
bool chmgr_register(channel_manager_t *m, channel_t *ch) {
    if (!m || !ch) return false;
    if (m->count >= CH_MAX) return false;
    m->slots[m->count++] = ch;
    return true;
}

/**
  * @brief          查找通道实例
  * @details        根据逻辑通道 ID 查找通道实例
  * @param[in]      m：通道管理器实例
  * @param[in]      id：逻辑通道 ID
  * @retval         通道对象指针
  */
channel_t* chmgr_find(channel_manager_t *m, uint8_t id) {
    if (!m) return NULL;
    for (uint8_t i = 0; i < m->count; ++i) {
        if (m->slots[i] && m->slots[i]->id == id) return m->slots[i];
    }
    return NULL;
}

/**
  * @brief          调度通道接收数据
  * @details        根据通道 ID 调度通道接收数据
  * @param[in]      m：通道管理器实例
  * @param[in]      id：逻辑通道 ID
  * @param[in]      payload：接收数据指针
  * @param[in]      len：接收数据长度
  * @retval         none
  */
void chmgr_dispatch_rx(channel_manager_t *m, uint8_t id, const uint8_t *payload, uint32_t len) {
    channel_t *ch = chmgr_find(m, id);
    if (!ch || !ch->hooks.on_rx) return;
    ch->hooks.on_rx(ch, payload, len, ch->user);
}

/**
  * @brief          通道调度
  * @details        调度所有通道，处理通道状态、定时任务和数据发送
  * @param[in]      m：通道管理器实例
  * @retval         none
  */
void chmgr_tick(channel_manager_t *m) {
    if (!m) return;
    for (uint8_t i = 0; i < m->count; ++i) {
        channel_t *ch = m->slots[i];
        if (ch && ch->hooks.on_tick) ch->hooks.on_tick(ch, ch->user);
    }
}

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
                    const uint8_t *payload) {
    if (!out || !hdr) return 0;
    if (hdr->sof != MUX_SOF || hdr->ver != MUX_VER) return 0;
    uint32_t need = sizeof(mux_hdr_t) + hdr->len + 2u;
    if (out_cap < need) return 0;
    memcpy(out, hdr, sizeof(mux_hdr_t));
    if (payload && hdr->len) memcpy(out + sizeof(mux_hdr_t), payload, hdr->len);
    uint16_t crc = get_CRC16_check_sum(out + 1, (uint32_t)(sizeof(mux_hdr_t) - 1 + hdr->len), 0xFFFF);
    /* append crc little-endian */
    out[sizeof(mux_hdr_t) + hdr->len + 0] = (uint8_t)(crc & 0xFF);
    out[sizeof(mux_hdr_t) + hdr->len + 1] = (uint8_t)(crc >> 8);
    return need;
}

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
                const uint8_t **out_payload) {
    if (!data || len < sizeof(mux_hdr_t) + 2u || !out_hdr) return false;
    mux_hdr_t hdr;
    memcpy(&hdr, data, sizeof(hdr));
    if (hdr.sof != MUX_SOF || hdr.ver != MUX_VER) return false;
    if (len < sizeof(mux_hdr_t) + hdr.len + 2u) return false;
    uint16_t crc_rx = (uint16_t)data[sizeof(mux_hdr_t) + hdr.len + 0]
                    | (uint16_t)data[sizeof(mux_hdr_t) + hdr.len + 1] << 8;
    uint16_t crc_calc = get_CRC16_check_sum((uint8_t*)(data + 1), (uint32_t)(sizeof(mux_hdr_t) - 1 + hdr.len), 0xFFFF);
    if (crc_rx != crc_calc) return false;
    *out_hdr = hdr;
    if (out_payload) *out_payload = data + sizeof(mux_hdr_t);
    return true;
}




static void on_mux_rx(uproto_context_t *ctx,
                      uint16_t txn_id,
                      const uint8_t *data,
                      uint32_t len,
                      void *user) {
    (void)ctx; (void)txn_id;
    ch_uproto_bind_t *b = (ch_uproto_bind_t*)user;
    if (!b || !b->mgr) return;
    mux_hdr_t hdr; const uint8_t *pl = NULL;
    if (!mux_decode(data, len, &hdr, &pl)) return;
    chmgr_dispatch_rx(b->mgr, hdr.channel, pl, hdr.len);
}

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
                    channel_manager_t *mgr) {
    if (!b) return;
    memset(b, 0, sizeof(*b));
    b->ctx = ctx;
    b->msg_type = msg_type;
    b->mgr = mgr;
}

/**
  * @brief          UPROTO 注册接收
  * @details        注册通道对象接收数据处理函数
  * @param[in]      b：通道对象
  * @retval         none
  */
void ch_uproto_register_rx(ch_uproto_bind_t *b) {
    if (!b || !b->ctx) return;
    uproto_register_handler(b->ctx, b->msg_type, on_mux_rx, b);
}

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
                           uint16_t len) {
    if (!b || !b->ctx) return false;
    mux_hdr_t hdr = {0};
    hdr.sof = MUX_SOF;
    hdr.ver = MUX_VER;
    hdr.channel = ch_id;
    hdr.flags = flags;
    hdr.len = len;
    hdr.sid = sid;
    hdr.seq = ++b->seq_counters[ch_id % CH_MAX];
    uint8_t buf[COMM_MUX_TX_BUFFER_SIZE];
    uint32_t wrote = mux_encode(buf, sizeof(buf), &hdr, payload);
    if (wrote == 0) return false;
    return uproto_send_notify(b->ctx, b->msg_type, buf, wrote) == UPROTO_OK;
}

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
                            uint16_t len) {
    if (!b || !b->mgr) return false;
    /* map ch_id to index slot (0..CH_MAX-1) */
    for (uint8_t i = 0; i < CH_MAX; ++i) {
        channel_t *ch = b->mgr->slots[i];
        if (ch && ch->id == ch_id) {
            if (len > sizeof(b->pending[i].payload)) return false; /* too big for queue */
            /* keep-latest semantics; preserve age if already pending */
            uint16_t prev_age = b->pending[i].used ? b->pending[i].age : 0;
            b->pending[i].used = 1;
            b->pending[i].ch_id = ch_id;
            b->pending[i].sid = sid;
            b->pending[i].flags = flags;
            b->pending[i].len = len;
            if (payload && len) memcpy(b->pending[i].payload, payload, len);
            b->pending[i].age = prev_age;
            return true;
        }
    }
    return false;
}

/**
  * @brief          UPROTO 轮询
  * @details        轮询 UPROTO 上下文，处理接收数据
  * @param[in]      b：通道对象
  * @retval         none
  */
static int find_next_ready_index(ch_uproto_bind_t *b) {
    if (!b || !b->mgr) return -1;
    /* bump age for all pending items */
    for (uint8_t i = 0; i < b->mgr->count; ++i) {
        if (b->pending[i].used && b->pending[i].age < 0xFFFF) {
            b->pending[i].age++;
        }
    }
    int best = -1;
    uint8_t best_prio = 0;
    /* first pass: find highest priority */
    for (uint8_t i = 0; i < b->mgr->count; ++i) {
        if (!b->pending[i].used) continue;
        channel_t *ch = b->mgr->slots[i];
        if (!ch) continue;
        uint8_t pr = ch->cfg.priority;
        if (best == -1 || pr > best_prio) {
            best = i; best_prio = pr;
        }
    }
    if (best == -1) return -1;
    /* starvation guard: if any lower priority has aged long enough, serve it */
    const uint16_t FAIR_AGE = (uint16_t)CH_ARB_FAIR_AGE; /* deliver at least 1 per ~CH_ARB_FAIR_AGE arbiter ticks */
    int aged_idx = -1; uint16_t max_age = 0;
    for (uint8_t i = 0; i < b->mgr->count; ++i) {
        if (!b->pending[i].used) continue;
        channel_t *ch = b->mgr->slots[i]; if (!ch) continue;
        if (ch->cfg.priority < best_prio && b->pending[i].age >= FAIR_AGE) {
            if (b->pending[i].age > max_age) { max_age = b->pending[i].age; aged_idx = i; }
        }
    }
    if (aged_idx >= 0) return aged_idx;
    /* tie-break among same priority with simple rr over indices */
    uint8_t prio = best_prio;
    uint8_t start = b->rr_cursor % b->mgr->count;
    for (uint8_t off = 0; off < b->mgr->count; ++off) {
        uint8_t idx = (uint8_t)((start + off) % b->mgr->count);
        if (!b->pending[idx].used) continue;
        channel_t *ch = b->mgr->slots[idx];
        if (ch && ch->cfg.priority == prio) {
            b->rr_cursor = (uint8_t)(idx + 1);
            return idx;
        }
    }
    return best;
}

/**
  * @brief          UPROTO 仲裁器定时处理
  * @details        处理通道对象中的仲裁器
  * @param[in]      b：通道对象
  * @retval         none
  */
void ch_uproto_arbiter_tick(ch_uproto_bind_t *b) {
    if (!b || !b->ctx || !b->mgr) return;
    int idx = find_next_ready_index(b);
    if (idx < 0) return;
    /* build mux and send */
    uint8_t ch_id = b->pending[idx].ch_id;
    uint16_t sid = b->pending[idx].sid;
    uint8_t flags = b->pending[idx].flags;
    uint16_t len = b->pending[idx].len;
    uint8_t *pl = b->pending[idx].payload;

    mux_hdr_t hdr = {0};
    hdr.sof = MUX_SOF; hdr.ver = MUX_VER; hdr.channel = ch_id; hdr.flags = flags;
    hdr.len = len; hdr.sid = sid; hdr.seq = ++b->seq_counters[ch_id % CH_MAX];
    uint8_t buf[COMM_MUX_TX_BUFFER_SIZE];
    uint32_t wrote = mux_encode(buf, sizeof(buf), &hdr, pl);
    if (wrote == 0) { b->pending[idx].used = 0; b->pending[idx].age = 0; return; }
    if (uproto_send_notify(b->ctx, b->msg_type, buf, wrote) == UPROTO_OK) {
        b->pending[idx].used = 0; b->pending[idx].age = 0; /* sent */
    }
}


