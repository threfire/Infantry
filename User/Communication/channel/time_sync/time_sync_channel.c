#include "time_sync_channel.h"
#include "../../core/comm_utils.h"
#include <string.h>

/**
  * @brief                  获取当前时间
  * @param[in] ts           时间同步通道对象
  * @retval                 时间（微秒）
  */
static uint64_t now_us(const time_sync_channel_t *ts) {
    return (ts && ts->time.now_us) ? ts->time.now_us(ts->time.user) : 0;
}

/**
  * @brief                  时间同步历史记录
  * @param[in] ts           时间同步通道对象
  * @param[in] seq          时间同步请求序号
  * @param[in] t0           时间同步请求发送时间
  * @retval                 none
  */
static void hist_put(time_sync_channel_t *ts, uint16_t seq, uint64_t t0) {
    ts->hist[ts->hist_pos].seq = seq;
    ts->hist[ts->hist_pos].t0 = t0;
    ts->hist_pos = (uint8_t)((ts->hist_pos + 1) & 0x1F);
}

/**
  * @brief                  时间同步历史记录
  * @param[in] ts           时间同步通道对象
  * @param[in] seq          时间同步请求序号
  * @param[in] t0           时间同步请求发送时间
  * @retval                 none
  */
static int hist_get(const time_sync_channel_t *ts, uint16_t seq, uint64_t *t0) {
    for(int i = 0; i < 32; i++) {
        if(ts->hist[i].seq == seq) {
            *t0 = ts->hist[i].t0;
            return 1;
        }
    }
    return 0;
}

/**
  * @brief                  时间同步通道接收数据处理
  * @param[in] ch           通道对象
  * @param[in] payload      数据包负载
  * @param[in] len          数据包长度
  * @param[in] user         用户数据
  * @retval                 none
  */
static void on_rx(channel_t *ch, const uint8_t *payload, uint32_t len, void *user) {
    (void)ch;
    time_sync_channel_t *ts = (time_sync_channel_t *)user;
    if(!payload || len < 2 || !ts)
        return;
    uint16_t sid = comm_read_u16_le(payload);

    if(sid == TS_SID_REQ) {
        if(len < 2 + 2 + 8)
            return;
        uint16_t seq = comm_read_u16_le(payload + 2);
        /* t1 = local rx time; t2 = local tx time (near send) */
        uint64_t t1 = now_us(ts);
        uint64_t t2 = now_us(ts);
        uint8_t buf[2 + 2 + 8 + 8];
        comm_write_u16_le(&buf[0], TS_SID_RESP);
        comm_write_u16_le(&buf[2], seq);
        comm_write_u64_le(&buf[4], t1);
        comm_write_u64_le(&buf[12], t2);
        (void)ch_uproto_queue_notify(ts->bind, ts->ch_id, TS_SID_RESP, 0, buf, (uint16_t)sizeof(buf));
    } else if(sid == TS_SID_RESP) {
        if(len < 2 + 2 + 8 + 8)
            return;
        uint16_t seq = comm_read_u16_le(payload + 2);
        uint64_t t1 = comm_read_u64_le(payload + 4);
        uint64_t t2 = comm_read_u64_le(payload + 12);
        uint64_t t3 = now_us(ts);
        uint64_t t0 = 0;
        if(!hist_get(ts, seq, &t0))
            return;
        uint64_t rtt = (t3 - t0);
        if(t2 >= t1)
            rtt -= (t2 - t1);
        ts->rtt_us_last = (uint32_t)rtt;

        int64_t offset = ((int64_t)(t1 - t0) + (int64_t)(t2 - t3)) / 2;
        /* accept all if no max_rtt; else check threshold */
        if(ts->max_rtt_us == 0 || rtt <= ts->max_rtt_us) {
            if(!ts->offset_origin_valid) {
                ts->offset_origin_us = offset;
                ts->offset_origin_valid = 1;
            }
            int64_t rel = offset - ts->offset_origin_us;
            ts->offset_us = (ts->offset_us * 9 + offset) / 10; /* IIR */
            ts->offset_display_us = (ts->offset_display_us * 9 + rel) / 10;
            ts->last_device_time_us = t2;
            ts->last_host_time_us = t3;
            ts->mapping_valid = 1;
            ts->mapping_version++;
        }
    }
}

/**
  * @brief                  时间同步通道定时任务
  * @param[in] ch           通道对象
  * @param[in] user         用户数据
  * @retval                 none
  */
static void on_tick(channel_t *ch, void *user) {
    (void)ch;
    time_sync_channel_t *ts = (time_sync_channel_t *)user;
    if(!ts || !ts->initiator)
        return;
    uint64_t now = now_us(ts);
    if((now - ts->last_req_us) >= (uint64_t)ts->period_ms * 1000ULL) {
        ts->last_req_us = now;
        uint16_t seq = ts->seq++;
        uint8_t buf[2 + 2 + 8];
        comm_write_u16_le(&buf[0], TS_SID_REQ);
        comm_write_u16_le(&buf[2], seq);
        /* t0 placed at send time; use current now */
        comm_write_u64_le(&buf[4], now);
        hist_put(ts, seq, now);
        (void)ch_uproto_queue_notify(ts->bind, ts->ch_id, TS_SID_REQ, 0, buf, (uint16_t)sizeof(buf));
    }
}

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
                            uint8_t priority) {
    if(!ts || !bind || !mgr || !time_ops)
        return;
    memset(ts, 0, sizeof(*ts));
    ts->bind = bind;
    ts->mgr = mgr;
    ts->ch_id = ch_id;
    ts->time = *time_ops;
    ts->period_ms = (period_ms == 0) ? 1000u : period_ms;
    ts->initiator = initiator ? 1u : 0u;
    ts->max_rtt_us = 0; /* accept all by default */
    ts->last_req_us = 0;

    channel_cfg_t cfg = {.mode = CH_MODE_QA, .reliable = 0, .expect_reply = 0, .priority = priority, .period_ms = 0};
    channel_hooks_t hooks = {.on_rx = on_rx, .on_tick = on_tick, .on_cfg_change = NULL};
    channel_init(&ts->ch, ts->ch_id, &cfg, &hooks, ts);
    channel_bind_transport(&ts->ch, ts->bind, ts->bind->msg_type);
    (void)chmgr_register(ts->mgr, &ts->ch);
}

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
                               const ts_time_ops_t *time_ops) {
    if(!cfg)
        return;
    time_sync_channel_init(ts, bind, mgr, cfg->ch_id, time_ops, cfg->period_ms, cfg->initiator, cfg->priority);
    if(cfg->max_rtt_us) {
        time_sync_channel_set_max_rtt_us(ts, cfg->max_rtt_us);
    }
}

/**
  * @brief                  时间同步通道定时任务
  * @param[in] ts           时间同步通道对象
  * @retval                 none
  */
void time_sync_channel_tick(time_sync_channel_t *ts) {
    if(!ts)
        return;
    on_tick(NULL, ts);
}

/**
  * @brief                  获取当前时间戳（微秒）
  * @param[in] ts           时间同步通道对象
  * @retval                 时间戳（微秒）
  */
uint64_t time_sync_channel_now_us(const time_sync_channel_t *ts) {
    if(!ts)
        return 0;
    uint64_t n = now_us(ts);
    if(ts->mapping_valid)
        return (uint64_t)((int64_t)n + ts->offset_us);
    return n;
}

/**
  * @brief                  设置最大允许RTT（微秒）
  * @param[in] ts           时间同步通道对象
  * @param[in] max_rtt_us   最大允许RTT（微秒）
  * @retval                 none
  */
void time_sync_channel_set_max_rtt_us(time_sync_channel_t *ts, uint32_t max_rtt_us) {
    if(!ts)
        return;
    ts->max_rtt_us = max_rtt_us;
}
