#include "camera_channel.h"
#include "camera_config.h"
#include "../../core/comm_utils.h"
#include "../../core/platform.h"


/**
  * @brief                 清空摄像头事件FIFO
  * @param[in] cam         摄像头通道实例
  * @retval                none
  */
static void cam_fifo_clear(camera_channel_t *cam){
    cam->fifo_count = 0;
    for (uint8_t i=0;i<CAM_FIFO_CAP;i++){ cam->fifo[i].frame_id = 0; cam->fifo[i].ts_us = 0; }
}       

/**
  * @brief                 摄像头通道RX回调（处理主机复位命令）
  * @param[in] ch          通道对象（未使用）
  * @param[in] payload     接收数据缓冲区
  * @param[in] len         数据长度（字节）
  * @param[in] user        用户上下文（camera_channel_t指针）
  * @retval                none
  */
static void camera_on_rx(channel_t *ch, const uint8_t *payload, uint32_t len, void *user)
{
    (void)ch;
    camera_channel_t *cam = (camera_channel_t*)user;
    if (!payload || len < 2u || !cam) return;
    uint16_t sid = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if (sid == CAM_SID_RESET) {
        cam_fifo_clear(cam);
        cam->next_frame_id = 1u;
        cam->ack_pending = 1u; /* 延迟到tick时发送ACK */
        if (cam->hooks.on_reset) cam->hooks.on_reset(cam, cam->hooks.user);
    }
}

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
                            const camera_channel_hooks_t *hooks)
{
    if (!cam || !bind || !mgr || !cfg) return;
    platform_memset(cam, 0, sizeof(*cam));
    cam->bind = bind;
    cam->mgr = mgr;
    cam->ch_id = cfg->ch_id;
    cam->now_us = now_us;
    cam->time_user = time_user;
    if (hooks) cam->hooks = *hooks; else platform_memset(&cam->hooks, 0, sizeof(cam->hooks));
    cam->next_frame_id = 1u;
    cam_fifo_clear(cam);
    /* no rate limiting by default; send all events */

    channel_cfg_t ch_cfg = { .mode = CH_MODE_PUSH, .reliable = 0, .expect_reply = 0, .priority = cfg->priority, .period_ms = 0 };
    channel_hooks_t ch_hooks = { .on_rx = camera_on_rx, .on_tick = NULL, .on_cfg_change = NULL };
    channel_init(&cam->ch, cam->ch_id, &ch_cfg, &ch_hooks, cam);
    channel_bind_transport(&cam->ch, cam->bind, cam->bind->msg_type);
    (void)chmgr_register(cam->mgr, &cam->ch);
}


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
                         const camera_channel_hooks_t *hooks)
{
    camera_channel_cfg_t cfg = { .ch_id = ch_id, .priority = CAM_PRIORITY };
    camera_channel_init_ex(cam, bind, mgr, &cfg, now_us, time_user, hooks);
}

/**
  * @brief                  摄像头触发边沿调用
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_trigger(camera_channel_t *cam)
{
    if (!cam) return;
#if !defined(CAM_EVENT_PUBLISH_ENABLE) || !(CAM_EVENT_PUBLISH_ENABLE)
    return;
#endif
    uint64_t ts = 0;
    if (cam->now_us) ts = cam->now_us(cam->time_user);
    cam_evt_t evt; evt.frame_id = cam->next_frame_id++; evt.ts_us = ts;
    if (cam->fifo_count < CAM_FIFO_CAP) {
        cam->fifo[cam->fifo_count++] = evt;
    }
    /* build payload using config-controlled packer */
    uint8_t buf[2 + 4 + 8];
    uint16_t len = camera_pack_event(buf, CAM_SID_EVENT, evt.frame_id, evt.ts_us);
    (void)ch_uproto_queue_notify(cam->bind, cam->ch_id, CAM_SID_EVENT, 0, buf, len);
    if (cam->hooks.on_event) cam->hooks.on_event(cam, evt.frame_id, evt.ts_us, cam->hooks.user);
}

/**
  * @brief                  周期性处理（发送延迟ACK等）
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_tick(camera_channel_t *cam)
{
    if (!cam) return;
    if (cam->ack_pending) {
        uint8_t ack[2];
        comm_write_u16_le(ack, CAM_SID_RESET);
        if (ch_uproto_queue_notify(cam->bind, cam->ch_id, CAM_SID_RESET, 0, ack, 2u)) {
            cam->ack_pending = 0u;
        }
    }
}

/**
  * @brief                  显式复位（本地操作）
  * @param[in] cam          摄像头通道对象
  * @retval                 none
  */
void camera_channel_reset(camera_channel_t *cam)
{
    if (!cam) return;
    cam_fifo_clear(cam);
    cam->next_frame_id = 1u;
}

/**
  * @brief                  获取下一帧ID
  * @param[in] cam          摄像头通道对象
  * @retval                 下一帧ID
  */
uint32_t camera_channel_next_frame_id(const camera_channel_t *cam)
{
    return cam ? cam->next_frame_id : 0u;
}

/* no rate control API when sending all events */
