#pragma once

#include <cstdint>
#include <string>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/select.h>

extern "C" {
#include "../../shared/protocol_ids.h"
#include "../../core/uproto.h"
#include "../../core/config.h"
#include "../../core/comm_utils.h"
}
#include "ts_host.hpp"
#include "gimbal_host.hpp"
#include "platform_posix.hpp"
#include "comm_shim.hpp"

class ISerialPort;
class IPlatform;

class HostApp {
public:
    inline HostApp(ISerialPort &port, IPlatform &platform) : port_(port), plat_(platform), ts_(
        TS_CH_ID, 1000u, true,
        [this](uint8_t ch, uint16_t sid, const uint8_t* p, uint16_t l){ return this->send_mux(ch, sid, p, l); }
    ) { self_ = this; }
    inline ~HostApp() { self_ = nullptr; }

    inline bool init(const std::string &dev, int baud) {
        if (!port_.open(dev, baud)) {
            std::fprintf(stderr, "failed to open %s\n", dev.c_str());
            return false;
        }
        plat_.init_uproto(&up_, &port_);
        (void)uproto_register_handler(&up_, UPROTO_MSG_MUX, &HostApp::on_uproto_mux, this);
        ts_.set_max_rtt_us(0);
        return true;
    }

    inline int run_loop() {
        std::printf("Listening on fd=%d ...\n", port_.fd());
        for(;;) {
            ts_.tick();
            plat_.tick(&up_);

            fd_set rfds; FD_ZERO(&rfds); FD_SET(port_.fd(), &rfds);
            struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
            int r = select(port_.fd() + 1, &rfds, NULL, NULL, &tv);
            if(r > 0 && FD_ISSET(port_.fd(), &rfds)) {
                uint8_t buf[1024];
                long n = port_.read_some(buf, sizeof(buf));
                if(n > 0) {
                    uproto_on_rx_bytes(&up_, buf, (uint32_t)n);
                }
            }
        }
        return 0;
    }

    // Send one MUX frame via uproto (notify)
    inline int send_mux(uint8_t ch, uint16_t sid, const uint8_t *payload, uint16_t len) {
        static uint32_t seqs[256] = {0};
        uint8_t buf[COMM_MUX_TX_BUFFER_SIZE];
        uint32_t wrote = cpp_mux_encode_build(ch, sid, ++seqs[ch], payload, len, buf, sizeof(buf));
        if(wrote == 0)
            return -1;
        return (uproto_send_notify(&up_, UPROTO_MSG_MUX, buf, wrote) == UPROTO_OK) ? 0 : -1;
    }

    // Accessors
    inline static HostApp* instance() { return self_; }
    inline uproto_context_t* uproto() { return &up_; }

private:
    inline static void on_uproto_mux(uproto_context_t *ctx, uint16_t txn_id,
                              const uint8_t *data, uint32_t len, void *user) {
        (void)ctx; (void)txn_id;
        HostApp *self = static_cast<HostApp*>(user);
        if (!self) return;
        uint8_t ch = 0; const uint8_t *pl = nullptr; uint32_t pl_len = 0;
        if (!cpp_mux_decode(data, len, &ch, &pl, &pl_len)) return;
        self->handle_mux_payload(ch, pl, pl_len);
    }

    inline void handle_mux_payload(uint8_t ch, const uint8_t *pl, uint32_t len) {
        if(!pl || len < 2) return;
        uint16_t sid = comm_read_u16_le(pl);
        if(ch == CAM_CH_ID) {
            if(sid == CAM_SID_RESET) {
                std::printf("CAM RESET ACK\n");
            } else if(len >= 2 + 4 + 8 && sid == CAM_SID_EVENT) {
                uint32_t frame_id = comm_read_u32_le(pl + 2);
                uint64_t ts_us = comm_read_u64_le(pl + 6);
                uint64_t now = []{ struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec*1000000ULL + (uint64_t)ts.tv_nsec/1000ULL; }();
                if(last_cam_print_us_ == 0 || (now - last_cam_print_us_) >= 1000000ULL) {
                    std::printf("CAM TRIG id=0x%08X ts=0x%016llX\n", frame_id, (unsigned long long)ts_us);
                    last_cam_print_us_ = now;
                }
            } else {
                std::printf("CAM CH1 sid=0x%04X len=%u\n", sid, (unsigned)len);
            }
        } else if(ch == TS_CH_ID) {
            if(sid == TS_SID_REQ) {
                uint16_t seq = comm_read_u16_le(pl + 2);
                ts_.on_mux(pl, len);
                std::printf("[TS] REQ seq=%u -> RESP sent\n", (unsigned)seq);
            } else if(sid == TS_SID_RESP) {
                uint16_t seq = comm_read_u16_le(pl + 2);
                ts_.on_mux(pl, len);
                uint32_t rtt = ts_.last_rtt_us();
                unsigned long long dev_now = (unsigned long long)ts_.now_device_us();
                std::printf("[TS] RESP seq=%u rtt=%u us dev_now=0x%016llX\n",
                        (unsigned)seq, (unsigned)rtt, dev_now);
            } else {
                std::printf("TS CH3 sid=0x%04X len=%u\n", sid, (unsigned)len);
                ts_.on_mux(pl, len);
            }
        } else if (ch == GIMBAL_CH_ID && sid == GIMBAL_SID_STATE) {
            gimbal_on_state(pl, len);
        } else if (ch == GIMBAL_CH_ID && sid == GIMBAL_SID_TFMINI) {
            if(len >= 2 + 2 + 2 + 2 + 2 + 8) {
                uint16_t dist_cm = comm_read_u16_le(pl + 2);
                uint16_t strength = comm_read_u16_le(pl + 4);
                int16_t temp_cdeg = (int16_t)comm_read_u16_le(pl + 6);
                uint16_t status = comm_read_u16_le(pl + 8);
                uint64_t ts_us = comm_read_u64_le(pl + 10);
                std::printf("[TFMINI] dist=%u cm strength=%u temp=%.2fC status=0x%04X ts=0x%016llX\n",
                            (unsigned)dist_cm, (unsigned)strength, (double)temp_cdeg / 100.0,
                            (unsigned)status, (unsigned long long)ts_us);
            }
        } else {
            std::printf("CH%u sid=0x%04X len=%u\n", ch, sid, (unsigned)len);
        }
    }

    ISerialPort &port_;
    IPlatform   &plat_;
    uproto_context_t up_{};
    TsHost ts_;
    uint64_t last_cam_print_us_ = 0;
    inline static HostApp *self_ = nullptr;
};

// C ABI bridge for C modules expecting this symbol
extern "C" inline int host_send_mux(int fd, uint8_t ch, uint16_t sid,
                              const uint8_t *payload, uint16_t pl_len) {
    (void)fd; // fd handled by uproto port
    HostApp *self = HostApp::instance();
    if (!self) return -1;
    return self->send_mux(ch, sid, payload, pl_len);
}
