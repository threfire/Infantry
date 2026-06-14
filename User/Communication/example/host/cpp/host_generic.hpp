// Generic Host, clean minimal implementation
#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include <atomic>
#include <sstream>
#include <iostream>
#if defined(_WIN32)
#include <winsock2.h>
#else
#include <sys/select.h>
#include <time.h>
#endif

extern "C" {
#include "../../core/comm_utils.h"
#include "../../core/config.h"
#include "../../core/uproto.h"
#include "../../core/comm.h"
#include "../../shared/protocol_ids.h"
}

#include "hal/transport.hpp"

inline uint32_t mux_build_frame(uint8_t ch, uint16_t sid, uint32_t seq,
                                const uint8_t *pl, uint16_t len,
                                uint8_t *out, uint32_t cap) {
    mux_hdr_t h{};
    h.sof = MUX_SOF; h.ver = MUX_VER; h.channel = ch; h.len = len; h.sid = sid; h.seq = seq;
    return mux_encode(out, cap, &h, pl);
}
inline bool mux_parse_frame(const uint8_t *data, uint32_t len, uint8_t *ch, const uint8_t **pl, uint32_t *pl_len) {
    mux_hdr_t h{}; const uint8_t *p = nullptr; if(!mux_decode(data, len, &h, &p)) return false;
    if(ch) *ch = h.channel; if(pl) *pl = p; if(pl_len) *pl_len = h.len; return true;
}

class TsLite {
  public:
    using SendFn = std::function<int(uint8_t, uint16_t, const uint8_t*, uint16_t)>;
    TsLite(uint8_t ch_id, uint32_t period_ms, bool initiator, SendFn f)
        : ch_(ch_id), per_(period_ms ? period_ms : 1000u), init_(initiator), send_(std::move(f)) {}
    void set_max_rtt_us(uint32_t v) { max_rtt_ = v; }
    void tick() {
        if(!init_) return; uint64_t now = now_us(); if(now - last_ >= (uint64_t)per_ * 1000ULL) { last_ = now; send_req(); }
    }
    void on_mux(const uint8_t *pl, uint32_t len) {
        if(!pl || len < 2) return; uint16_t sid = comm_read_u16_le(pl);
        if(sid == TS_SID_REQ) {
            if(len < 2 + 2 + 8) return; uint16_t seq = comm_read_u16_le(pl + 2); (void)seq;
            uint64_t t1 = now_us(), t2 = now_us(); uint8_t buf[2 + 2 + 8 + 8];
            comm_write_u16_le(&buf[0], TS_SID_RESP); comm_write_u16_le(&buf[2], seq);
            comm_write_u64_le(&buf[4], t1); comm_write_u64_le(&buf[12], t2);
            (void)send_(ch_, TS_SID_RESP, buf, sizeof(buf));
        } else if(sid == TS_SID_RESP) {
            if(len < 2 + 2 + 8 + 8) return; uint16_t seq = comm_read_u16_le(pl + 2);
            uint64_t t1 = comm_read_u64_le(pl + 4), t2 = comm_read_u64_le(pl + 12), t3 = now_us(); uint64_t t0 = 0;
            for(auto &e : hist_) if(e.seq == seq){ t0=e.t0; break; } if(!t0) return; uint64_t r = (t3 - t0); if(t2 >= t1) r -= (t2 - t1);
            rtt_ = (uint32_t)r;
        }
    }
    uint32_t last_rtt_us() const { return rtt_; }
  private:
    static uint64_t now_us() {
#if defined(_WIN32)
        static LARGE_INTEGER freq; static BOOL inited=FALSE; if(!inited){ QueryPerformanceFrequency(&freq); inited=TRUE; }
        LARGE_INTEGER c; QueryPerformanceCounter(&c); return (uint64_t)(c.QuadPart * 1000000ULL / freq.QuadPart);
#else
        struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
#endif
    }
    void send_req() {
        if(!send_) return; uint16_t seq = seq_++; uint64_t t0 = now_us(); uint8_t buf[2 + 2 + 8];
        comm_write_u16_le(&buf[0], TS_SID_REQ); comm_write_u16_le(&buf[2], seq); comm_write_u64_le(&buf[4], t0);
        hist_[pos_] = {seq, t0}; pos_ = (uint8_t)((pos_ + 1) & 0x1F); (void)send_(ch_, TS_SID_REQ, buf, sizeof(buf));
    }
    struct Item { uint16_t seq; uint64_t t0; } hist_[32]{};
    uint8_t pos_ = 0; uint8_t ch_ = 0; uint32_t per_ = 1000; bool init_ = false; SendFn send_{}; uint16_t seq_ = 0; uint64_t last_ = 0; uint32_t rtt_ = 0; uint32_t max_rtt_ = 0;
};

template <class Transport>
class GenericPlatform {
  public:
    void init_uproto(uproto_context_t *ctx, Transport *tp) {
        tp_ = tp; uproto_port_ops_t pop{}; pop.user = this;
        pop.get_mtu = [](void *u)->uint16_t { auto *self=(GenericPlatform*)u; return self && self->tp_ ? self->tp_->mtu() : 1024; };
        pop.flush = [](void*){};
        pop.write = [](void *u, const uint8_t *d, uint32_t l)->uint32_t { auto *self=(GenericPlatform*)u; return (self&&self->tp_&& self->tp_->write_all(d,l))?l:0u; };
        uproto_time_ops_t to{}; to.user=nullptr; to.now_ms = [](void*)->uint32_t { auto now=std::chrono::steady_clock::now(); return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count(); };
        uproto_config_t cfg{}; cfg.handshake_timeout_ms=1000; cfg.heartbeat_interval_ms=0; cfg.default_timeout_ms=3000; cfg.default_retries=1; cfg.enable_auto_handshake=true; cfg.event_cb=nullptr; cfg.event_user=nullptr;
        uproto_init(ctx, &pop, &to, &cfg);
    }
    void tick(uproto_context_t *ctx) { if(ctx) uproto_tick(ctx); }
  private: Transport *tp_ = nullptr; };


template <class Transport, class Platform>
class HostAppGT {
  public:
    HostAppGT() : ts_(TS_CH_ID, 1000u, true, [this](uint8_t ch, uint16_t sid, const uint8_t *p, uint16_t l){ return this->send_mux(ch,sid,p,l); }) {}
    bool init(const std::string &endpoint) {
        std::printf("Opening %s ...\n", endpoint.c_str());
        if(!tp_.open(endpoint)) { std::fprintf(stderr, "failed to open %s\n", endpoint.c_str()); return false; }
        plat_.init_uproto(&up_, &tp_);
        (void)uproto_register_handler(&up_, UPROTO_MSG_MUX, &HostAppGT::on_mux, this);
        (void)uproto_start_handshake(&up_);
        ts_.set_max_rtt_us(0u);
        return true;
    }
    int run_loop() {
        for(;;){
            ts_.tick();
            sine_tick_();
            plat_.tick(&up_);
            int fd = tp_.fd();
            if(fd >= 0){ fd_set rfds; FD_ZERO(&rfds); FD_SET(fd,&rfds); struct timeval tv{0,20000}; int r=select(fd+1,&rfds,nullptr,nullptr,&tv); if(r>0&&FD_ISSET(fd,&rfds)){ uint8_t buf[1024]; long n=tp_.read_some(buf,sizeof(buf)); if(n>0) uproto_on_rx_bytes(&up_, buf, (uint32_t)n);} }
            else { (void)tp_.wait_readable(10); uint8_t buf[1024]; long n=tp_.read_some(buf,sizeof(buf)); if(n>0) uproto_on_rx_bytes(&up_, buf, (uint32_t)n);} }
        return 0;
    }
    int send_mux(uint8_t ch, uint16_t sid, const uint8_t *pl, uint16_t len) {
        static uint32_t seqs[256]={0};
        uint8_t buf[COMM_MUX_TX_BUFFER_SIZE];
        uint32_t w=mux_build_frame(ch,sid,++seqs[ch],pl,len,buf,sizeof(buf));
        if(!w) return -1;
        // TX log: channel, sid, len and a short hex preview
        if(log_tx_) {
            std::printf("TX  CH%u sid=0x%04X len=%u", ch, sid, (unsigned)len);
            if(pl && len){
                std::printf(" data=");
                uint16_t show = len < 32 ? len : 32; // preview up to 32 bytes
                for(uint16_t i=0;i<show;i++) std::printf("%02X", pl[i]);
                if(len>show) std::printf("...");
            }
            std::printf("\n");
        }
        return (uproto_send_notify(&up_, UPROTO_MSG_MUX, buf, w)==UPROTO_OK)?0:-1;
    }
    // Public controls for external flags
    void quiet(){ log_rx_ = false; log_tx_ = false; }
    void start_sine(const char *axis, double freq_hz, double amp_deg, uint32_t rate_hz){
        Axis ax = Axis::None; if(axis){ if(std::string(axis)=="yaw") ax=Axis::Yaw; else if(std::string(axis)=="pitch") ax=Axis::Pitch; }
        std::lock_guard<std::mutex> lk(sine_mtx_);
        sine_cfg_.axis = ax; sine_cfg_.freq_hz = freq_hz; sine_cfg_.amp_deg = amp_deg; sine_cfg_.rate_hz = rate_hz?rate_hz:50;
        yaw_pos_prev_ = 0.0; pitch_pos_prev_ = 0.0; sine_next_us_ = 0;
    }
  private:
    static void on_mux(uproto_context_t*, uint16_t, const uint8_t *data, uint32_t len, void *user){ auto*self=(HostAppGT*)user; if(!self) return; uint8_t ch=0; const uint8_t*pl=nullptr; uint32_t pl_len=0; if(!mux_parse_frame(data,len,&ch,&pl,&pl_len)) return; self->handle(ch,pl,pl_len);}    
    void handle(uint8_t ch, const uint8_t *pl, uint32_t len){
        if(!pl||len<2) return;
        uint16_t sid=comm_read_u16_le(pl);
        // RX log for every frame
        if(log_rx_) {
            std::printf("RX  CH%u sid=0x%04X len=%u", ch, sid, (unsigned)len);
            if(len>2){ std::printf(" data="); uint16_t show = len<32?len:32; for(uint16_t i=0;i<show;i++){ std::printf("%02X", pl[i]); } if(len>show) std::printf("..."); }
            std::printf("\n");
        }
        if(ch==TS_CH_ID){ ts_.on_mux(pl,len); if(sid==TS_SID_RESP){ std::printf("[TS] RESP rtt=%u us\n", (unsigned)ts_.last_rtt_us()); } return;}
        if(ch==CAM_CH_ID){ if(sid==CAM_SID_EVENT && len>=2+4+8){ uint32_t frame_id=comm_read_u32_le(pl+2); uint64_t ts=comm_read_u64_le(pl+6); std::printf("CAM TRIG id=0x%08X ts=0x%016llX\n", frame_id, (unsigned long long)ts);} return;}
        if(ch==GIMBAL_CH_ID){
            if(sid==GIMBAL_SID_STATE && len>=2+4+4+4+4+4+8){ int32_t enc_yaw=comm_read_i32_le(pl+2), enc_pitch=comm_read_i32_le(pl+6); int32_t yaw_udeg=comm_read_i32_le(pl+10), pitch_udeg=comm_read_i32_le(pl+14), roll_udeg=comm_read_i32_le(pl+18); uint64_t ts=comm_read_u64_le(pl+22); auto u2d=[](int32_t u)->double{ return (double)u/1000000.0;}; std::printf("[GIMBAL] enc_yaw=%d enc_pitch=%d yaw=%.3f pitch=%.3f roll=%.3f ts=0x%016llX\n", enc_yaw, enc_pitch, u2d(yaw_udeg), u2d(pitch_udeg), u2d(roll_udeg), (unsigned long long)ts); return; }
            if(sid==GIMBAL_SID_TFMINI && len>=2+2+2+2+2+8){ uint16_t dist_cm=comm_read_u16_le(pl+2); uint16_t strength=comm_read_u16_le(pl+4); int16_t temp_cdeg=(int16_t)comm_read_u16_le(pl+6); uint16_t status=comm_read_u16_le(pl+8); uint64_t ts=comm_read_u64_le(pl+10); double temp_c=(double)temp_cdeg/100.0; std::printf("[TFMINI] dist=%u cm strength=%u temp=%.2fC status=0x%04X ts=0x%016llX\n", (unsigned)dist_cm, (unsigned)strength, temp_c, (unsigned)status, (unsigned long long)ts); return; }
            if(sid==GIMBAL_SID_DELTA){ std::printf("[GIMBAL] DELTA ACK\n"); return; }
            return;
        }
        std::printf("CH%u sid=0x%04X len=%u\n", ch, sid, (unsigned)len);
    }
    // -------- Sine generator + CLI ----------
    enum class Axis { None=0, Yaw=1, Pitch=2 };
    struct SineCfg { Axis axis = Axis::None; double freq_hz = 0.0; double amp_deg = 0.0; uint32_t rate_hz = 50; };
    std::mutex sine_mtx_{};
    SineCfg sine_cfg_{};
    uint64_t sine_next_us_ = 0;
    double yaw_pos_prev_ = 0.0, pitch_pos_prev_ = 0.0;
    bool log_rx_ = true, log_tx_ = true;

    static uint64_t now_us_() { auto now=std::chrono::steady_clock::now(); return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count(); }
    static int32_t deg_to_udeg_(double d){ return (int32_t)(d*1000000.0 + (d>=0?0.5:-0.5)); }

    void sine_tick_(){
        SineCfg cfg; { std::lock_guard<std::mutex> lk(sine_mtx_); cfg = sine_cfg_; }
        if(cfg.axis == Axis::None || cfg.freq_hz <= 0.0 || cfg.amp_deg <= 0.0 || cfg.rate_hz == 0)
            return;
        uint64_t now = now_us_();
        uint64_t period_us = (uint64_t)(1000000ULL / (cfg.rate_hz ? cfg.rate_hz : 50));
        if(sine_next_us_ == 0) sine_next_us_ = now;
        if(now < sine_next_us_) return;
        sine_next_us_ += period_us;

        double t = (double)now/1000000.0;
        double pos = cfg.amp_deg * std::sin(2.0 * 3.141592653589793 * cfg.freq_hz * t);
        int32_t dyaw = 0, dpitch = 0;
        if(cfg.axis == Axis::Yaw){ double d = pos - yaw_pos_prev_; dyaw = deg_to_udeg_(d); yaw_pos_prev_ = pos; }
        else if(cfg.axis == Axis::Pitch){ double d = pos - pitch_pos_prev_; dpitch = deg_to_udeg_(d); pitch_pos_prev_ = pos; }
        send_gimbal_delta_(dyaw, dpitch, 0, now);
    }

    int send_gimbal_delta_(int32_t dyaw_udeg, int32_t dpitch_udeg, uint16_t status, uint64_t ts_us){
        uint8_t buf[2 + 4 + 4 + 2 + 8];
        comm_write_u16_le(&buf[0], GIMBAL_SID_DELTA);
        comm_write_i32_le(&buf[2], dyaw_udeg);
        comm_write_i32_le(&buf[6], dpitch_udeg);
        comm_write_u16_le(&buf[10], status);
        comm_write_u64_le(&buf[12], ts_us);
        return send_mux(GIMBAL_CH_ID, GIMBAL_SID_DELTA, buf, sizeof(buf));
    }

    void start_console_(){
        console_stop_.store(false);
        console_thr_ = std::thread([this]{ console_loop_(); });
        console_thr_.detach();
        std::printf("Type: cil help\n");
    }
    void console_loop_(){
        for(std::string line; !console_stop_.load() && std::getline(std::cin, line); ){
            handle_cli_line_(line);
        }
    }
    void handle_cli_line_(const std::string &line){
        std::istringstream iss(line);
        std::string cmd; if(!(iss>>cmd)) return;
        if(cmd != "cil") return;
        std::string sub; if(!(iss>>sub)){ print_help_(); return; }
        if(sub == "help"){ print_help_(); return; }
        if(sub == "quiet"){ log_rx_=false; log_tx_=false; std::printf("[CIL] logging off (rx/tx)\n"); return; }
        if(sub == "log"){ std::string which, onoff; if(!(iss>>which>>onoff)){ print_help_(); return; } bool on = (onoff=="on"); if(which=="rx") log_rx_=on; else if(which=="tx") log_tx_=on; else if(which=="all") { log_rx_=on; log_tx_=on; } else if(which=="off") { log_rx_=false; log_tx_=false; } else { print_help_(); return; } std::printf("[CIL] log %s %s\n", which.c_str(), on?"on":"off"); return; }
        if(sub == "stop"){ std::lock_guard<std::mutex> lk(sine_mtx_); sine_cfg_ = {}; std::printf("[CIL] sine stopped\n"); return; }
        if(sub == "sine"){
            std::string axis; double freq=0, amp=0; uint32_t rate=50; if(!(iss>>axis>>freq>>amp)) { print_help_(); return; }
            if(iss.good()) iss>>rate;
            Axis ax = Axis::None; if(axis=="yaw") ax=Axis::Yaw; else if(axis=="pitch") ax=Axis::Pitch; else { print_help_(); return; }
            {
                std::lock_guard<std::mutex> lk(sine_mtx_);
                sine_cfg_.axis = ax; sine_cfg_.freq_hz=freq; sine_cfg_.amp_deg=amp; sine_cfg_.rate_hz= rate?rate:50;
                yaw_pos_prev_ = 0.0; pitch_pos_prev_ = 0.0; sine_next_us_ = 0;
            }
            std::printf("[CIL] sine axis=%s freq=%.3fHz amp=%.3fdeg rate=%uHz\n", axis.c_str(), freq, amp, (unsigned)rate);
            return;
        }
        print_help_();
    }
    void print_help_(){
        std::printf("CIL usage:\n  cil sine yaw <freq_hz> <amp_deg> [rate_hz]\n  cil sine pitch <freq_hz> <amp_deg> [rate_hz]\n  cil stop\n");
    }

    std::atomic<bool> console_stop_{false};
    std::thread console_thr_{};

    // -------- Existing members ----------
    Transport tp_{}; Platform plat_{}; uproto_context_t up_{}; TsLite ts_;
};
