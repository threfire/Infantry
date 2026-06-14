// Single-header host implementation (POSIX), header-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <string>
#include <sys/select.h>
#include <utility>

extern "C" {
#include "../../core/comm.h"           // mux_encode/mux_decode
#include "../../core/comm_utils.h"     // LE read/write helpers
#include "../../core/config.h"         // COMM_MUX_TX_BUFFER_SIZE
#include "../../core/platform.h"       // platform_* prototypes
#include "../../core/uproto.h"         // uproto API
#include "../../shared/protocol_ids.h" // channel/sid ids
}

// ========== Core platform shim (implementations guarded by macro) ==========
#ifdef HOST_PLATFORM_SHIM_IMPL
extern "C" uint32_t platform_get_tick_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}
extern "C" void platform_delay_ms(uint32_t ms) {
    struct timespec req{(time_t)(ms / 1000u), (long)((ms % 1000u) * 1000000ul)};
    nanosleep(&req, nullptr);
}
extern "C" void *platform_memcpy(void *dst, const void *src, uint32_t n) {
    auto *d = (unsigned char *)dst;
    auto *s = (const unsigned char *)src;
    while(n--)
        *d++ = *s++;
    return dst;
}
extern "C" void *platform_memset(void *s, int c, uint32_t n) {
    auto *p = (unsigned char *)s;
    unsigned char v = (unsigned char)c;
    while(n--)
        *p++ = v;
    return s;
}
extern "C" void *platform_memmove(void *dst, const void *src, uint32_t n) {
    auto *d = (unsigned char *)dst;
    auto *s = (const unsigned char *)src;
    if(d < s) {
        while(n--)
            *d++ = *s++;
    } else if(d > s) {
        d += n;
        s += n;
        while(n--)
            *--d = *--s;
    }
    return dst;
}
extern "C" int platform_memcmp(const void *s1, const void *s2, uint32_t n) {
    auto *a = (const unsigned char *)s1;
    auto *b = (const unsigned char *)s2;
    while(n--) {
        if(*a != *b)
            return (*a < *b) ? -1 : 1;
        ++a;
        ++b;
    }
    return 0;
}
extern "C" uint32_t platform_strlen(const char *s) {
    uint32_t l = 0;
    while(s && *s++)
        ++l;
    return l;
}
extern "C" char *platform_strcpy(char *dst, const char *src) {
    char *ret = dst;
    while(src && (*dst++ = *src++)) {
    }
    return ret;
}
extern "C" int platform_strcmp(const char *s1, const char *s2) {
    while(s1 && s2 && *s1 && (*s1 == *s2)) {
        ++s1;
        ++s2;
    }
    return *(const unsigned char *)s1 - *(const unsigned char *)s2;
}
#endif

// ========== MUX helpers (C++ inline wrappers over core) ==========
inline uint32_t cpp_mux_encode_build(uint8_t ch, uint16_t sid, uint32_t seq,
                                     const uint8_t *payload, uint16_t len,
                                     uint8_t *out, uint32_t cap) {
    mux_hdr_t hdr{};
    hdr.sof = MUX_SOF;
    hdr.ver = MUX_VER;
    hdr.channel = ch;
    hdr.flags = 0;
    hdr.len = len;
    hdr.sid = sid;
    hdr.seq = seq;
    return mux_encode(out, cap, &hdr, payload);
}
inline bool cpp_mux_decode(const uint8_t *data, uint32_t len,
                           uint8_t *out_ch, const uint8_t **out_pl, uint32_t *out_pl_len) {
    mux_hdr_t hdr{};
    const uint8_t *pl = nullptr;
    if(!mux_decode(data, len, &hdr, &pl))
        return false;
    if(out_ch)
        *out_ch = hdr.channel;
    if(out_pl)
        *out_pl = pl;
    if(out_pl_len)
        *out_pl_len = hdr.len;
    return true;
}

// ========== Header-only TsHost (time sync) ==========
class TsHost {
  public:
    using SendMuxFn = std::function<int(uint8_t, uint16_t, const uint8_t *, uint16_t)>;
    inline TsHost(uint8_t ch_id, uint32_t period_ms, bool initiator, SendMuxFn send)
        : ch_id_(ch_id), period_ms_(period_ms ? period_ms : 1000u), initiator_(initiator), send_(std::move(send)) {
    }
    inline void set_max_rtt_us(uint32_t v) {
        max_rtt_us_ = v;
    }
    inline void tick() {
        if(!initiator_)
            return;
        uint64_t now = now_us_();
        if(now - last_req_us_ >= (uint64_t)period_ms_ * 1000ULL) {
            last_req_us_ = now;
            send_req_();
        }
    }
    inline void on_mux(const uint8_t *pl, uint32_t len) {
        if(!pl || len < 2)
            return;
        uint16_t sid = comm_read_u16_le(pl);
        if(sid == TS_SID_REQ) {
            if(len < 2 + 2 + 8)
                return;
            uint16_t seq = comm_read_u16_le(pl + 2);
            (void)seq;
            uint64_t t1 = now_us_(), t2 = now_us_();
            uint8_t buf[2 + 2 + 8 + 8];
            comm_write_u16_le(&buf[0], TS_SID_RESP);
            comm_write_u16_le(&buf[2], seq);
            comm_write_u64_le(&buf[4], t1);
            comm_write_u64_le(&buf[12], t2);
            (void)send_(ch_id_, TS_SID_RESP, buf, sizeof(buf));
        } else if(sid == TS_SID_RESP) {
            if(len < 2 + 2 + 8 + 8)
                return;
            uint16_t seq = comm_read_u16_le(pl + 2);
            uint64_t t1 = comm_read_u64_le(pl + 4), t2 = comm_read_u64_le(pl + 12), t3 = now_us_();
            uint64_t t0 = 0;
            for(auto &h : hist_) {
                if(h.seq == seq) {
                    t0 = h.t0;
                    break;
                }
            }
            if(!t0)
                return;
            uint64_t rtt = (t3 - t0);
            if(t2 >= t1)
                rtt -= (t2 - t1);
            rtt_us_last_ = (uint32_t)rtt;
            int64_t offset = ((int64_t)(t1 - t0) + (int64_t)(t2 - t3)) / 2;
            if(max_rtt_us_ == 0 || rtt <= max_rtt_us_) {
                if(!offset_origin_valid_) {
                    offset_origin_us_ = offset;
                    offset_origin_valid_ = true;
                }
                int64_t rel = offset - offset_origin_us_;
                offset_us_ = (offset_us_ * 9 + offset) / 10;
                offset_display_us_ = (offset_display_us_ * 9 + rel) / 10;
                mapping_valid_ = true;
                mapping_version_++;
            }
        }
    }
    inline uint64_t now_device_us() const {
        uint64_t now = now_us_();
        return mapping_valid_ ? (uint64_t)((int64_t)now + offset_us_) : now;
    }
    inline uint32_t last_rtt_us() const {
        return rtt_us_last_;
    }

  private:
    inline static uint64_t now_us_() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    }
    inline void send_req_() {
        if(!send_)
            return;
        uint16_t seq = seq_++;
        uint64_t t0 = now_us_();
        uint8_t buf[2 + 2 + 8];
        comm_write_u16_le(&buf[0], TS_SID_REQ);
        comm_write_u16_le(&buf[2], seq);
        comm_write_u64_le(&buf[4], t0);
        hist_[hist_pos_] = {seq, t0};
        hist_pos_ = (uint8_t)((hist_pos_ + 1) & 0x1F);
        (void)send_(ch_id_, TS_SID_REQ, buf, sizeof(buf));
    }
    struct Hist {
        uint16_t seq;
        uint64_t t0;
    };
    uint8_t ch_id_ = 0;
    uint32_t period_ms_ = 1000;
    bool initiator_ = false;
    SendMuxFn send_{};
    uint16_t seq_ = 0;
    Hist hist_[32]{};
    uint8_t hist_pos_ = 0;
    uint64_t last_req_us_ = 0;
    uint32_t rtt_us_last_ = 0;
    int64_t offset_us_ = 0;
    int64_t offset_origin_us_ = 0;
    bool offset_origin_valid_ = false;
    int64_t offset_display_us_ = 0;
    bool mapping_valid_ = false;
    uint32_t mapping_version_ = 0;
    uint32_t max_rtt_us_ = 0;
};

// ========== Gimbal helpers ==========
inline int32_t deg_to_udeg(double deg) {
    return (int32_t)(deg * 1000000.0 + (deg >= 0 ? 0.5 : -0.5));
}
inline double udeg_to_deg(int32_t udeg) {
    return (double)udeg / 1000000.0;
}
inline double udeg_to_pi(int32_t udeg) {
    return ((double)udeg / 1000000.0) / 180.0;
}
inline void gimbal_on_state(const uint8_t *pl, uint32_t len) {
    if(!pl || len < 2 + 4 + 4 + 4 + 4 + 4 + 8)
        return;
    int32_t enc_yaw = comm_read_i32_le(pl + 2);
    int32_t enc_pitch = comm_read_i32_le(pl + 6);
    int32_t yaw_udeg = comm_read_i32_le(pl + 10);
    int32_t pitch_udeg = comm_read_i32_le(pl + 14);
    int32_t roll_udeg = comm_read_i32_le(pl + 18);
    uint64_t ts_us = comm_read_u64_le(pl + 22);
    std::printf("[GIMBAL] enc_yaw=%d enc_pitch=%d yaw=%.3f deg pitch=%.3f deg roll=%.3f deg ts=0x%016llX\n", enc_yaw, enc_pitch, udeg_to_deg(yaw_udeg), udeg_to_deg(pitch_udeg), udeg_to_deg(roll_udeg), (unsigned long long)ts_us);
}

// ========== POSIX Serial (header-only) ==========
#if !defined(_WIN32)
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
inline speed_t map_baud_posix(int baud) {
    switch(baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
    default:
        return B115200;
    }
}
class PosixSerialPort {
  public:
    inline bool open(const std::string &path, int baud) {
        close();
        int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(fd < 0)
            return false;
        struct termios tio;
        if(tcgetattr(fd, &tio) != 0) {
            ::close(fd);
            return false;
        }
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        speed_t sp = map_baud_posix(baud);
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
#ifdef CRTSCTS
        tio.c_cflag &= ~CRTSCTS;
#endif
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 1;
        if(tcsetattr(fd, TCSANOW, &tio) != 0) {
            ::close(fd);
            return false;
        }
        fd_ = fd;
        return true;
    }
    inline void close() {
        if(fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    inline int fd() const {
        return fd_;
    }
    inline bool write_all(const uint8_t *buf, size_t len) {
        if(fd_ < 0)
            return false;
        size_t w = 0;
        while(w < len) {
            ssize_t n = ::write(fd_, buf + w, len - w);
            if(n > 0) {
                w += (size_t)n;
                continue;
            }
            if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                struct timeval tv{0, 20000};
                fd_set wfds;
                FD_ZERO(&wfds);
                FD_SET(fd_, &wfds);
                (void)select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
                continue;
            }
            return false;
        }
        return true;
    }
    inline long read_some(uint8_t *buf, size_t cap) {
        if(fd_ < 0)
            return -1;
        return (long)::read(fd_, buf, cap);
    }

  private:
    int fd_ = -1;
};
#endif

// ========== POSIX Platform binder for uproto ==========
class PosixPlatform {
  public:
    inline void init_uproto(uproto_context_t *ctx, PosixSerialPort *port) {
        port_ = port;
        uproto_port_ops_t pop{};
        pop.user = this;
        pop.get_mtu = [](void *) -> uint16_t { return 1024; };
        pop.flush = [](void *) {};
        pop.write = [](void *u, const uint8_t *data, uint32_t len) -> uint32_t { auto*self=(PosixPlatform*)u; return (self&&self->port_ && self->port_->write_all(data,len))?len:0u; };
        uproto_time_ops_t to{};
        to.user = nullptr;
        to.now_ms = [](void *) -> uint32_t { struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return (uint32_t)(ts.tv_sec*1000u + ts.tv_nsec/1000000u); };
        uproto_config_t cfg{};
        cfg.handshake_timeout_ms = 0;
        cfg.heartbeat_interval_ms = 0;
        cfg.default_timeout_ms = 3000;
        cfg.default_retries = 3;
        cfg.enable_auto_handshake = false;
        cfg.event_cb = nullptr;
        cfg.event_user = nullptr;
        uproto_init(ctx, &pop, &to, &cfg);
    }
    inline void tick(uproto_context_t *ctx) {
        if(ctx)
            uproto_tick(ctx);
    }

  private:
    PosixSerialPort *port_ = nullptr;
};

// ========== MUX Dispatcher (compile-time mapping) ==========
template <uint8_t ChV, uint16_t SidV, typename Fn>
struct MuxHandler {
    Fn fn;
    static constexpr uint8_t Ch = ChV;
    static constexpr uint16_t Sid = SidV;
    inline bool try_call(uint8_t ch, uint16_t sid, const uint8_t *pl, uint32_t len) const {
        if(ch == Ch && sid == Sid) {
            fn(pl, len);
            return true;
        }
        return false;
    }
};
template <typename... Hs>
class MuxDispatcher {
  public:
    explicit MuxDispatcher(Hs... h) : handlers_(std::move(h)...) {
    }
    inline bool dispatch(uint8_t ch, uint16_t sid, const uint8_t *pl, uint32_t len) const {
        bool handled = false;
        apply([&](auto const &...hs) { int dummy[] = {0, (handled = handled || hs.try_call(ch,sid,pl,len), 0)...}; (void)dummy; });
        return handled;
    }

  private:
    std::tuple<Hs...> handlers_;
    template <typename F>
    inline void apply(F &&f) const {
        apply_impl(std::forward<F>(f), std::index_sequence_for<Hs...>{});
    }
    template <typename F, std::size_t... Is>
    inline void apply_impl(F &&f, std::index_sequence<Is...>) const {
        f(std::get<Is>(handlers_)...);
    }
};

// ========== HostAppT template ==========
template <class SerialPortT, class PlatformT>
class HostAppT {
  public:
    HostAppT() : ts_(TS_CH_ID, 1000u, true, [this](uint8_t ch, uint16_t sid, const uint8_t *p, uint16_t l) { return this->send_mux(ch, sid, p, l); }), dispatcher_(make_dispatcher_()) {
    }
    inline bool init(const std::string &dev, int baud) {
        if(!port_.open(dev, baud)) {
            std::fprintf(stderr, "failed to open %s\n", dev.c_str());
            return false;
        }
        plat_.init_uproto(&up_, &port_);
        (void)uproto_register_handler(&up_, UPROTO_MSG_MUX, &HostAppT::on_uproto_mux, this);
        ts_.set_max_rtt_us(0);
        return true;
    }
    inline int run_loop() {
        std::printf("Listening on fd=%d ...\n", port_.fd());
        for(;;) {
            ts_.tick();
            plat_.tick(&up_);
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(port_.fd(), &rfds);
            struct timeval tv{1, 0};
            int r = select(port_.fd() + 1, &rfds, nullptr, nullptr, &tv);
            if(r > 0 && FD_ISSET(port_.fd(), &rfds)) {
                uint8_t buf[1024];
                long n = port_.read_some(buf, sizeof(buf));
                if(n > 0)
                    uproto_on_rx_bytes(&up_, buf, (uint32_t)n);
            }
        }
        return 0;
    }
    inline int send_mux(uint8_t ch, uint16_t sid, const uint8_t *payload, uint16_t len) {
        static uint32_t seqs[256] = {0};
        uint8_t buf[COMM_MUX_TX_BUFFER_SIZE];
        uint32_t wrote = cpp_mux_encode_build(ch, sid, ++seqs[ch], payload, len, buf, sizeof(buf));
        if(!wrote)
            return -1;
        return (uproto_send_notify(&up_, UPROTO_MSG_MUX, buf, wrote) == UPROTO_OK) ? 0 : -1;
    }

  private:
    static void on_uproto_mux(uproto_context_t *, uint16_t, const uint8_t *data, uint32_t len, void *user) {
        auto *self = (HostAppT *)user;
        if(!self)
            return;
        uint8_t ch = 0;
        const uint8_t *pl = nullptr;
        uint32_t pl_len = 0;
        if(!cpp_mux_decode(data, len, &ch, &pl, &pl_len))
            return;
        self->handle_mux_payload(ch, pl, pl_len);
    }
    inline void handle_mux_payload(uint8_t ch, const uint8_t *pl, uint32_t len) {
        if(!pl || len < 2)
            return;
        uint16_t sid = comm_read_u16_le(pl);
        if(dispatcher_.dispatch(ch, sid, pl, len))
            return;
        if(ch == TS_CH_ID) {
            std::printf("TS CH3 sid=0x%04X len=%u\n", sid, (unsigned)len);
            ts_.on_mux(pl, len);
            return;
        }
        std::printf("CH%u sid=0x%04X len=%u\n", ch, sid, (unsigned)len);
    }
    // Build dispatcher with five handlers
    inline auto make_dispatcher_() {
        using F = std::function<void(const uint8_t *, uint32_t)>;
        using H1 = MuxHandler<CAM_CH_ID, CAM_SID_RESET, F>;
        using H2 = MuxHandler<CAM_CH_ID, CAM_SID_EVENT, F>;
        using H3 = MuxHandler<TS_CH_ID, TS_SID_REQ, F>;
        using H4 = MuxHandler<TS_CH_ID, TS_SID_RESP, F>;
        using H5 = MuxHandler<GIMBAL_CH_ID, GIMBAL_SID_STATE, F>;
        auto h1 = H1{F([&](auto, auto) { std::printf("CAM RESET ACK\n"); })};
        auto h2 = H2{F([&](const uint8_t *pl, uint32_t len) { (void)len; if(len>=2+4+8){ uint32_t frame_id=comm_read_u32_le(pl+2); uint64_t ts_us=comm_read_u64_le(pl+6); struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); uint64_t now=(uint64_t)ts.tv_sec*1000000ULL+(uint64_t)ts.tv_nsec/1000ULL; if(last_cam_print_us_==0||(now-last_cam_print_us_)>=1000000ULL){ std::printf("CAM TRIG id=0x%08X ts=0x%016llX\n", frame_id, (unsigned long long)ts_us); last_cam_print_us_=now; } } })};
        auto h3 = H3{F([&](const uint8_t *pl, uint32_t len) { uint16_t seq=(len>=4)?comm_read_u16_le(pl+2):0; (void)seq; ts_.on_mux(pl,len); std::printf("[TS] REQ seq=%u -> RESP sent\n", (unsigned)seq); })};
        auto h4 = H4{F([&](const uint8_t *pl, uint32_t len) { uint16_t seq=(len>=4)?comm_read_u16_le(pl+2):0; ts_.on_mux(pl,len); uint32_t rtt=ts_.last_rtt_us(); unsigned long long dev_now=(unsigned long long)ts_.now_device_us(); std::printf("[TS] RESP seq=%u rtt=%u us dev_now=0x%016llX\n", (unsigned)seq,(unsigned)rtt,dev_now); })};
        auto h5 = H5{F([&](const uint8_t *pl, uint32_t len) { gimbal_on_state(pl, len); })};
        return MuxDispatcher<decltype(h1), decltype(h2), decltype(h3), decltype(h4), decltype(h5)>(h1, h2, h3, h4, h5);
    }
    SerialPortT port_{};
    PlatformT plat_{};
    uproto_context_t up_{};
    TsHost ts_;
    uint64_t last_cam_print_us_ = 0;
    using Self = HostAppT<SerialPortT, PlatformT>;
    using DispatcherType = decltype(std::declval<Self &>().make_dispatcher_());
    DispatcherType dispatcher_;
};
