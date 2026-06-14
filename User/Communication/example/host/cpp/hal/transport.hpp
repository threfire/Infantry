// Generic transport HAL using CRTP. Header-only.
#pragma once

#include <cstdint>
#include <string>
#if defined(_WIN32)
#include <winsock2.h>
#else
#include <sys/select.h>
#include <time.h>
#endif

template <class Derived>
class IoTransport {
  public:
    // Open a transport endpoint (format depends on Derived)
    inline bool open(const std::string &endpoint) {
        return self().open_impl(endpoint);
    }
    inline void close() {
        self().close_impl();
    }

    // Write all bytes; Derived should provide write_some_impl()
    inline bool write_all(const uint8_t *buf, size_t len) {
        size_t written = 0;
        while(written < len) {
            long n = self().write_some_impl(buf + written, len - written);
            if(n > 0) {
                written += (size_t)n;
                continue;
            }
            if(n == 0)
                break; // shouldn't happen
            // n < 0: EAGAIN-like 鈥?wait for writable if fd valid, else small sleep
            if(fd() >= 0) {
                struct timeval tv{0, 20000};
                fd_set wfds;
                FD_ZERO(&wfds);
                FD_SET(fd(), &wfds);
                (void)select(fd() + 1, nullptr, &wfds, nullptr, &tv);
                continue;
            } else {
                
#if defined(_WIN32)
                Sleep(20);
#else
                struct timespec ts{0, 20 * 1000 * 1000};
                nanosleep(&ts, nullptr);
#endif
                continue;
            }
        }
        return written == len;
    }

    // Read some bytes (non-blocking or timeout handled by caller)
    inline long read_some(uint8_t *buf, size_t cap) {
        return self().read_some_impl(buf, cap);
    }

    // Wait for readable up to timeout_ms. Default uses fd() with select; return >0 readable, 0 timeout, <0 error.
    inline int wait_readable(int timeout_ms) {
#if defined(_WIN32)
        (void)timeout_ms;
        Sleep(timeout_ms);
        return 0;
#else
        if(fd() < 0) {
            struct timespec ts{timeout_ms / 1000, (long)((timeout_ms % 1000) * 1000000ul)};
            nanosleep(&ts, nullptr);
            return 0;
        }
        struct timeval tv{timeout_ms / 1000, (suseconds_t)((timeout_ms % 1000) * 1000)};
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd(), &rfds);
        return select(fd() + 1, &rfds, nullptr, nullptr, &tv);
    #endif
    }

    // Optional: underlying fd for POSIX transports
    inline int fd() const {
        return self().fd_impl();
    }
    inline uint16_t mtu() const {
        return self().mtu_impl();
    }

  private:
    inline Derived &self() {
        return static_cast<Derived &>(*this);
    }
    inline const Derived &self() const {
        return static_cast<const Derived &>(*this);
    }
};
