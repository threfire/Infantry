// POSIX serial transport implementing IoTransport CRTP
#pragma once

#if !defined(_WIN32)

#include "transport.hpp"
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

class SerialTransportPosix : public IoTransport<SerialTransportPosix> {
public:
    inline SerialTransportPosix() = default;
    inline ~SerialTransportPosix() { close_impl(); }

    inline bool open_impl(const std::string &endpoint) {
        // endpoint format: serial:/dev/ttyACM0?baud=115200 or plain path
        std::string path = endpoint;
        int baud = 115200;
        auto pos = endpoint.find("serial:");
        if (pos == 0) {
            auto q = endpoint.find('?');
            path = endpoint.substr(7, q == std::string::npos ? std::string::npos : q - 7);
            if (q != std::string::npos) {
                auto bpos = endpoint.find("baud=", q);
                if (bpos != std::string::npos) baud = std::atoi(endpoint.c_str() + bpos + 5);
            }
        }
        close_impl();
        int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;
        struct termios tio;
        if (tcgetattr(fd, &tio) != 0) { ::close(fd); return false; }
        tio.c_iflag = 0; tio.c_oflag = 0; tio.c_lflag = 0;
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        speed_t sp = map_baud_(baud);
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
#ifdef CRTSCTS
        tio.c_cflag &= ~CRTSCTS;
#endif
        tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 1;
        if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return false; }
        fd_ = fd; return true;
    }

    inline void close_impl() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    inline long write_some_impl(const uint8_t *buf, size_t len) { return (long)::write(fd_, buf, len); }
    inline long read_some_impl(uint8_t *buf, size_t cap) { return (long)::read(fd_, buf, cap); }
    inline int fd_impl() const { return fd_; }
    inline uint16_t mtu_impl() const { return 1024; }

private:
    static inline speed_t map_baud_(int baud){ switch(baud){ case 9600:return B9600; case 19200:return B19200; case 38400:return B38400; case 57600:return B57600; case 115200: default:return B115200; } }
    int fd_ = -1;
};

#endif // ! _WIN32

