#pragma once

#if defined(_WIN32)

#include "transport.hpp"
#include <cstdlib>
#include <string>
#include <windows.h>

class SerialTransportWin32 : public IoTransport<SerialTransportWin32> {
  public:
    inline SerialTransportWin32() = default;
    inline ~SerialTransportWin32() { close_impl(); }

    inline bool open_impl(const std::string &endpoint) {
        std::string path = endpoint;
        int baud = 115200;
        auto prefix = std::string("serial:");
        if(path.rfind(prefix, 0) == 0) {
            std::string trimmed = path.substr(prefix.size());
            auto q = trimmed.find('?');
            std::string port_part = (q == std::string::npos) ? trimmed : trimmed.substr(0, q);
            if(!port_part.empty())
                path = port_part;
            if(q != std::string::npos) {
                std::string query = trimmed.substr(q + 1);
                auto bpos = query.find("baud=");
                if(bpos != std::string::npos)
                    baud = std::atoi(query.c_str() + bpos + 5);
            }
        }
        std::string port = path;
        if(port.rfind("\\\\.", 0) != 0 && port.rfind("COM", 0) == 0) {
            port = std::string("\\\\.\\") + port;
        }
        close_impl();
        HANDLE h = CreateFileA(port.c_str(),
                               GENERIC_READ | GENERIC_WRITE,
                               0,
                               nullptr,
                               OPEN_EXISTING,
                               FILE_ATTRIBUTE_NORMAL,
                               nullptr);
        if(h == INVALID_HANDLE_VALUE)
            return false;
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if(!GetCommState(h, &dcb)) {
            CloseHandle(h);
            return false;
        }
        dcb.BaudRate = static_cast<DWORD>(baud);
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        if(!SetCommState(h, &dcb)) {
            CloseHandle(h);
            return false;
        }
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = 0;
        to.ReadTotalTimeoutConstant = 10;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant = 10;
        SetCommTimeouts(h, &to);
        handle_ = h;
        return true;
    }

    inline void close_impl() {
        if(handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }

    inline long write_some_impl(const uint8_t *buf, size_t len) {
        if(handle_ == INVALID_HANDLE_VALUE || len == 0)
            return 0;
        DWORD written = 0;
        if(!WriteFile(handle_, buf, (DWORD)len, &written, nullptr))
            return -1;
        return static_cast<long>(written);
    }

    inline long read_some_impl(uint8_t *buf, size_t cap) {
        if(handle_ == INVALID_HANDLE_VALUE || cap == 0)
            return 0;
        DWORD read = 0;
        if(!ReadFile(handle_, buf, (DWORD)cap, &read, nullptr)) {
            DWORD err = GetLastError();
            if(err == ERROR_IO_PENDING)
                return 0;
            return -1;
        }
        return static_cast<long>(read);
    }

    inline int fd_impl() const {
        return -1;
    }

    inline uint16_t mtu_impl() const {
        return 1024;
    }

  private:
    HANDLE handle_ = INVALID_HANDLE_VALUE;
};

#endif // _WIN32
