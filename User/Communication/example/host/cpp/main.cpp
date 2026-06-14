#define HOST_PLATFORM_SHIM_IMPL
#include "platform_shim.hpp"
#include "host_generic.hpp"
#if defined(_WIN32)
#include "hal/serial_transport_win32.hpp"
#include <windows.h>
#else
#include "hal/serial_transport_posix.hpp"
#endif
#include <cstdio>
#include <cstring>
#include <string>

static void print_usage() {
    std::printf("Usage:\n");
#if defined(_WIN32)
    std::printf("  uproto_host_cpp [COMx | serial:COMx[?baud=NNN]]\n");
    std::printf("  uproto_host_cpp --list   # list COM ports\n");
#else
    std::printf("  uproto_host_cpp [/dev/ttyXXX | serial:/dev/ttyXXX[?baud=NNN]]\n");
#endif
    std::printf("Examples:\n");
#if defined(_WIN32)
    std::printf("  uproto_host_cpp COM3\n  uproto_host_cpp serial:COM5?baud=115200\n");
#else
    std::printf("  uproto_host_cpp /dev/ttyACM0\n  uproto_host_cpp serial:/dev/ttyUSB0?baud=921600\n");
#endif
}

#if defined(_WIN32)
static void list_com_ports() {
    char buf[65536];
    DWORD n = QueryDosDeviceA(nullptr, buf, sizeof(buf));
    if(!n) { std::printf("(no COM ports found)\n"); return; }
    std::printf("Available COM ports:\n");
    const char *p = buf;
    while(*p) {
        if(std::strncmp(p, "COM", 3) == 0) {
            std::printf("  %s\n", p);
        }
        p += std::strlen(p) + 1;
    }
}
#endif

int main(int argc, char **argv) {
#if defined(_WIN32)
    std::string dev = "COM3";
#else
    std::string dev = "/dev/ttyACM0";
#endif

    bool flag_quiet = false;
    const char *sine_opt = nullptr; // axis:freq:amp[@rate]
    std::string endpoint_arg;
    for(int i=1;i<argc;i++){
        std::string arg = argv[i];
        if(arg == "--help" || arg == "-h" || arg == "/?") { print_usage(); return 0; }
#if defined(_WIN32)
        if(arg == "--list") { list_com_ports(); return 0; }
#endif
        if(arg == "--quiet") { flag_quiet = true; continue; }
        if(arg == "--sine" && (i+1) < argc) { sine_opt = argv[++i]; continue; }
        if(endpoint_arg.empty()) endpoint_arg = arg; // first non-flag is endpoint
    }
    if(!endpoint_arg.empty()) dev = endpoint_arg;

    std::string endpoint;
    if(dev.rfind("serial:", 0) == 0) {
        endpoint = dev; // already full
    } else if(dev.find('?') != std::string::npos) {
        endpoint = std::string("serial:") + dev; // user provided query
    } else {
        endpoint = std::string("serial:") + dev + "?baud=115200";
    }

#if defined(_WIN32)
    using App = HostAppGT<SerialTransportWin32, GenericPlatform<SerialTransportWin32>>;
#else
    using App = HostAppGT<SerialTransportPosix, GenericPlatform<SerialTransportPosix>>;
#endif
    App app;
    std::printf("Opening %s ...\n", endpoint.c_str());
    if (!app.init(endpoint)) return 1;
    if(flag_quiet) { app.quiet(); std::printf("[AUTO] quiet mode: rx/tx logs off\n"); }
    if(sine_opt){
        // parse axis:freq:amp[@rate]
        std::string s = sine_opt;
        std::string axis; double freq=0, amp=0; uint32_t rate=50;
        size_t p1 = s.find(':'); size_t p2 = s.find(':', p1==std::string::npos?0:p1+1);
        if(p1!=std::string::npos && p2!=std::string::npos){
            axis = s.substr(0,p1);
            freq = std::atof(s.substr(p1+1, p2-(p1+1)).c_str());
            std::string tail = s.substr(p2+1);
            size_t at = tail.find('@');
            if(at==std::string::npos){ amp = std::atof(tail.c_str()); }
            else { amp = std::atof(tail.substr(0,at).c_str()); rate = (uint32_t)std::atoi(tail.substr(at+1).c_str()); }
            app.start_sine(axis.c_str(), freq, amp, rate);
            std::printf("[AUTO] sine axis=%s freq=%.3fHz amp=%.3fdeg rate=%uHz\n", axis.c_str(), freq, amp, (unsigned)rate);
        } else {
            std::printf("[AUTO] --sine format invalid, expected axis:freq:amp[@rate]\n");
        }
    }
    return app.run_loop();
}
