#pragma once

#include <cstdint>

// Forward declare from C core
extern "C" {
    typedef struct uproto_context uproto_context_t;
}

class ISerialPort;

class IPlatform {
public:
    virtual ~IPlatform() = default;
    // Initialize uproto with a serial backend
    virtual void init_uproto(uproto_context_t *ctx, ISerialPort *port) = 0;
    virtual void tick(uproto_context_t *ctx) = 0;
};
