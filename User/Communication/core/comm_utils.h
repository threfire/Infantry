#ifndef CORE_COMM_UTILS_H
#define CORE_COMM_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Approximate π value for float math. */
#define COMM_PI_F 3.14159265358979323846f

static inline uint16_t comm_read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void comm_write_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline uint32_t comm_read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static inline void comm_write_u32_le(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
    p[2] = (uint8_t)((value >> 16) & 0xFFu);
    p[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static inline uint64_t comm_read_u64_le(const uint8_t *p) {
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | p[i];
    }
    return value;
}

static inline void comm_write_u64_le(uint8_t *p, uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        p[i] = (uint8_t)((value >> (8 * i)) & 0xFFu);
    }
}

static inline int32_t comm_read_i32_le(const uint8_t *p) {
    return (int32_t)comm_read_u32_le(p);
}

static inline void comm_write_i32_le(uint8_t *p, int32_t value) {
    comm_write_u32_le(p, (uint32_t)value);
}

static inline float comm_deg_to_rad(float degrees) {
    return degrees * (COMM_PI_F / 180.0f);
}

static inline float comm_rad_to_deg(float radians) {
    return radians * (180.0f / COMM_PI_F);
}

static inline int32_t comm_float_to_fixed(float value, uint8_t frac_bits) {
    const float scale = (float)(1UL << frac_bits);
    return (int32_t)(value * scale);
}

static inline float comm_fixed_to_float(int32_t value, uint8_t frac_bits) {
    const float scale = (float)(1UL << frac_bits);
    return (float)value / scale;
}

#ifdef __cplusplus
}
#endif

#endif /* CORE_COMM_UTILS_H */
