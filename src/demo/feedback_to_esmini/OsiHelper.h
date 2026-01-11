#pragma once

#include <cstdint>

// Simple helper to decode OSMP pointer format
// OSMP encodes a pointer as two 32-bit integers (lo and hi parts) plus size
inline void* DecodeOSMPPointer(int32_t lo, int32_t hi) {
    // Combine lo and hi to form 64-bit pointer
    uint64_t ptr_value = (static_cast<uint64_t>(static_cast<uint32_t>(hi)) << 32) | 
                         static_cast<uint64_t>(static_cast<uint32_t>(lo));
    return reinterpret_cast<void*>(ptr_value);
}

inline void EncodeOSMPPointer(void* ptr, int32_t& lo, int32_t& hi) {
    uint64_t val = reinterpret_cast<uint64_t>(ptr);
    lo = static_cast<int32_t>(val & 0xFFFFFFFF);
    hi = static_cast<int32_t>((val >> 32) & 0xFFFFFFFF);
}
