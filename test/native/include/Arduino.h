// Minimal Arduino compatibility shim for native unit tests.
#ifndef TMC51X0_TEST_NATIVE_ARDUINO_H
#define TMC51X0_TEST_NATIVE_ARDUINO_H

#include <stddef.h>
#include <stdint.h>

#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW 0x0
#endif
#ifndef OUTPUT
#define OUTPUT 0x1
#endif

inline void
pinMode (size_t, uint8_t)
{
}
inline void
digitalWrite (size_t, uint8_t)
{
}

#endif
