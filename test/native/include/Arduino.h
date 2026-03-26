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

#ifndef DEC
#define DEC 10
#endif
#ifndef BIN
#define BIN 2
#endif
#ifndef HEX
#define HEX 16
#endif

class Print {
public:
  virtual ~Print() = default;

  size_t print(const char*) {
    return 0;
  }

  size_t print(uint32_t, int = DEC) {
    return 0;
  }

  size_t print(int32_t, int = DEC) {
    return 0;
  }

  size_t println(const char* = "") {
    return 0;
  }

  size_t println(uint32_t, int = DEC) {
    return 0;
  }

  size_t println(int32_t, int = DEC) {
    return 0;
  }
};

class Stream : public Print {
public:
  virtual int available() {
    return 0;
  }

  virtual int read() {
    return -1;
  }

  virtual size_t write(uint8_t) {
    return 1;
  }

  virtual void flush() {}
};

inline Stream Serial;

inline void pinMode(size_t, uint8_t) {}
inline void digitalWrite(size_t, uint8_t) {}

inline void noInterrupts() {}

inline void interrupts() {}

inline unsigned long micros() {
  return 0;
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline void delayMicroseconds(unsigned int) {}

#endif
