// Minimal SPI compatibility shim for native unit tests.
#ifndef TMC51X0_TEST_NATIVE_SPI_H
#define TMC51X0_TEST_NATIVE_SPI_H

#include <stddef.h>
#include <stdint.h>

#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef SPI_MODE3
#define SPI_MODE3 3
#endif

class SPISettings {
public:
  SPISettings(uint32_t = 1000000, uint8_t = MSBFIRST, uint8_t = SPI_MODE3) {}
};

class SPIClass {
public:
  virtual ~SPIClass() = default;

  virtual uint8_t transfer(uint8_t data) { return data; }

  // In-place buffer transfer, as the Arduino cores provide it. Routed through
  // the byte transfer so a fake that scripts bytes sees every one.
  virtual void transfer(void *buf, size_t count) {
    uint8_t *bytes = static_cast<uint8_t *>(buf);
    for (size_t i = 0; i < count; ++i) {
      bytes[i] = transfer(bytes[i]);
    }
  }

  virtual void beginTransaction(const SPISettings &) {}

  virtual void endTransaction() {}
};

#endif
