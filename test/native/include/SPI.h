// Minimal SPI compatibility shim for native unit tests.
#ifndef TMC51X0_TEST_NATIVE_SPI_H
#define TMC51X0_TEST_NATIVE_SPI_H

#include <stdint.h>

#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef SPI_MODE3
#define SPI_MODE3 3
#endif

class SPISettings
{
public:
  SPISettings (uint32_t = 1000000,
               uint8_t = MSBFIRST,
               uint8_t = SPI_MODE3)
  {
  }
};

class SPIClass
{
public:
  virtual ~SPIClass () = default;

  virtual uint8_t
  transfer (uint8_t data)
  {
    return data;
  }

  virtual void
  beginTransaction (const SPISettings &)
  {
  }

  virtual void
  endTransaction ()
  {
  }
};

#endif
