// ----------------------------------------------------------------------------
// Interface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_INTERFACE_HPP
#define TMC51X0_INTERFACE_HPP

#include <stdint.h>

#include "Result.hpp"

namespace tmc51x0
{
class Interface
{
public:
  virtual ~Interface () = default;

  enum InterfaceMode
  {
    SpiMode,
    UartMode
  };
  InterfaceMode interface_mode{ SpiMode };

  virtual void writeRegister (uint8_t /* register_address */,
                              uint32_t /* data */) {};
  virtual uint32_t
  readRegister (uint8_t /* register_address */)
  {
    return 0;
  };

  // Explicit-error variants. The default implementation preserves the legacy
  // behavior for transports like SPI that do not currently surface a richer
  // error model.
  virtual Result<void>
  writeRegisterResult (uint8_t register_address,
                       uint32_t data)
  {
    writeRegister (register_address, data);
    return Result<void>{};
  }

  virtual Result<uint32_t>
  readRegisterResult (uint8_t register_address)
  {
    Result<uint32_t> r;
    r.value = readRegister (register_address);
    return r;
  }
};
}
#endif
