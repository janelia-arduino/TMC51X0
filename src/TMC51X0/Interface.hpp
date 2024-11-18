// ----------------------------------------------------------------------------
// Interface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_INTERFACE_HPP
#define TMC51X0_INTERFACE_HPP


namespace tmc51x0
{
class Interface
{
public:
  enum InterfaceMode
  {
    SPI,
    UART
  };
  InterfaceMode interface_mode;

  virtual void writeRegister(uint8_t /* register_address */,
    uint32_t /* data */){};
  virtual uint32_t readRegister(uint8_t /* register_address */){return 0;};
};
}
#endif
