// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"


using namespace tmc51x0;

void Registers::write(RegisterAddress register_address,
  uint32_t data)
{
  if ((register_address < ADDRESS_COUNT) && (writeable_[register_address]))
  {
    interface_.writeRegister(register_address, data);
    stored_[register_address] = data;
  }
}

uint32_t Registers::read(RegisterAddress register_address)
{
  if ((register_address < ADDRESS_COUNT) && (readable_[register_address]))
  {
    uint32_t data = interface_.readRegister(register_address);
    stored_[register_address] = data;
    return data;
  }
  else
  {
    return 0;
  }
}

uint32_t Registers::getStored(RegisterAddress register_address)
{
  if (register_address < ADDRESS_COUNT)
  {
    return stored_[register_address];
  }
  else
  {
    return 0;
  }
}

bool Registers::writeable(RegisterAddress register_address)
{
  if (register_address < ADDRESS_COUNT)
  {
    return writeable_[register_address];
  }
  else
  {
    return false;
  }
}

bool Registers::readable(RegisterAddress register_address)
{
  if (register_address < ADDRESS_COUNT)
  {
    return readable_[register_address];
  }
  else
  {
    return false;
  }
}

// private
void Registers::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  stored_[GLOBAL_CONFIG] = 0x9;
  writeable_[GLOBAL_CONFIG] = true;
  readable_[GLOBAL_CONFIG] = true;

  stored_[GSTAT] = 0x5;
  writeable_[GSTAT] = true;
  readable_[GSTAT] = true;

  readable_[IFCNT] = true;

  writeable_[NODECONF] = true;

  readable_[IOIN] = true;

  stored_[CHOPPER_CONFIG] = 0x10410150;
  writeable_[CHOPPER_CONFIG] = true;
  readable_[CHOPPER_CONFIG] = true;

  interface_.setup(spi, chip_select_pin);
}
