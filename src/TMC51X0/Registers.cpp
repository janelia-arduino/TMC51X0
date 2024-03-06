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
  stored_[GCONF] = 0x9;
  writeable_[GCONF] = true;
  readable_[GCONF] = true;

  stored_[GSTAT] = 0x5;
  writeable_[GSTAT] = true;
  readable_[GSTAT] = true;

  readable_[IFCNT] = true;

  writeable_[NODECONF] = true;

  readable_[IOIN] = true;

  writeable_[X_COMPARE] = true;

  writeable_[OTP_PROG] = true;

  readable_[OTP_READ] = true;

  stored_[FACTORY_CONF] = 0xE;
  writeable_[FACTORY_CONF] = true;
  readable_[FACTORY_CONF] = true;

  writeable_[SHORT_CONF] = true;

  writeable_[DRV_CONF] = true;

  writeable_[GLOBAL_SCALER] = true;

  readable_[OFFSET_READ] = true;

  writeable_[IHOLD_IRUN] = true;

  writeable_[TPOWER_DOWN] = true;

  readable_[TSTEP] = true;

  writeable_[TPWMTHRS] = true;

  writeable_[TCOOLTHRS] = true;

  writeable_[THIGH] = true;

  writeable_[RAMPMODE] = true;
  readable_[RAMPMODE] = true;

  writeable_[XACTUAL] = true;
  readable_[XACTUAL] = true;

  readable_[VACTUAL] = true;

  writeable_[VSTART] = true;

  writeable_[A1] = true;

  writeable_[V1] = true;

  writeable_[AMAX] = true;

  writeable_[VMAX] = true;

  writeable_[DMAX] = true;

  writeable_[D1] = true;

  writeable_[VSTOP] = true;

  writeable_[TZEROWAIT] = true;

  writeable_[XTARGET] = true;
  readable_[XTARGET] = true;

  writeable_[VDCMIN] = true;

  writeable_[SW_MODE] = true;
  readable_[SW_MODE] = true;

  stored_[RAMP_STAT] = 0x780;
  writeable_[RAMP_STAT] = true;
  readable_[RAMP_STAT] = true;

  readable_[XLATCH] = true;

  writeable_[ENCMODE] = true;
  readable_[ENCMODE] = true;

  writeable_[X_ENC] = true;
  readable_[X_ENC] = true;

  writeable_[ENC_CONST] = true;

  writeable_[ENC_STATUS] = true;
  readable_[ENC_STATUS] = true;

  readable_[ENC_LATCH] = true;

  writeable_[ENC_DEVIATION] = true;

  stored_[CHOPCONF] = 0x10410150;
  writeable_[CHOPCONF] = true;
  readable_[CHOPCONF] = true;

  interface_.setup(spi, chip_select_pin);
}
