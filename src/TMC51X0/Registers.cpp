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
  if ((register_address < AddressCount) && (writeable_[register_address]))
  {
    interface_ptr_->writeRegister(register_address, data);
    stored_[register_address] = data;
  }
}

uint32_t Registers::read(RegisterAddress register_address)
{
  if ((register_address < AddressCount) && (readable_[register_address]))
  {
    uint32_t data = interface_ptr_->readRegister(register_address);
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
  if (register_address < AddressCount)
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
  if (register_address < AddressCount)
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
  if (register_address < AddressCount)
  {
    return readable_[register_address];
  }
  else
  {
    return false;
  }
}


Registers::Gstat Registers::readAndClearGstat()
{
  Gstat gstat_read, gstat_write;
  gstat_read.bytes = read(tmc51x0::Registers::GstatAddress);
  gstat_write.reset = 1;
  gstat_write.drv_err = 1;
  gstat_write.uv_cp = 1;
  write(GstatAddress, gstat_write.bytes);
  return gstat_read;
}

// private
void Registers::initialize(Interface & interface)
{
  interface_ptr_ = &interface;

  for (uint8_t register_address=0; register_address<AddressCount; ++register_address)
  {
    stored_[register_address] = 0;
    writeable_[register_address] = false;
    readable_[register_address] = false;
  }

  stored_[GconfAddress] = 0x0;
  writeable_[GconfAddress] = true;
  readable_[GconfAddress] = true;

  stored_[GstatAddress] = 0x5;
  writeable_[GstatAddress] = true;
  readable_[GstatAddress] = true;

  readable_[IfcntAddress] = true;

  writeable_[NodeconfAddress] = true;

  readable_[IoinAddress] = true;

  writeable_[XCompareAddress] = true;

  writeable_[OtpProgAddress] = true;

  readable_[OtpReadAddress] = true;

  stored_[FactoryConfAddress] = 0xE;
  writeable_[FactoryConfAddress] = true;
  readable_[FactoryConfAddress] = true;

  writeable_[ShortConfAddress] = true;

  writeable_[DrvConfAddress] = true;

  writeable_[GlobalScalerAddress] = true;

  readable_[OffsetReadAddress] = true;

  writeable_[IholdIrunAddress] = true;

  writeable_[TpowerDownAddress] = true;

  readable_[TstepAddress] = true;

  writeable_[TpwmthrsAddress] = true;

  writeable_[TcoolthrsAddress] = true;

  writeable_[ThighAddress] = true;

  writeable_[RampmodeAddress] = true;
  readable_[RampmodeAddress] = true;

  writeable_[XactualAddress] = true;
  // Refer to datasheet "Errata in Read Access"
  if (interface_ptr_->interface_mode == Interface::SpiMode)
  {
    readable_[XactualAddress] = true;
  }

  // Refer to datasheet "Errata in Read Access"
  if (interface_ptr_->interface_mode == Interface::SpiMode)
  {
    readable_[VactualAddress] = true;
  }

  writeable_[VstartAddress] = true;

  writeable_[Acceleration1Address] = true;

  writeable_[Velocity1Address] = true;

  writeable_[AmaxAddress] = true;

  writeable_[VmaxAddress] = true;

  writeable_[DmaxAddress] = true;

  writeable_[Deceleration1Address] = true;

  writeable_[VstopAddress] = true;

  writeable_[TzerowaitAddress] = true;

  writeable_[XtargetAddress] = true;
  readable_[XtargetAddress] = true;

  writeable_[VdcminAddress] = true;

  writeable_[SwModeAddress] = true;
  readable_[SwModeAddress] = true;

  stored_[RampStatAddress] = 0x780;
  writeable_[RampStatAddress] = true;
  readable_[RampStatAddress] = true;

  readable_[XlatchAddress] = true;

  writeable_[EncmodeAddress] = true;
  readable_[EncmodeAddress] = true;

  writeable_[XencAddress] = true;
  // Refer to datasheet "Errata in Read Access"
  if (interface_ptr_->interface_mode == Interface::SpiMode)
  {
    readable_[XencAddress] = true;
  }

  writeable_[EncConstAddress] = true;

  writeable_[EncStatusAddress] = true;
  readable_[EncStatusAddress] = true;

  readable_[EncLatchAddress] = true;

  writeable_[EncDeviationAddress] = true;

  writeable_[Mslut0Address] = true;

  writeable_[Mslut1Address] = true;

  writeable_[Mslut2Address] = true;

  writeable_[Mslut3Address] = true;

  writeable_[Mslut4Address] = true;

  writeable_[Mslut5Address] = true;

  writeable_[Mslut6Address] = true;

  writeable_[Mslut7Address] = true;

  writeable_[MslutselAddress] = true;

  writeable_[MSLUTSTART] = true;

  // Refer to datasheet "Errata in Read Access"
  if (interface_ptr_->interface_mode == Interface::SpiMode)
  {
    readable_[MscntAddress] = true;
  }

  readable_[MscuractAddress] = true;

  stored_[ChopconfAddress] = 0x10410150;
  writeable_[ChopconfAddress] = true;
  readable_[ChopconfAddress] = true;

  writeable_[CoolconfAddress] = true;

  writeable_[DcctrlAddress] = true;

  readable_[DrvStatusAddress] = true;

  stored_[PwmconfAddress] = 0xC40C001E;
  writeable_[PwmconfAddress] = true;

  readable_[PwmScaleAddress] = true;

  readable_[PwmAutoAddress] = true;

  readable_[LostStepsAddress] = true;
}
