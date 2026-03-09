// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"

using namespace tmc51x0;

void
Registers::write (RegisterAddress register_address,
                  uint32_t data)
{
  if ((register_address < AddressCount)
      && (writeable_[register_address])
      && (interface_ptr_ != nullptr))
    {
      Result<void> result = interface_ptr_->writeRegisterResult (register_address,
                                                                 data);
      if (result.ok ())
        {
          // Only advance the software mirror after an explicit successful
          // transport operation.
          stored_[register_address] = data;
          stored_valid_[register_address] = true;
        }
    }
}

uint32_t
Registers::read (RegisterAddress register_address)
{
  if ((register_address < AddressCount)
      && (readable_[register_address])
      && (interface_ptr_ != nullptr))
    {
      Result<uint32_t> result = interface_ptr_->readRegisterResult (register_address);
      if (result.ok ())
        {
          // Keep the last-known-good mirror intact if the transport reports an
          // error instead of poisoning it with an implicit fallback value.
          stored_[register_address] = result.value;
          stored_valid_[register_address] = true;
          return result.value;
        }
    }

  return 0;
}

uint32_t
Registers::getStored (RegisterAddress register_address)
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

bool
Registers::storedValid (RegisterAddress register_address) const
{
  if (register_address < AddressCount)
    {
      return stored_valid_[register_address];
    }
  else
    {
      return false;
    }
}

void
Registers::assumeDeviceReset ()
{
  seedStoredResetValues_ ();
}

bool
Registers::writeable (RegisterAddress register_address)
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

bool
Registers::readable (RegisterAddress register_address)
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

Registers::Gstat
Registers::readAndClearGstat ()
{
  Gstat gstat_read, gstat_write;
  gstat_read.raw = read (tmc51x0::Registers::GstatAddress);
  if (gstat_read.reset ())
    {
      assumeDeviceReset ();
    }
  gstat_write.reset (true);
  gstat_write.drv_err (true);
  gstat_write.uv_cp (true);
  write (GstatAddress, gstat_write.raw);
  return gstat_read;
}

void
Registers::seedStoredResetValues_ ()
{
  for (uint8_t register_address = 0; register_address < AddressCount; ++register_address)
    {
      stored_[register_address] = 0;
      stored_valid_[register_address] = false;
    }

  stored_[GconfAddress] = 0x0;
  stored_valid_[GconfAddress] = true;

  stored_[GstatAddress] = 0x5;
  stored_valid_[GstatAddress] = true;

  stored_[FactoryConfAddress] = 0xE;
  stored_valid_[FactoryConfAddress] = true;

  stored_[RampStatAddress] = 0x780;
  stored_valid_[RampStatAddress] = true;

  stored_[ChopconfAddress] = 0x10410150;
  stored_valid_[ChopconfAddress] = true;

  stored_[PwmconfAddress] = 0xC40C001E;
  stored_valid_[PwmconfAddress] = true;
}

// private
void
Registers::initialize (Interface &interface)
{
  interface_ptr_ = &interface;

  for (uint8_t register_address = 0; register_address < AddressCount; ++register_address)
    {
      writeable_[register_address] = false;
      readable_[register_address] = false;
    }

  writeable_[GconfAddress] = true;
  readable_[GconfAddress] = true;

  writeable_[GstatAddress] = true;
  readable_[GstatAddress] = true;

  readable_[IfcntAddress] = true;

  writeable_[NodeconfAddress] = true;

  readable_[IoinAddress] = true;

  writeable_[XCompareAddress] = true;

  writeable_[OtpProgAddress] = true;

  readable_[OtpReadAddress] = true;

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

  writeable_[ChopconfAddress] = true;
  readable_[ChopconfAddress] = true;

  writeable_[CoolconfAddress] = true;

  writeable_[DcctrlAddress] = true;

  readable_[DrvStatusAddress] = true;

  writeable_[PwmconfAddress] = true;

  readable_[PwmScaleAddress] = true;

  readable_[PwmAutoAddress] = true;

  readable_[LostStepsAddress] = true;

  seedStoredResetValues_ ();
}
