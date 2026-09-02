// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"

using namespace tmc51x0;

namespace {
// CHOPCONF's reset default is not the same on both parts, and the difference is
// not cosmetic: bits 23:20 are TPFD on the TMC5160 (mid-range resonance
// dampening, reset default 4) and SYNC on the TMC5130 (chopSync, where a
// non-zero value ENABLES synchronisation at fCLK/(SYNC*64)). Seeding the
// TMC5160 value for a TMC5130 therefore turned chopSync on by accident,
// because every high-level CHOPCONF write is a read-modify-write of this
// mirror.
//
// Unknown maps to the TMC5160 value, matching pwmconfResetDefault, because
// setupSpi() seeds before it identifies the part: setDeviceModel() re-seeds
// both registers once the identity read lands, which is what actually closes
// this for a caller that did not declare the model.
//
// The TMC5160 datasheet rev 1.18 gives 0x10410150. The TMC5130 datasheet gives
// no reset default for CHOPCONF, and a TMC5130A read 0x00000000 from a cold
// supply on bench hardware -- so the value below is not a claim about silicon.
// It keeps the chopper defaults this library has always applied (TOFF 0,
// HSTRT 5, HEND 2, TBL 2, CHM 0, MRES 0, intpol 1) and clears only the field
// whose meaning differs between the parts.
uint32_t chopconfResetDefault(Registers::DeviceModel device_model) {
  switch (device_model) {
  case Registers::DeviceModel::TMC5130A:
    return 0x10010150UL;
  case Registers::DeviceModel::TMC5160A:
  case Registers::DeviceModel::Unknown:
  default:
    return 0x10410150UL;
  }
}

uint32_t pwmconfResetDefault(Registers::DeviceModel device_model) {
  switch (device_model) {
  case Registers::DeviceModel::TMC5130A:
    return 0x00050480UL;
  case Registers::DeviceModel::TMC5160A:
  case Registers::DeviceModel::Unknown:
  default:
    return 0xC40C001EUL;
  }
}
} // namespace

void Registers::setDeviceModel(DeviceModel device_model) {
  if (device_model == device_model_) {
    return;
  }
  device_model_ = device_model;

  // CHOPCONF and PWMCONF have model-dependent reset defaults, and the model is
  // often only learned from the identity read *after* the mirror was seeded --
  // setupSpi() may be called with DeviceModel::Unknown. Re-seed those two, but
  // only while they still hold a seeded default: anything read from the chip or
  // written by the caller is better information than a reset default and must
  // not be overwritten here.
  if (stored_confidence_[ChopconfAddress] == MirrorConfidence::ResetDefault) {
    seedStoredResetValue_(ChopconfAddress, chopconfResetDefault(device_model_));
  }
  if (stored_confidence_[PwmconfAddress] == MirrorConfidence::ResetDefault) {
    seedStoredResetValue_(PwmconfAddress, pwmconfResetDefault(device_model_));
  }
}

Registers::DeviceModel Registers::deviceModel() const { return device_model_; }

bool Registers::deviceModelKnown() const {
  return device_model_ != DeviceModel::Unknown;
}

void Registers::write(RegisterAddress register_address, uint32_t data) {
  if ((register_address < AddressCount) && (writeable_[register_address]) &&
      (interface_ptr_ != nullptr)) {
    Result<void> result =
        interface_ptr_->writeRegisterResult(register_address, data);
    recordTransportReset_();
    if (result.ok()) {
      // Only advance the software mirror after an explicit successful
      // transport operation.
      stored_[register_address] = data;
      stored_valid_[register_address] = true;
      stored_confidence_[register_address] = MirrorConfidence::AssumedWritten;
    }
  }
}

uint32_t Registers::read(RegisterAddress register_address) {
  if ((register_address < AddressCount) && (readable_[register_address]) &&
      (interface_ptr_ != nullptr)) {
    Result<uint32_t> result =
        interface_ptr_->readRegisterResult(register_address);
    recordTransportReset_();
    if (result.ok()) {
      // Keep the last-known-good mirror intact if the transport reports an
      // error instead of poisoning it with an implicit fallback value.
      stored_[register_address] = result.value;
      stored_valid_[register_address] = true;
      stored_confidence_[register_address] = MirrorConfidence::ReadVerified;
      return result.value;
    }
  }

  return 0;
}

bool Registers::refresh(RegisterAddress register_address) {
  if ((register_address < AddressCount) && (readable_[register_address]) &&
      (interface_ptr_ != nullptr)) {
    Result<uint32_t> result =
        interface_ptr_->readRegisterResult(register_address);
    recordTransportReset_();
    if (result.ok()) {
      stored_[register_address] = result.value;
      stored_valid_[register_address] = true;
      stored_confidence_[register_address] = MirrorConfidence::ReadVerified;
      return true;
    }
  }

  return false;
}

bool Registers::resyncReadableConfiguration() {
  const RegisterAddress addresses[] = {
      GconfAddress,  FactoryConfAddress, RampmodeAddress,
      SwModeAddress, EncmodeAddress,     ChopconfAddress,
  };

  bool ok = true;
  for (size_t i = 0; i < (sizeof(addresses) / sizeof(addresses[0])); ++i) {
    ok = refresh(addresses[i]) && ok;
  }
  return ok;
}

uint32_t Registers::getStored(RegisterAddress register_address) {
  if (register_address < AddressCount) {
    return stored_[register_address];
  } else {
    return 0;
  }
}

bool Registers::storedValid(RegisterAddress register_address) const {
  if (register_address < AddressCount) {
    return stored_valid_[register_address];
  } else {
    return false;
  }
}

Registers::MirrorConfidence
Registers::storedConfidence(RegisterAddress register_address) const {
  if (register_address < AddressCount) {
    return stored_confidence_[register_address];
  } else {
    return MirrorConfidence::Unknown;
  }
}

void Registers::assumeDeviceReset() { seedStoredResetValues_(); }

void Registers::notePossibleDrift() { resync_required_ = true; }

bool Registers::resyncRequired() const { return resync_required_; }

void Registers::clearResyncRequired() { resync_required_ = false; }

bool Registers::writeable(RegisterAddress register_address) {
  if (register_address < AddressCount) {
    return writeable_[register_address];
  } else {
    return false;
  }
}

bool Registers::readable(RegisterAddress register_address) {
  if (register_address < AddressCount) {
    return readable_[register_address];
  } else {
    return false;
  }
}

Registers::Gstat Registers::readAndClearGstat() {
  Gstat gstat_read, gstat_write;
  gstat_read.raw = read(tmc51x0::Registers::GstatAddress);
  if (gstat_read.reset()) {
    if (!resync_required_) {
      assumeDeviceReset();
    }
    resync_required_ = true;
  }
  gstat_write.reset(true);
  gstat_write.drv_err(true);
  gstat_write.uv_cp(true);
  write(GstatAddress, gstat_write.raw);
  return gstat_read;
}

void Registers::seedStoredResetValue_(RegisterAddress register_address,
                                      uint32_t data) {
  stored_[register_address] = data;
  stored_valid_[register_address] = true;
  stored_confidence_[register_address] = MirrorConfidence::ResetDefault;
}

void Registers::recordTransportReset_() {
  if ((interface_ptr_ != nullptr) &&
      interface_ptr_->consumeDeviceResetObserved()) {
    if (!resync_required_) {
      assumeDeviceReset();
    }
    resync_required_ = true;
  }
}

void Registers::seedStoredResetValues_() {
  for (uint8_t register_address = 0; register_address < AddressCount;
       ++register_address) {
    stored_[register_address] = 0;
    stored_valid_[register_address] = false;
    stored_confidence_[register_address] = MirrorConfidence::Unknown;
  }

  seedStoredResetValue_(GconfAddress, 0x0UL);
  seedStoredResetValue_(GstatAddress, 0x5UL);
  seedStoredResetValue_(FactoryConfAddress, 0xEUL);
  seedStoredResetValue_(RampStatAddress, 0x780UL);
  seedStoredResetValue_(ChopconfAddress, chopconfResetDefault(device_model_));
  seedStoredResetValue_(PwmconfAddress, pwmconfResetDefault(device_model_));
}

// private
void Registers::initialize(Interface &interface) {
  interface_ptr_ = &interface;
  resync_required_ = false;

  for (uint8_t register_address = 0; register_address < AddressCount;
       ++register_address) {
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
  if (interface_ptr_->interface_mode == Interface::SpiMode) {
    readable_[XactualAddress] = true;
  }

  // Refer to datasheet "Errata in Read Access"
  if (interface_ptr_->interface_mode == Interface::SpiMode) {
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
  if (interface_ptr_->interface_mode == Interface::SpiMode) {
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
  if (interface_ptr_->interface_mode == Interface::SpiMode) {
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

  seedStoredResetValues_();
}
