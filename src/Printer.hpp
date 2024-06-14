// ----------------------------------------------------------------------------
// Printer.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_PRINTER_HPP
#define TMC51X0_PRINTER_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
class Printer
{
public:
  void printGconf(Registers::Gconf gconf);
  void printGstat(Registers::Gstat gstat);
  void printIoin(Registers::Ioin ioin);
  void printSwMode(Registers::SwMode sw_mode);
  void printRampStat(Registers::RampStat ramp_stat);
  void printDrvStatus(Registers::DrvStatus drv_status);
  void printPwmScale(Registers::PwmScale pwm_scale);
private:
  void printRegisterPortion(const char * str, uint32_t value, bool hex=false);
};
}
#endif
