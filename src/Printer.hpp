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
  void readAndPrintGconf();
  void printGconf(Registers::Gconf gconf);

  void readAndPrintGstat();
  void printGstat(Registers::Gstat gstat);

  void readAndPrintIoin();
  void printIoin(Registers::Ioin ioin);

  void readAndPrintSwMode();
  void printSwMode(Registers::SwMode sw_mode);

  void readAndPrintRampStat();
  void printRampStat(Registers::RampStat ramp_stat);

  void readAndPrintDrvStatus();
  void printDrvStatus(Registers::DrvStatus drv_status);

  void readAndPrintPwmconf();
  void printPwmconf(Registers::Pwmconf pwmconf);

  void readAndPrintPwmScale();
  void printPwmScale(Registers::PwmScale pwm_scale);
private:
  Registers * registers_ptr_;

  void initialize(Registers & registers);
  void printRegisterPortion(const char * str, uint32_t value, bool hex=false);

  friend class ::TMC51X0;
};
}
#endif
