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
  Printer();

  void setup(Print & print);

  void readAndPrintGconf();
  void printRegister(Registers::Gconf gconf);

  void readClearAndPrintGstat();
  void printRegister(Registers::Gstat gstat);

  void readAndPrintIoin();
  void printRegister(Registers::Ioin ioin);

  void readAndPrintSwMode();
  void printRegister(Registers::SwMode sw_mode);

  void readAndPrintRampStat();
  void printRegister(Registers::RampStat ramp_stat);

  void readAndPrintChopconf();
  void printRegister(Registers::Chopconf chopconf);

  void readAndPrintDrvStatus();
  void printRegister(Registers::DrvStatus drv_status);

  void getStoredAndPrintPwmconf();
  void printRegister(Registers::Pwmconf pwmconf);

  void readAndPrintPwmScale();
  void printRegister(Registers::PwmScale pwm_scale);

  void readAndPrintPwmAuto();
  void printRegister(Registers::PwmAuto pwm_auto);

private:
  Print * print_ptr_;
  Registers * registers_ptr_;

  void initialize(Registers & registers);
  void printRegisterPortion(const char * str, uint32_t value, int base=DEC);

  friend class ::TMC51X0;
};
}
#endif
