// ----------------------------------------------------------------------------
// Printer.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Printer.hpp"


using namespace tmc51x0;

Printer::Printer()
{
  print_ptr_ = nullptr;
  registers_ptr_ = nullptr;
}

void Printer::setup(Print & print)
{
  print_ptr_ = &print;
}

void Printer::readAndPrintGconf()
{
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->read(Registers::GconfAddress);
  printRegister(gconf);
}

void Printer::printRegister(Registers::Gconf gconf)
{
  printRegisterPortion("gconf", gconf.bytes, HEX);
  printRegisterPortion("recalibrate_i_scale_analog", gconf.recalibrate_i_scale_analog, BIN);
  printRegisterPortion("faststandstill_internal_rsense", gconf.faststandstill_internal_rsense, BIN);
  printRegisterPortion("en_pwm_mode", gconf.en_pwm_mode, BIN);
  printRegisterPortion("multistep_filt_enc_commutation", gconf.multistep_filt_enc_commutation, BIN);
  printRegisterPortion("shaft", gconf.shaft, BIN);
  printRegisterPortion("diag0_error", gconf.diag0_error, BIN);
  printRegisterPortion("diag0_otpw", gconf.diag0_otpw, BIN);
  printRegisterPortion("diag0_stall_int_step", gconf.diag0_stall_int_step, BIN);
  printRegisterPortion("diag1_stall_poscomp_dir", gconf.diag1_stall_poscomp_dir, BIN);
  printRegisterPortion("diag1_index", gconf.diag1_index, BIN);
  printRegisterPortion("diag1_onstate", gconf.diag1_onstate, BIN);
  printRegisterPortion("diag1_steps_skipped", gconf.diag1_steps_skipped, BIN);
  printRegisterPortion("diag0_int_pushpull", gconf.diag0_int_pushpull, BIN);
  printRegisterPortion("diag1_poscomp_pushpull", gconf.diag1_poscomp_pushpull, BIN);
  printRegisterPortion("small_hysteresis", gconf.small_hysteresis, BIN);
  printRegisterPortion("stop_enable", gconf.stop_enable, BIN);
  printRegisterPortion("direct_mode", gconf.direct_mode, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::readClearAndPrintGstat()
{
  Registers::Gstat gstat = registers_ptr_->readAndClearGstat();
  printRegister(gstat);
}

void Printer::printRegister(Registers::Gstat gstat)
{
  printRegisterPortion("reset", gstat.reset, BIN);
  printRegisterPortion("drv_err", gstat.drv_err, BIN);
  printRegisterPortion("uv_cp", gstat.uv_cp, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintIoin()
{
  Registers::Ioin ioin;
  ioin.bytes = registers_ptr_->read(Registers::IoinAddress);
  printRegister(ioin);
}

void Printer::printRegister(Registers::Ioin ioin)
{
  printRegisterPortion("ioin", ioin.bytes, HEX);
  printRegisterPortion("refl_step", ioin.refl_step, BIN);
  printRegisterPortion("refr_dir", ioin.refr_dir, BIN);
  printRegisterPortion("encb_dcen_cfg4", ioin.encb_dcen_cfg4, BIN);
  printRegisterPortion("enca_dcin_cfg5", ioin.enca_dcin_cfg5, BIN);
  printRegisterPortion("drv_enn", ioin.drv_enn, BIN);
  printRegisterPortion("enc_n_dco_cfg6", ioin.enc_n_dco_cfg6, BIN);
  printRegisterPortion("sd_mode", ioin.sd_mode, BIN);
  printRegisterPortion("swcomp_in", ioin.swcomp_in, BIN);
  printRegisterPortion("version", ioin.version, HEX);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintSwMode()
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->read(Registers::SwModeAddress);
  printRegister(sw_mode);
}

void Printer::printRegister(Registers::SwMode sw_mode)
{
  printRegisterPortion("sw_mode", sw_mode.bytes, HEX);
  printRegisterPortion("stop_l_enable", sw_mode.stop_l_enable, BIN);
  printRegisterPortion("stop_r_enable", sw_mode.stop_r_enable, BIN);
  printRegisterPortion("pol_stop_l", sw_mode.pol_stop_l, BIN);
  printRegisterPortion("pol_stop_r", sw_mode.pol_stop_r, BIN);
  printRegisterPortion("swap_lr", sw_mode.swap_lr, BIN);
  printRegisterPortion("latch_l_active", sw_mode.latch_l_active, BIN);
  printRegisterPortion("latch_l_inactive", sw_mode.latch_l_inactive, BIN);
  printRegisterPortion("latch_r_active", sw_mode.latch_r_active, BIN);
  printRegisterPortion("latch_r_inactive", sw_mode.latch_r_inactive, BIN);
  printRegisterPortion("en_latch_encoder", sw_mode.en_latch_encoder, BIN);
  printRegisterPortion("sg_stop", sw_mode.sg_stop, BIN);
  printRegisterPortion("en_softstop", sw_mode.en_softstop, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintRampStat()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  printRegister(ramp_stat);
}

void Printer::printRegister(Registers::RampStat ramp_stat)
{
  printRegisterPortion("ramp_stat", ramp_stat.bytes, HEX);
  printRegisterPortion("status_stop_l", ramp_stat.status_stop_l, BIN);
  printRegisterPortion("status_stop_r", ramp_stat.status_stop_r, BIN);
  printRegisterPortion("status_latch_l", ramp_stat.status_latch_l, BIN);
  printRegisterPortion("status_latch_r", ramp_stat.status_latch_r, BIN);
  printRegisterPortion("event_stop_l", ramp_stat.event_stop_l, BIN);
  printRegisterPortion("event_stop_r", ramp_stat.event_stop_r, BIN);
  printRegisterPortion("event_pos_reached", ramp_stat.event_pos_reached, BIN);
  printRegisterPortion("velocity_reached", ramp_stat.velocity_reached, BIN);
  printRegisterPortion("position_reached", ramp_stat.position_reached, BIN);
  printRegisterPortion("vzero", ramp_stat.vzero, BIN);
  printRegisterPortion("t_zerowait_active", ramp_stat.t_zerowait_active, BIN);
  printRegisterPortion("second_move", ramp_stat.second_move, BIN);
  printRegisterPortion("status_sg", ramp_stat.status_sg, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintChopconf()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->read(Registers::ChopconfAddress);
  printRegister(chopconf);
}

void Printer::printRegister(Registers::Chopconf chopconf)
{
  printRegisterPortion("chopconf", chopconf.bytes, HEX);
  printRegisterPortion("toff", chopconf.toff, BIN);
  printRegisterPortion("hstart", chopconf.hstart, BIN);
  printRegisterPortion("hend", chopconf.hend, BIN);
  printRegisterPortion("fd3", chopconf.fd3, BIN);
  printRegisterPortion("disfdcc", chopconf.disfdcc, BIN);
  printRegisterPortion("chm", chopconf.chm, BIN);
  printRegisterPortion("tbl", chopconf.tbl, BIN);
  printRegisterPortion("vhighfs", chopconf.vhighfs, BIN);
  printRegisterPortion("vhighchm", chopconf.vhighchm, BIN);
  printRegisterPortion("tpfd", chopconf.tpfd, BIN);
  printRegisterPortion("mres", chopconf.mres, BIN);
  printRegisterPortion("interpolation", chopconf.interpolation, BIN);
  printRegisterPortion("double edge", chopconf.double_edge, BIN);
  printRegisterPortion("diss2g", chopconf.diss2g, BIN);
  printRegisterPortion("diss2vs", chopconf.diss2vs, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintDrvStatus()
{
  Registers::DrvStatus drv_status;
  drv_status.bytes = registers_ptr_->read(Registers::DrvStatusAddress);
  printRegister(drv_status);
}

void Printer::printRegister(Registers::DrvStatus drv_status)
{
  printRegisterPortion("drv_status", drv_status.bytes, HEX);
  printRegisterPortion("sg_result", drv_status.sg_result, DEC);
  printRegisterPortion("s2vsa", drv_status.s2vsa, BIN);
  printRegisterPortion("s2vsb", drv_status.s2vsb, BIN);
  printRegisterPortion("stealth", drv_status.stealth, BIN);
  printRegisterPortion("fsactive", drv_status.fsactive, BIN);
  printRegisterPortion("cs_actual", drv_status.cs_actual, DEC);
  printRegisterPortion("stallguard", drv_status.stallguard, BIN);
  printRegisterPortion("ot", drv_status.ot, BIN);
  printRegisterPortion("otpw", drv_status.otpw, BIN);
  printRegisterPortion("s2ga", drv_status.s2ga, BIN);
  printRegisterPortion("s2gb", drv_status.s2gb, BIN);
  printRegisterPortion("ola", drv_status.ola, BIN);
  printRegisterPortion("olb", drv_status.olb, BIN);
  printRegisterPortion("stst", drv_status.stst, BIN);
  print_ptr_->println("--------------------------");
}

void Printer::getStoredAndPrintPwmconf()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PwmconfAddress);
  printRegister(pwmconf);
}

void Printer::printRegister(Registers::Pwmconf pwmconf)
{
  printRegisterPortion("pwmconf", pwmconf.bytes, HEX);
  printRegisterPortion("pwm_ofs", pwmconf.pwm_ofs, DEC);
  printRegisterPortion("pwm_grad", pwmconf.pwm_grad, DEC);
  printRegisterPortion("pwm_freq", pwmconf.pwm_freq, BIN);
  printRegisterPortion("pwm_autoscale", pwmconf.pwm_autoscale, BIN);
  printRegisterPortion("pwm_autograd", pwmconf.pwm_autograd, BIN);
  printRegisterPortion("freewheel", pwmconf.freewheel, BIN);
  printRegisterPortion("pwm_reg", pwmconf.pwm_reg, DEC);
  printRegisterPortion("pwm_lim", pwmconf.pwm_lim, DEC);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintPwmScale()
{
  Registers::PwmScale pwm_scale;
  pwm_scale.bytes = registers_ptr_->read(Registers::PwmScaleAddress);
  printRegister(pwm_scale);
}

void Printer::printRegister(Registers::PwmScale pwm_scale)
{
  printRegisterPortion("pwm_scale", pwm_scale.bytes, HEX);
  printRegisterPortion("pwm_scale_sum", pwm_scale.pwm_scale_sum, DEC);
  int16_t pwm_scale_auto = pwm_scale.pwm_scale_auto;
  if (pwm_scale_auto > 255)
  {
    pwm_scale_auto = 255 - pwm_scale_auto;
  }
  printRegisterPortion("pwm_scale_auto", pwm_scale_auto, DEC);
  print_ptr_->println("--------------------------");
}

void Printer::readAndPrintPwmAuto()
{
  Registers::PwmAuto pwm_auto;
  pwm_auto.bytes = registers_ptr_->read(Registers::PwmAutoAddress);
  printRegister(pwm_auto);
}

void Printer::printRegister(Registers::PwmAuto pwm_auto)
{
  printRegisterPortion("pwm_auto", pwm_auto.bytes, HEX);
  printRegisterPortion("pwm_ofs_auto", pwm_auto.pwm_ofs_auto, DEC);
  printRegisterPortion("pwm_grad_auto", pwm_auto.pwm_grad_auto, DEC);
  print_ptr_->println("--------------------------");
}

// private

void Printer::initialize(Registers & registers)
{
  print_ptr_ = &Serial;
  registers_ptr_ = &registers;
}

void Printer::printRegisterPortion(const char * str, uint32_t value, int base)
{
  print_ptr_->print(str);
  switch (base)
  {
    case BIN:
      print_ptr_->print(": 0b");
      break;
    case HEX:
      print_ptr_->print(": 0x");
      break;
    default:
      print_ptr_->print(": ");
  }
  print_ptr_->print(value, base);
  print_ptr_->println();
}

