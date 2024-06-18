// ----------------------------------------------------------------------------
// Printer.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Printer.hpp"


using namespace tmc51x0;

void Printer::readAndPrintGconf()
{
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->read(tmc51x0::Registers::GCONF);
  printGconf(gconf);
}

void Printer::printGconf(Registers::Gconf gconf)
{
  printRegisterPortion("gconf", gconf.bytes, true);
  printRegisterPortion("recalibrate", gconf.recalibrate);
  printRegisterPortion("faststandstill", gconf.faststandstill);
  printRegisterPortion("en_pwm_mode", gconf.en_pwm_mode);
  printRegisterPortion("multistep_filt", gconf.multistep_filt);
  printRegisterPortion("shaft", gconf.shaft);
  printRegisterPortion("diag0_error", gconf.diag0_error);
  printRegisterPortion("diag0_otpw", gconf.diag0_otpw);
  printRegisterPortion("diag0_stall_int_step", gconf.diag0_stall_int_step);
  printRegisterPortion("diag1_stall_poscomp_dir", gconf.diag1_stall_poscomp_dir);
  printRegisterPortion("diag1_index", gconf.diag1_index);
  printRegisterPortion("diag1_onstate", gconf.diag1_onstate);
  printRegisterPortion("diag1_steps_skipped", gconf.diag1_steps_skipped);
  printRegisterPortion("diag0_int_pushpull", gconf.diag0_int_pushpull);
  printRegisterPortion("diag1_poscomp_pushpull", gconf.diag1_poscomp_pushpull);
  printRegisterPortion("small_hysteresis", gconf.small_hysteresis);
  printRegisterPortion("stop_enable", gconf.stop_enable);
  printRegisterPortion("direct_mode", gconf.direct_mode);

  Serial.println("--------------------------");
}

void Printer::readAndPrintGstat()
{
  Registers::Gstat gstat;
  gstat.bytes = registers_ptr_->read(tmc51x0::Registers::GSTAT);
  printGstat(gstat);
}

void Printer::printGstat(Registers::Gstat gstat)
{
  printRegisterPortion("reset", gstat.reset);
  printRegisterPortion("drv_err", gstat.drv_err);
  printRegisterPortion("uv_cp", gstat.uv_cp);
  Serial.println("--------------------------");
}

void Printer::readAndPrintIoin()
{
  Registers::Ioin ioin;
  ioin.bytes = registers_ptr_->read(tmc51x0::Registers::IOIN);
  printIoin(ioin);
}

void Printer::printIoin(Registers::Ioin ioin)
{
  printRegisterPortion("ioin", ioin.bytes, true);
  printRegisterPortion("refl_step", ioin.refl_step);
  printRegisterPortion("refr_dir", ioin.refr_dir);
  printRegisterPortion("encb_dcen_cfg4", ioin.encb_dcen_cfg4);
  printRegisterPortion("enca_dcin_cfg5", ioin.enca_dcin_cfg5);
  printRegisterPortion("drv_enn", ioin.drv_enn);
  printRegisterPortion("enc_n_dco_cfg6", ioin.enc_n_dco_cfg6);
  printRegisterPortion("sd_mode", ioin.sd_mode);
  printRegisterPortion("swcomp_in", ioin.swcomp_in);
  printRegisterPortion("version", ioin.version, true);

  Serial.println("--------------------------");
}

void Printer::readAndPrintSwMode()
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->read(tmc51x0::Registers::SW_MODE);
  printSwMode(sw_mode);
}

void Printer::printSwMode(Registers::SwMode sw_mode)
{
  printRegisterPortion("sw_mode", sw_mode.bytes, true);
  printRegisterPortion("stop_l_enable", sw_mode.stop_l_enable);
  printRegisterPortion("stop_r_enable", sw_mode.stop_r_enable);
  printRegisterPortion("pol_stop_l", sw_mode.pol_stop_l);
  printRegisterPortion("pol_stop_r", sw_mode.pol_stop_r);
  printRegisterPortion("swap_lr", sw_mode.swap_lr);
  printRegisterPortion("latch_l_active", sw_mode.latch_l_active);
  printRegisterPortion("latch_l_inactive", sw_mode.latch_l_inactive);
  printRegisterPortion("latch_r_active", sw_mode.latch_r_active);
  printRegisterPortion("latch_r_inactive", sw_mode.latch_r_inactive);
  printRegisterPortion("en_latch_encoder", sw_mode.en_latch_encoder);
  printRegisterPortion("sg_stop", sw_mode.sg_stop);
  printRegisterPortion("en_softstop", sw_mode.en_softstop);

  Serial.println("--------------------------");
}

void Printer::readAndPrintRampStat()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(tmc51x0::Registers::RAMP_STAT);
  printRampStat(ramp_stat);
}

void Printer::printRampStat(Registers::RampStat ramp_stat)
{
  printRegisterPortion("ramp_stat", ramp_stat.bytes, true);
  printRegisterPortion("status_stop_l", ramp_stat.status_stop_l);
  printRegisterPortion("status_stop_r", ramp_stat.status_stop_r);
  printRegisterPortion("status_latch_l", ramp_stat.status_latch_l);
  printRegisterPortion("status_latch_r", ramp_stat.status_latch_r);
  printRegisterPortion("event_stop_l", ramp_stat.event_stop_l);
  printRegisterPortion("event_stop_r", ramp_stat.event_stop_r);
  printRegisterPortion("event_pos_reached", ramp_stat.event_pos_reached);
  printRegisterPortion("velocity_reached", ramp_stat.velocity_reached);
  printRegisterPortion("position_reached", ramp_stat.position_reached);
  printRegisterPortion("vzero", ramp_stat.vzero);
  printRegisterPortion("t_zerowait_active", ramp_stat.t_zerowait_active);
  printRegisterPortion("second_move", ramp_stat.second_move);
  printRegisterPortion("status_sg", ramp_stat.status_sg);
  Serial.println("--------------------------");
}

void Printer::readAndPrintDrvStatus()
{
  Registers::DrvStatus drv_status;
  drv_status.bytes = registers_ptr_->read(tmc51x0::Registers::DRV_STATUS);
  printDrvStatus(drv_status);
}

void Printer::printDrvStatus(Registers::DrvStatus drv_status)
{
  printRegisterPortion("drv_status", drv_status.bytes, true);
  printRegisterPortion("sg_result", drv_status.sg_result, true);
  printRegisterPortion("stealth", drv_status.stealth);
  printRegisterPortion("cs_actual", drv_status.cs_actual, true);
  printRegisterPortion("stst", drv_status.stst);
  Serial.println("--------------------------");
}

void Printer::readAndPrintPwmconf()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->read(tmc51x0::Registers::PWMCONF);
  printPwmconf(pwmconf);
}

void Printer::printPwmconf(Registers::Pwmconf pwmconf)
{
  printRegisterPortion("pwmconf", pwmconf.bytes, true);
  printRegisterPortion("pwm_ofs", pwmconf.pwm_ofs, true);
  printRegisterPortion("pwm_grad", pwmconf.pwm_grad, true);
  printRegisterPortion("pwm_freq", pwmconf.pwm_freq);
  printRegisterPortion("pwm_autoscale", pwmconf.pwm_autoscale);
  printRegisterPortion("pwm_autograd", pwmconf.pwm_autograd);
  printRegisterPortion("freewheel", pwmconf.freewheel);
  printRegisterPortion("pwm_reg", pwmconf.pwm_reg, true);
  printRegisterPortion("pwm_lim", pwmconf.pwm_lim, true);
  Serial.println("--------------------------");
}

void Printer::readAndPrintPwmScale()
{
  Registers::PwmScale pwm_scale;
  pwm_scale.bytes = registers_ptr_->read(tmc51x0::Registers::PWM_SCALE);
  printPwmScale(pwm_scale);
}

void Printer::printPwmScale(Registers::PwmScale pwm_scale)
{
  printRegisterPortion("pwm_scale", pwm_scale.bytes, true);
  printRegisterPortion("pwm_scale_sum", pwm_scale.pwm_scale_sum, true);
  printRegisterPortion("pwm_scale_auto", pwm_scale.pwm_scale_auto, true);
  Serial.println("--------------------------");
}

// private

void Printer::initialize(Registers & registers)
{
  registers_ptr_ = &registers;
}

void Printer::printRegisterPortion(const char * str, uint32_t value, bool hex)
{
  Serial.print(str);
  if (not hex)
  {
    Serial.print(": 0b");
    Serial.print(value, BIN);
  }
  else
  {
    Serial.print(": 0x");
    Serial.print(value, HEX);
  }
  Serial.println();
}

