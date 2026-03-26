#include <unity.h>

#include <stdint.h>

#include "Registers.hpp"

using namespace tmc51x0;

static uint32_t bit32(unsigned pos) {
  return (uint32_t(1) << pos);
}

static uint32_t field32(unsigned pos, uint32_t value) {
  return (value << pos);
}

static void test_gconf_gstat_nodeconf_ioin(void) {
  // --------------------------------------------------------------------------
  // Gconf (0x00)
  // --------------------------------------------------------------------------
  {
    Registers::Gconf r;

    r.raw = 0;
    r.recalibrate_i_scale_analog(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.recalibrate_i_scale_analog());

    r.raw = 0;
    r.faststandstill_internal_rsense(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.faststandstill_internal_rsense());

    r.raw = 0;
    r.en_pwm_mode(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.en_pwm_mode());

    r.raw = 0;
    r.multistep_filt_enc_commutation(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(3), r.raw);
    TEST_ASSERT_TRUE(r.multistep_filt_enc_commutation());

    r.raw = 0;
    r.shaft(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(4), r.raw);
    TEST_ASSERT_TRUE(r.shaft());

    r.raw = 0;
    r.diag0_error(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(5), r.raw);
    TEST_ASSERT_TRUE(r.diag0_error());

    r.raw = 0;
    r.diag0_otpw(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(6), r.raw);
    TEST_ASSERT_TRUE(r.diag0_otpw());

    r.raw = 0;
    r.diag0_stall_int_step(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(7), r.raw);
    TEST_ASSERT_TRUE(r.diag0_stall_int_step());

    r.raw = 0;
    r.diag1_stall_poscomp_dir(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(8), r.raw);
    TEST_ASSERT_TRUE(r.diag1_stall_poscomp_dir());

    r.raw = 0;
    r.diag1_index(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(9), r.raw);
    TEST_ASSERT_TRUE(r.diag1_index());

    r.raw = 0;
    r.diag1_onstate(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(10), r.raw);
    TEST_ASSERT_TRUE(r.diag1_onstate());

    r.raw = 0;
    r.diag1_steps_skipped(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(11), r.raw);
    TEST_ASSERT_TRUE(r.diag1_steps_skipped());

    r.raw = 0;
    r.diag0_int_pushpull(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(12), r.raw);
    TEST_ASSERT_TRUE(r.diag0_int_pushpull());

    r.raw = 0;
    r.diag1_poscomp_pushpull(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(13), r.raw);
    TEST_ASSERT_TRUE(r.diag1_poscomp_pushpull());

    r.raw = 0;
    r.small_hysteresis(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(14), r.raw);
    TEST_ASSERT_TRUE(r.small_hysteresis());

    r.raw = 0;
    r.stop_enable(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(15), r.raw);
    TEST_ASSERT_TRUE(r.stop_enable());

    r.raw = 0;
    r.direct_mode(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(16), r.raw);
    TEST_ASSERT_TRUE(r.direct_mode());
  }

  // --------------------------------------------------------------------------
  // Gstat (0x01)
  // --------------------------------------------------------------------------
  {
    Registers::Gstat r;

    r.raw = 0;
    r.reset(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.reset());

    r.raw = 0;
    r.drv_err(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.drv_err());

    r.raw = 0;
    r.uv_cp(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.uv_cp());
  }

  // --------------------------------------------------------------------------
  // Nodeconf (0x03)
  // --------------------------------------------------------------------------
  {
    Registers::Nodeconf r;

    r.raw = 0;
    r.nodeaddr(0x5A);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x5A), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x5A, r.nodeaddr());

    r.raw = 0;
    r.senddelay(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.senddelay());
  }

  // --------------------------------------------------------------------------
  // Ioin (0x04)
  // --------------------------------------------------------------------------
  {
    Registers::Ioin r;

    r.raw = 0;
    r.refl_step(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.refl_step());

    r.raw = 0;
    r.refr_dir(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.refr_dir());

    r.raw = 0;
    r.encb_dcen_cfg4(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.encb_dcen_cfg4());

    r.raw = 0;
    r.enca_dcin_cfg5(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(3), r.raw);
    TEST_ASSERT_TRUE(r.enca_dcin_cfg5());

    r.raw = 0;
    r.drv_enn(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(4), r.raw);
    TEST_ASSERT_TRUE(r.drv_enn());

    r.raw = 0;
    r.enc_n_dco_cfg6(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(5), r.raw);
    TEST_ASSERT_TRUE(r.enc_n_dco_cfg6());

    r.raw = 0;
    r.sd_mode(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(6), r.raw);
    TEST_ASSERT_TRUE(r.sd_mode());

    r.raw = 0;
    r.swcomp_in(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(7), r.raw);
    TEST_ASSERT_TRUE(r.swcomp_in());

    r.raw = 0;
    r.version(0xA5);
    TEST_ASSERT_EQUAL_HEX32(field32(24, 0xA5), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xA5, r.version());
  }
}

static void test_shortconf_drvconf_iholdirun(void) {
  // --------------------------------------------------------------------------
  // ShortConf (0x09)
  // --------------------------------------------------------------------------
  {
    Registers::ShortConf r;

    r.raw = 0;
    r.s2vs_level(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.s2vs_level());

    r.raw = 0;
    r.s2g_level(0xA);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0xA), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xA, r.s2g_level());

    r.raw = 0;
    r.shortfilter(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.shortfilter());

    r.raw = 0;
    r.shortdelay(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(18), r.raw);
    TEST_ASSERT_TRUE(r.shortdelay());
  }

  // --------------------------------------------------------------------------
  // DrvConf (0x0A)
  // --------------------------------------------------------------------------
  {
    Registers::DrvConf r;

    r.raw = 0;
    r.bbmtime(0x1F);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x1F), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x1F, r.bbmtime());

    r.raw = 0;
    r.bbmclks(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.bbmclks());

    r.raw = 0;
    r.otselect(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.otselect());

    r.raw = 0;
    r.drvstrength(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(18, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.drvstrength());

    r.raw = 0;
    r.filt_isense(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(20, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.filt_isense());
  }

  // --------------------------------------------------------------------------
  // IholdIrun (0x10)
  // --------------------------------------------------------------------------
  {
    Registers::IholdIrun r;

    r.raw = 0;
    r.ihold(0x1F);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x1F), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x1F, r.ihold());

    r.raw = 0;
    r.irun(0x1E);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0x1E), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x1E, r.irun());

    r.raw = 0;
    r.iholddelay(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.iholddelay());
  }
}

static void test_swmode_rampstat(void) {
  // --------------------------------------------------------------------------
  // SwMode (0x34)
  // --------------------------------------------------------------------------
  {
    Registers::SwMode r;

    r.raw = 0;
    r.stop_l_enable(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.stop_l_enable());

    r.raw = 0;
    r.stop_r_enable(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.stop_r_enable());

    r.raw = 0;
    r.pol_stop_l(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.pol_stop_l());

    r.raw = 0;
    r.pol_stop_r(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(3), r.raw);
    TEST_ASSERT_TRUE(r.pol_stop_r());

    r.raw = 0;
    r.swap_lr(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(4), r.raw);
    TEST_ASSERT_TRUE(r.swap_lr());

    r.raw = 0;
    r.latch_l_active(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(5), r.raw);
    TEST_ASSERT_TRUE(r.latch_l_active());

    r.raw = 0;
    r.latch_l_inactive(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(6), r.raw);
    TEST_ASSERT_TRUE(r.latch_l_inactive());

    r.raw = 0;
    r.latch_r_active(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(7), r.raw);
    TEST_ASSERT_TRUE(r.latch_r_active());

    r.raw = 0;
    r.latch_r_inactive(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(8), r.raw);
    TEST_ASSERT_TRUE(r.latch_r_inactive());

    r.raw = 0;
    r.en_latch_encoder(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(9), r.raw);
    TEST_ASSERT_TRUE(r.en_latch_encoder());

    r.raw = 0;
    r.sg_stop(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(10), r.raw);
    TEST_ASSERT_TRUE(r.sg_stop());

    r.raw = 0;
    r.en_softstop(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(11), r.raw);
    TEST_ASSERT_TRUE(r.en_softstop());
  }

  // --------------------------------------------------------------------------
  // RampStat (0x35)
  // --------------------------------------------------------------------------
  {
    Registers::RampStat r;

    r.raw = 0;
    r.status_stop_l(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.status_stop_l());

    r.raw = 0;
    r.status_stop_r(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.status_stop_r());

    r.raw = 0;
    r.status_latch_l(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.status_latch_l());

    r.raw = 0;
    r.status_latch_r(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(3), r.raw);
    TEST_ASSERT_TRUE(r.status_latch_r());

    r.raw = 0;
    r.event_stop_l(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(4), r.raw);
    TEST_ASSERT_TRUE(r.event_stop_l());

    r.raw = 0;
    r.event_stop_r(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(5), r.raw);
    TEST_ASSERT_TRUE(r.event_stop_r());

    r.raw = 0;
    r.event_stop_sg(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(6), r.raw);
    TEST_ASSERT_TRUE(r.event_stop_sg());

    r.raw = 0;
    r.event_pos_reached(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(7), r.raw);
    TEST_ASSERT_TRUE(r.event_pos_reached());

    r.raw = 0;
    r.velocity_reached(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(8), r.raw);
    TEST_ASSERT_TRUE(r.velocity_reached());

    r.raw = 0;
    r.position_reached(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(9), r.raw);
    TEST_ASSERT_TRUE(r.position_reached());

    r.raw = 0;
    r.vzero(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(10), r.raw);
    TEST_ASSERT_TRUE(r.vzero());

    r.raw = 0;
    r.t_zerowait_active(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(11), r.raw);
    TEST_ASSERT_TRUE(r.t_zerowait_active());

    r.raw = 0;
    r.second_move(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(12), r.raw);
    TEST_ASSERT_TRUE(r.second_move());

    r.raw = 0;
    r.status_sg(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(13), r.raw);
    TEST_ASSERT_TRUE(r.status_sg());
  }
}

static void test_encoder_registers(void) {
  // --------------------------------------------------------------------------
  // Encmode (0x38)
  // --------------------------------------------------------------------------
  {
    Registers::Encmode r;

    r.raw = 0;
    r.pol_a(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.pol_a());

    r.raw = 0;
    r.pol_b(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.pol_b());

    r.raw = 0;
    r.pol_n(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(2), r.raw);
    TEST_ASSERT_TRUE(r.pol_n());

    r.raw = 0;
    r.ignore_ab(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(3), r.raw);
    TEST_ASSERT_TRUE(r.ignore_ab());

    r.raw = 0;
    r.clr_cont(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(4), r.raw);
    TEST_ASSERT_TRUE(r.clr_cont());

    r.raw = 0;
    r.clr_once(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(5), r.raw);
    TEST_ASSERT_TRUE(r.clr_once());

    r.raw = 0;
    r.pos_edge(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(6), r.raw);
    TEST_ASSERT_TRUE(r.pos_edge());

    r.raw = 0;
    r.neg_edge(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(7), r.raw);
    TEST_ASSERT_TRUE(r.neg_edge());

    r.raw = 0;
    r.clr_enc_x(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(8), r.raw);
    TEST_ASSERT_TRUE(r.clr_enc_x());

    r.raw = 0;
    r.latch_x_act(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(9), r.raw);
    TEST_ASSERT_TRUE(r.latch_x_act());

    r.raw = 0;
    r.enc_sel_decimal(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(10), r.raw);
    TEST_ASSERT_TRUE(r.enc_sel_decimal());
  }

  // --------------------------------------------------------------------------
  // EncConst (0x3A)
  // --------------------------------------------------------------------------
  {
    Registers::EncConst r;

    r.raw = 0;
    r.fractional(0xBEEF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0xBEEF), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0xBEEF, r.fractional());

    r.raw = 0;
    r.integer(0x1234);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x1234), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x1234, r.integer());
  }

  // --------------------------------------------------------------------------
  // EncStatus (0x3B)
  // --------------------------------------------------------------------------
  {
    Registers::EncStatus r;

    r.raw = 0;
    r.n_event(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(0), r.raw);
    TEST_ASSERT_TRUE(r.n_event());

    r.raw = 0;
    r.deviation_warn(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(1), r.raw);
    TEST_ASSERT_TRUE(r.deviation_warn());
  }
}

static void test_microstep_and_driver_registers(void) {
  // --------------------------------------------------------------------------
  // Mslutstart (0x69)
  // --------------------------------------------------------------------------
  {
    Registers::Mslutstart r;

    r.raw = 0;
    r.start_sin(0x5A);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x5A), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x5A, r.start_sin());

    r.raw = 0;
    r.start_sin90(0xC3);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0xC3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xC3, r.start_sin90());
  }

  // --------------------------------------------------------------------------
  // Mscuract (0x6B)
  // --------------------------------------------------------------------------
  {
    Registers::Mscuract r;

    r.raw = 0;
    r.cur_b(0x1FF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x1FF), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x1FF, r.cur_b());

    r.raw = 0;
    r.cur_a(0x155);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x155), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x155, r.cur_a());
  }

  // --------------------------------------------------------------------------
  // Chopconf (0x6C)
  // --------------------------------------------------------------------------
  {
    Registers::Chopconf r;

    r.raw = 0;
    r.toff(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.toff());

    r.raw = 0;
    r.hstart(0x7);
    TEST_ASSERT_EQUAL_HEX32(field32(4, 0x7), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x7, r.hstart());

    r.raw = 0;
    r.hend(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(7, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.hend());

    r.raw = 0;
    r.fd3(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(11), r.raw);
    TEST_ASSERT_TRUE(r.fd3());

    r.raw = 0;
    r.disfdcc(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(12), r.raw);
    TEST_ASSERT_TRUE(r.disfdcc());

    r.raw = 0;
    r.chm(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(14), r.raw);
    TEST_ASSERT_TRUE(r.chm());

    r.raw = 0;
    r.tbl(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(15, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.tbl());

    r.raw = 0;
    r.vhighfs(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(18), r.raw);
    TEST_ASSERT_TRUE(r.vhighfs());

    r.raw = 0;
    r.vhighchm(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(19), r.raw);
    TEST_ASSERT_TRUE(r.vhighchm());

    r.raw = 0;
    r.tpfd(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(20, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.tpfd());

    r.raw = 0;
    r.mres(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(24, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.mres());

    r.raw = 0;
    r.interpolation(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(28), r.raw);
    TEST_ASSERT_TRUE(r.interpolation());

    r.raw = 0;
    r.double_edge(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(29), r.raw);
    TEST_ASSERT_TRUE(r.double_edge());

    r.raw = 0;
    r.diss2g(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(30), r.raw);
    TEST_ASSERT_TRUE(r.diss2g());

    r.raw = 0;
    r.diss2vs(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(31), r.raw);
    TEST_ASSERT_TRUE(r.diss2vs());
  }

  // --------------------------------------------------------------------------
  // Coolconf (0x6D)
  // --------------------------------------------------------------------------
  {
    Registers::Coolconf r;

    r.raw = 0;
    r.semin(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.semin());

    r.raw = 0;
    r.seup(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(5, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.seup());

    r.raw = 0;
    r.semax(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.semax());

    r.raw = 0;
    r.sedn(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(13, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.sedn());

    r.raw = 0;
    r.seimin(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(15), r.raw);
    TEST_ASSERT_TRUE(r.seimin());

    // sgt is a 7-bit signed field on the wire; we store its 7-bit two's
    // complement representation.
    r.raw = 0;
    r.sgt(-1);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x7F), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x7F, r.sgt());

    r.raw = 0;
    r.sfilt(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(24), r.raw);
    TEST_ASSERT_TRUE(r.sfilt());
  }

  // --------------------------------------------------------------------------
  // Dcctrl (0x6E)
  // --------------------------------------------------------------------------
  {
    Registers::Dcctrl r;

    r.raw = 0;
    r.dc_time(0x3FF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x3FF), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x3FF, r.dc_time());

    r.raw = 0;
    r.dc_sg(0xA5);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0xA5), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xA5, r.dc_sg());
  }

  // --------------------------------------------------------------------------
  // DrvStatus (0x6F)
  // --------------------------------------------------------------------------
  {
    Registers::DrvStatus r;

    r.raw = 0;
    r.sg_result(0x3FF);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x3FF), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x3FF, r.sg_result());

    r.raw = 0;
    r.s2vsa(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(12), r.raw);
    TEST_ASSERT_TRUE(r.s2vsa());

    r.raw = 0;
    r.s2vsb(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(13), r.raw);
    TEST_ASSERT_TRUE(r.s2vsb());

    r.raw = 0;
    r.stealth(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(14), r.raw);
    TEST_ASSERT_TRUE(r.stealth());

    r.raw = 0;
    r.fsactive(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(15), r.raw);
    TEST_ASSERT_TRUE(r.fsactive());

    r.raw = 0;
    r.cs_actual(0x1F);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x1F), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x1F, r.cs_actual());

    r.raw = 0;
    r.stallguard(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(24), r.raw);
    TEST_ASSERT_TRUE(r.stallguard());

    r.raw = 0;
    r.ot(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(25), r.raw);
    TEST_ASSERT_TRUE(r.ot());

    r.raw = 0;
    r.otpw(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(26), r.raw);
    TEST_ASSERT_TRUE(r.otpw());

    r.raw = 0;
    r.s2ga(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(27), r.raw);
    TEST_ASSERT_TRUE(r.s2ga());

    r.raw = 0;
    r.s2gb(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(28), r.raw);
    TEST_ASSERT_TRUE(r.s2gb());

    r.raw = 0;
    r.ola(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(29), r.raw);
    TEST_ASSERT_TRUE(r.ola());

    r.raw = 0;
    r.olb(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(30), r.raw);
    TEST_ASSERT_TRUE(r.olb());

    r.raw = 0;
    r.stst(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(31), r.raw);
    TEST_ASSERT_TRUE(r.stst());
  }

  // --------------------------------------------------------------------------
  // Pwmconf (0x70)
  // --------------------------------------------------------------------------
  {
    Registers::Pwmconf r;

    r.raw = 0;
    r.pwm_ofs(0xA5);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0xA5), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xA5, r.pwm_ofs());

    r.raw = 0;
    r.pwm_grad(0x5A);
    TEST_ASSERT_EQUAL_HEX32(field32(8, 0x5A), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x5A, r.pwm_grad());

    r.raw = 0;
    r.pwm_freq(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.pwm_freq());

    r.raw = 0;
    r.pwm_autoscale(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(18), r.raw);
    TEST_ASSERT_TRUE(r.pwm_autoscale());

    r.raw = 0;
    r.pwm_autograd(true);
    TEST_ASSERT_EQUAL_HEX32(bit32(19), r.raw);
    TEST_ASSERT_TRUE(r.pwm_autograd());

    r.raw = 0;
    r.freewheel(0x3);
    TEST_ASSERT_EQUAL_HEX32(field32(20, 0x3), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x3, r.freewheel());

    r.raw = 0;
    r.pwm_reg(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(24, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.pwm_reg());

    r.raw = 0;
    r.pwm_lim(0xF);
    TEST_ASSERT_EQUAL_HEX32(field32(28, 0xF), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0xF, r.pwm_lim());
  }

  // --------------------------------------------------------------------------
  // PwmScale (0x71)
  // --------------------------------------------------------------------------
  {
    Registers::PwmScale r;

    r.raw = 0;
    r.pwm_scale_sum(0x5A);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x5A), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x5A, r.pwm_scale_sum());

    r.raw = 0;
    r.pwm_scale_auto(0x1FF);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x1FF), r.raw);
    TEST_ASSERT_EQUAL_UINT16(0x1FF, r.pwm_scale_auto());
  }

  // --------------------------------------------------------------------------
  // PwmAuto (0x72)
  // --------------------------------------------------------------------------
  {
    Registers::PwmAuto r;

    r.raw = 0;
    r.pwm_ofs_auto(0x12);
    TEST_ASSERT_EQUAL_HEX32(field32(0, 0x12), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x12, r.pwm_ofs_auto());

    r.raw = 0;
    r.pwm_grad_auto(0x34);
    TEST_ASSERT_EQUAL_HEX32(field32(16, 0x34), r.raw);
    TEST_ASSERT_EQUAL_UINT8(0x34, r.pwm_grad_auto());
  }
}

void setUp(void) {}

void tearDown(void) {}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_gconf_gstat_nodeconf_ioin);
  RUN_TEST(test_shortconf_drvconf_iholdirun);
  RUN_TEST(test_swmode_rampstat);
  RUN_TEST(test_encoder_registers);
  RUN_TEST(test_microstep_and_driver_registers);
  return UNITY_END();
}
