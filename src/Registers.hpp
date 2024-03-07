// ----------------------------------------------------------------------------
// Registers.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_REGISTERS_HPP
#define TMC51X0_REGISTERS_HPP
#include <Arduino.h>

#include "Interface.hpp"


class TMC51X0;

namespace tmc51x0
{
struct Registers
{
  enum RegisterAddress
  {
    GCONF = 0x00,
    GSTAT = 0x01,
    IFCNT = 0x02,
    NODECONF = 0x03,
    IOIN = 0x04,
    X_COMPARE = 0x05,
    OTP_PROG = 0x06,
    OTP_READ = 0x07,
    FACTORY_CONF = 0x08,
    SHORT_CONF = 0x09,
    DRV_CONF = 0x0A,
    GLOBAL_SCALER = 0x0B,
    OFFSET_READ = 0x0C,
    IHOLD_IRUN = 0x10,
    TPOWER_DOWN = 0x11,
    TSTEP = 0x12,
    TPWMTHRS = 0x13,
    TCOOLTHRS = 0x14,
    THIGH = 0x15,
    RAMPMODE = 0x20,
    XACTUAL = 0x21,
    VACTUAL = 0x22,
    VSTART = 0x23,
    A1 = 0x24,
    V1 = 0x25,
    AMAX = 0x26,
    VMAX = 0x27,
    DMAX = 0x28,
    D1 = 0x2A,
    VSTOP = 0x2B,
    TZEROWAIT = 0x2C,
    XTARGET = 0x2D,
    VDCMIN = 0x33,
    SW_MODE = 0x34,
    RAMP_STAT = 0x35,
    XLATCH = 0x36,
    ENCMODE = 0x38,
    X_ENC = 0x39,
    ENC_CONST = 0x3A,
    ENC_STATUS = 0x3B,
    ENC_LATCH = 0x3C,
    ENC_DEVIATION = 0x3D,
    MSLUT_0 = 0x60,
    MSLUT_1 = 0x61,
    MSLUT_2 = 0x62,
    MSLUT_3 = 0x63,
    MSLUT_4 = 0x64,
    MSLUT_5 = 0x65,
    MSLUT_6 = 0x66,
    MSLUT_7 = 0x67,
    MSLUTSEL = 0x68,
    MSLUTSTART = 0x69,
    MSCNT = 0x6A,
    MSCURACT = 0x6B,
    CHOPCONF = 0x6C,
    COOLCONF = 0x6D,
    DCCTRL = 0x6E,
    DRV_STATUS = 0x6F,
    PWMCONF = 0x70,
    PWM_SCALE = 0x71,
    PWM_AUTO = 0x72,
    LOST_STEPS = 0x73,
    ADDRESS_COUNT = 0x74,
  };

  void write(RegisterAddress register_address,
    uint32_t data);
  uint32_t read(RegisterAddress register_address);
  uint32_t getStored(RegisterAddress register_address);
  bool writeable(RegisterAddress register_address);
  bool readable(RegisterAddress register_address);

  union Gconf
  {
    struct
    {
      uint32_t recalibrate : 1;
      uint32_t faststandstill : 1;
      uint32_t enable_pwm_mode : 1;
      uint32_t multistep_filt : 1;
      uint32_t shaft : 1;
      uint32_t diag0_error : 1;
      uint32_t diag0_otpw : 1;
      uint32_t diag0_stall_int_step : 1;
      uint32_t diag1_stall_poscomp_dir : 1;
      uint32_t diag1_index : 1;
      uint32_t diag1_onstate : 1;
      uint32_t diag1_steps_skipped : 1;
      uint32_t diag0_int_pushpull : 1;
      uint32_t diag1_poscomp_pushpull : 1;
      uint32_t small_hysteresis : 1;
      uint32_t stop_enable : 1;
      uint32_t direct_mode : 1;
      uint32_t reserved : 15;
    };
    uint32_t bytes;
  };

  union Ioin
  {
    struct
    {
      uint32_t refl_step : 1;
      uint32_t refr_dir : 1;
      uint32_t encb_dcen_cfg4 : 1;
      uint32_t enca_dcin_cfg5 : 1;
      uint32_t drv_enn : 1;
      uint32_t enc_n_dco_cfg6 : 1;
      uint32_t sd_mode : 1;
      uint32_t swcomp_in : 1;
      uint32_t reserved : 16;
      uint32_t version : 8;
    };
    uint32_t bytes;
  };

  union Encmode
  {
    struct
    {
      uint32_t pol_a : 1;
      uint32_t pol_b : 1;
      uint32_t pol_n : 1;
      uint32_t ignore_ab : 1;
      uint32_t clr_cont : 1;
      uint32_t clr_once : 1;
      uint32_t pos_edge : 1;
      uint32_t neg_edge : 1;
      uint32_t clr_enc_x : 1;
      uint32_t latch_x_act : 1;
      uint32_t enc_sel_decimal : 1;
      uint32_t reserved : 21;
    };
    uint32_t bytes;
  };

  union EncStatus
  {
    struct
    {
      uint32_t n_event : 1;
      uint32_t devition_warn : 1;
      uint32_t reserved : 30;
    };
    uint32_t bytes;
  };

  union Chopconf
  {
    struct
    {
      uint32_t toff : 4;
      uint32_t hstart : 3;
      uint32_t hend : 4;
      uint32_t fd3 : 1;
      uint32_t disfdcc : 1;
      uint32_t reserved_0 : 1;
      uint32_t chopper_mode : 1;
      uint32_t tbl : 2;
      uint32_t reserved_1 : 1;
      uint32_t vhighfs : 1;
      uint32_t vhighchm : 1;
      uint32_t tpfd : 4;
      uint32_t mres : 4;
      uint32_t interpolation : 1;
      uint32_t double_edge : 1;
      uint32_t diss2g : 1;
      uint32_t diss2vs : 1;
    };
    uint32_t bytes;
  };
  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;
  const static uint8_t HSTART_DEFAULT = 0b101;
  const static uint8_t HEND_DEFAULT = 0b10;
  const static uint8_t TBL_DEFAULT = 0b10;
  const static uint8_t TPFD_DEFAULT = 0b100;
  const static uint8_t INTERPOLATION_DEFAULT = 0b1;

private:
  Interface interface_;

  uint32_t stored_[ADDRESS_COUNT] = {0};
  bool writeable_[ADDRESS_COUNT] = {false};
  bool readable_[ADDRESS_COUNT] = {false};

  void setup(SPIClass & spi,
    size_t chip_select_pin);

  const static uint8_t MRES_256 = 0b0000;
  const static uint8_t MRES_128 = 0b0001;
  const static uint8_t MRES_064 = 0b0010;
  const static uint8_t MRES_032 = 0b0011;
  const static uint8_t MRES_016 = 0b0100;
  const static uint8_t MRES_008 = 0b0101;
  const static uint8_t MRES_004 = 0b0110;
  const static uint8_t MRES_002 = 0b0111;
  const static uint8_t MRES_001 = 0b1000;

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  friend class ::TMC51X0;
};
}
#endif
