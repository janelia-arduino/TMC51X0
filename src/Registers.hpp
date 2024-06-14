// ----------------------------------------------------------------------------
// Registers.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_REGISTERS_HPP
#define TMC51X0_REGISTERS_HPP
#include <Arduino.h>

#include "TMC51X0/Interface.hpp"


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
      uint32_t en_pwm_mode : 1;
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

  union Gstat
  {
    struct
    {
      uint32_t reset : 1;
      uint32_t drv_err : 1;
      uint32_t uv_cp : 1;
      uint32_t reserved : 29;
    };
    uint32_t bytes;
  };

  union Nodeconf
  {
    struct
    {
      uint32_t nodeaddr : 8;
      uint32_t senddelay : 4;
      uint32_t reserved : 20;
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

  union ShortConf
  {
    struct
    {
      uint32_t s2vs_level : 4;
      uint32_t reserved0 : 4;
      uint32_t s2g_level : 4;
      uint32_t reserved1 : 4;
      uint32_t shortfilter : 2;
      uint32_t shortdelay : 1;
      uint32_t reserved2 : 13;
    };
    uint32_t bytes;
  };

  union DrvConf
  {
    struct
    {
      uint32_t bbmtime : 5;
      uint32_t reserved0 : 3;
      uint32_t bbmclks : 4;
      uint32_t reserved1 : 4;
      uint32_t otselect : 2;
      uint32_t drvstrength : 2;
      uint32_t filt_isense : 2;
      uint32_t reserved2 : 10;
    };
    uint32_t bytes;
  };

  union IholdIrun
  {
    struct
    {
      uint32_t ihold : 5;
      uint32_t reserved0 : 3;
      uint32_t irun : 5;
      uint32_t reserved1 : 3;
      uint32_t iholddelay : 4;
      uint32_t reserved2 : 12;
    };
    uint32_t bytes;
  };

  union SwMode
  {
    struct
    {
      uint32_t stop_l_enable : 1;
      uint32_t stop_r_enable : 1;
      uint32_t pol_stop_l : 1;
      uint32_t pol_stop_r : 1;
      uint32_t swap_lr : 1;
      uint32_t latch_l_active : 1;
      uint32_t latch_l_inactive : 1;
      uint32_t latch_r_active : 1;
      uint32_t latch_r_inactive : 1;
      uint32_t en_latch_encoder : 1;
      uint32_t sg_stop : 1;
      uint32_t en_softstop : 1;
      uint32_t reserved : 20;
    };
    uint32_t bytes;
  };

  union RampStat
  {
    struct
    {
      uint32_t status_stop_l : 1;
      uint32_t status_stop_r : 1;
      uint32_t status_latch_l : 1;
      uint32_t status_latch_r : 1;
      uint32_t event_stop_l : 1;
      uint32_t event_stop_r : 1;
      uint32_t event_stop_sg : 1;
      uint32_t event_pos_reached : 1;
      uint32_t velocity_reached : 1;
      uint32_t position_reached : 1;
      uint32_t vzero : 1;
      uint32_t t_zerowait_active : 1;
      uint32_t second_move : 1;
      uint32_t status_sg : 1;
      uint32_t reserved : 18;
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

  union EncConst
  {
    struct
    {
      uint32_t fractional : 16;
      uint32_t integer : 16;
    };
    uint32_t bytes;
  };

  union EncStatus
  {
    struct
    {
      uint32_t n_event : 1;
      uint32_t deviation_warn : 1;
      uint32_t reserved : 30;
    };
    uint32_t bytes;
  };

  union Mslutstart
  {
    struct
    {
      uint32_t start_sin : 8;
      uint32_t reserved0 : 8;
      uint32_t start_sin90 : 8;
      uint32_t reserved1 : 8;
    };
    uint32_t bytes;
  };

  union Mscuract
  {
    struct
    {
      uint32_t cur_b : 9;
      uint32_t reserved0 : 7;
      uint32_t cur_a : 9;
      uint32_t reserved1 : 7;
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
      uint32_t chm : 1;
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

  union Coolconf
  {
    struct
    {
      uint32_t semin : 4;
      uint32_t reserved0 : 1;
      uint32_t seup : 2;
      uint32_t reserved1 : 1;
      uint32_t semax : 4;
      uint32_t reserved2 : 1;
      uint32_t sedn : 2;
      uint32_t seimin : 1;
      uint32_t sgt : 7;
      uint32_t reserved3 : 1;
      uint32_t sfilt : 1;
      uint32_t reserved4 : 7;
    };
    uint32_t bytes;
  };

  union Dcctrl
  {
    struct
    {
      uint32_t dc_time : 10;
      uint32_t reserved0 : 6;
      uint32_t dc_sg : 8;
      uint32_t reserved1 : 8;
    };
    uint32_t bytes;
  };

  union DrvStatus
  {
    struct
    {
      uint32_t sg_result : 10;
      uint32_t reserved0 : 2;
      uint32_t s2vsa : 1;
      uint32_t s2vsb : 1;
      uint32_t stealth : 1;
      uint32_t fsactive : 1;
      uint32_t cs_actual : 5;
      uint32_t reserved1 : 3;
      uint32_t stallguard : 1;
      uint32_t ot : 1;
      uint32_t otpw : 1;
      uint32_t s2ga : 1;
      uint32_t s2gb : 1;
      uint32_t ola : 1;
      uint32_t olb : 1;
      uint32_t stst : 1;
    };
    uint32_t bytes;
  };

  union Pwmconf
  {
    struct
    {
      uint32_t pwm_ofs : 8;
      uint32_t pwm_grad : 8;
      uint32_t pwm_freq : 2;
      uint32_t pwm_autoscale : 1;
      uint32_t pwm_autograd : 1;
      uint32_t freewheel : 2;
      uint32_t reserved0 : 2;
      uint32_t pwm_reg : 4;
      uint32_t pwm_lim : 4;
    };
    uint32_t bytes;
  };

  union PwmScale
  {
    struct
    {
      uint32_t pwm_scale_sum : 8;
      uint32_t reserved0 : 8;
      uint32_t pwm_scale_auto : 9;
      uint32_t reserved1 : 7;
    };
    uint32_t bytes;
  };

  union PwmAuto
  {
    struct
    {
      uint32_t pwm_ofs_auto : 8;
      uint32_t reserved0 : 8;
      uint32_t pwm_grad_auto : 8;
      uint32_t reserved1 : 8;
    };
    uint32_t bytes;
  };

private:
  Interface interface_;

  uint32_t stored_[ADDRESS_COUNT] = {0};
  bool writeable_[ADDRESS_COUNT] = {false};
  bool readable_[ADDRESS_COUNT] = {false};

  void initialize(SPIClass & spi,
    size_t chip_select_pin);

  friend class ::TMC51X0;
};
}
#endif
