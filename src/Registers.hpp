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
    GconfAddress = 0x00,
    GstatAddress = 0x01,
    IfcntAddress = 0x02,
    NodeconfAddress = 0x03,
    IoinAddress = 0x04,
    XCompareAddress = 0x05,
    OtpProgAddress = 0x06,
    OtpReadAddress = 0x07,
    FactoryConfAddress = 0x08,
    ShortConfAddress = 0x09,
    DrvConfAddress = 0x0A,
    GlobalScalerAddress = 0x0B,
    OffsetReadAddress = 0x0C,
    IholdIrunAddress = 0x10,
    TpowerDownAddress = 0x11,
    TstepAddress = 0x12,
    TpwmthrsAddress = 0x13,
    TcoolthrsAddress = 0x14,
    ThighAddress = 0x15,
    RampmodeAddress = 0x20,
    XactualAddress = 0x21,
    VactualAddress = 0x22,
    VstartAddress = 0x23,
    Acceleration1Address = 0x24,
    Velocity1Address = 0x25,
    AmaxAddress = 0x26,
    VmaxAddress = 0x27,
    DmaxAddress = 0x28,
    Deceleration1Address = 0x2A,
    VstopAddress = 0x2B,
    TzerowaitAddress = 0x2C,
    XtargetAddress = 0x2D,
    VdcminAddress = 0x33,
    SwModeAddress = 0x34,
    RampStatAddress = 0x35,
    XlatchAddress = 0x36,
    EncmodeAddress = 0x38,
    XencAddress = 0x39,
    EncConstAddress = 0x3A,
    EncStatusAddress = 0x3B,
    EncLatchAddress = 0x3C,
    EncDeviationAddress = 0x3D,
    Mslut0Address = 0x60,
    Mslut1Address = 0x61,
    Mslut2Address = 0x62,
    Mslut3Address = 0x63,
    Mslut4Address = 0x64,
    Mslut5Address = 0x65,
    Mslut6Address = 0x66,
    Mslut7Address = 0x67,
    MslutselAddress = 0x68,
    MSLUTSTART = 0x69,
    MscntAddress = 0x6A,
    MscuractAddress = 0x6B,
    ChopconfAddress = 0x6C,
    CoolconfAddress = 0x6D,
    DcctrlAddress = 0x6E,
    DrvStatusAddress = 0x6F,
    PwmconfAddress = 0x70,
    PwmScaleAddress = 0x71,
    PwmAutoAddress = 0x72,
    LostStepsAddress = 0x73,
    AddressCount = 0x74,
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
      uint32_t recalibrate_i_scale_analog : 1;
      uint32_t faststandstill_internal_rsense : 1;
      uint32_t en_pwm_mode : 1;
      uint32_t multistep_filt_enc_commutation : 1;
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
  Gstat readAndClearGstat();

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
  Interface * interface_ptr_;

  uint32_t stored_[AddressCount] = {0};
  bool writeable_[AddressCount] = {false};
  bool readable_[AddressCount] = {false};

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  void initialize(Interface & interface);

  friend class ::TMC51X0;
};

const size_t NO_PIN = 255;
}
#endif
