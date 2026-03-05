// ----------------------------------------------------------------------------
// Registers.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_REGISTERS_HPP
#define TMC51X0_REGISTERS_HPP
#include <Arduino.h>

#include "Constants.hpp"

#include "TMC51X0/Interface.hpp"

#include "tmc_bits.hpp"

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

  void write (RegisterAddress register_address,
              uint32_t data);
  uint32_t read (RegisterAddress register_address);
  uint32_t getStored (RegisterAddress register_address);
  bool writeable (RegisterAddress register_address);
  bool readable (RegisterAddress register_address);
  struct Gconf
  {
    uint32_t raw{ 0 };

    using RECALIBRATE_I_SCALE_ANALOG = tmc::bits::Bit<0>;
    using FASTSTANDSTILL_INTERNAL_RSENSE = tmc::bits::Bit<1>;
    using EN_PWM_MODE = tmc::bits::Bit<2>;
    using MULTISTEP_FILT_ENC_COMMUTATION = tmc::bits::Bit<3>;
    using SHAFT = tmc::bits::Bit<4>;
    using DIAG0_ERROR = tmc::bits::Bit<5>;
    using DIAG0_OTPW = tmc::bits::Bit<6>;
    using DIAG0_STALL_INT_STEP = tmc::bits::Bit<7>;
    using DIAG1_STALL_POSCOMP_DIR = tmc::bits::Bit<8>;
    using DIAG1_INDEX = tmc::bits::Bit<9>;
    using DIAG1_ONSTATE = tmc::bits::Bit<10>;
    using DIAG1_STEPS_SKIPPED = tmc::bits::Bit<11>;
    using DIAG0_INT_PUSHPULL = tmc::bits::Bit<12>;
    using DIAG1_POSCOMP_PUSHPULL = tmc::bits::Bit<13>;
    using SMALL_HYSTERESIS = tmc::bits::Bit<14>;
    using STOP_ENABLE = tmc::bits::Bit<15>;
    using DIRECT_MODE = tmc::bits::Bit<16>;

    Gconf &
    recalibrate_i_scale_analog (bool v)
    {
      RECALIBRATE_I_SCALE_ANALOG::set (raw, v);
      return *this;
    }
    bool
    recalibrate_i_scale_analog () const
    {
      return RECALIBRATE_I_SCALE_ANALOG::get (raw);
    }

    Gconf &
    faststandstill_internal_rsense (bool v)
    {
      FASTSTANDSTILL_INTERNAL_RSENSE::set (raw, v);
      return *this;
    }
    bool
    faststandstill_internal_rsense () const
    {
      return FASTSTANDSTILL_INTERNAL_RSENSE::get (raw);
    }

    Gconf &
    en_pwm_mode (bool v)
    {
      EN_PWM_MODE::set (raw, v);
      return *this;
    }
    bool
    en_pwm_mode () const
    {
      return EN_PWM_MODE::get (raw);
    }

    Gconf &
    multistep_filt_enc_commutation (bool v)
    {
      MULTISTEP_FILT_ENC_COMMUTATION::set (raw, v);
      return *this;
    }
    bool
    multistep_filt_enc_commutation () const
    {
      return MULTISTEP_FILT_ENC_COMMUTATION::get (raw);
    }

    Gconf &
    shaft (bool v)
    {
      SHAFT::set (raw, v);
      return *this;
    }
    bool
    shaft () const
    {
      return SHAFT::get (raw);
    }

    Gconf &
    diag0_error (bool v)
    {
      DIAG0_ERROR::set (raw, v);
      return *this;
    }
    bool
    diag0_error () const
    {
      return DIAG0_ERROR::get (raw);
    }

    Gconf &
    diag0_otpw (bool v)
    {
      DIAG0_OTPW::set (raw, v);
      return *this;
    }
    bool
    diag0_otpw () const
    {
      return DIAG0_OTPW::get (raw);
    }

    Gconf &
    diag0_stall_int_step (bool v)
    {
      DIAG0_STALL_INT_STEP::set (raw, v);
      return *this;
    }
    bool
    diag0_stall_int_step () const
    {
      return DIAG0_STALL_INT_STEP::get (raw);
    }

    Gconf &
    diag1_stall_poscomp_dir (bool v)
    {
      DIAG1_STALL_POSCOMP_DIR::set (raw, v);
      return *this;
    }
    bool
    diag1_stall_poscomp_dir () const
    {
      return DIAG1_STALL_POSCOMP_DIR::get (raw);
    }

    Gconf &
    diag1_index (bool v)
    {
      DIAG1_INDEX::set (raw, v);
      return *this;
    }
    bool
    diag1_index () const
    {
      return DIAG1_INDEX::get (raw);
    }

    Gconf &
    diag1_onstate (bool v)
    {
      DIAG1_ONSTATE::set (raw, v);
      return *this;
    }
    bool
    diag1_onstate () const
    {
      return DIAG1_ONSTATE::get (raw);
    }

    Gconf &
    diag1_steps_skipped (bool v)
    {
      DIAG1_STEPS_SKIPPED::set (raw, v);
      return *this;
    }
    bool
    diag1_steps_skipped () const
    {
      return DIAG1_STEPS_SKIPPED::get (raw);
    }

    Gconf &
    diag0_int_pushpull (bool v)
    {
      DIAG0_INT_PUSHPULL::set (raw, v);
      return *this;
    }
    bool
    diag0_int_pushpull () const
    {
      return DIAG0_INT_PUSHPULL::get (raw);
    }

    Gconf &
    diag1_poscomp_pushpull (bool v)
    {
      DIAG1_POSCOMP_PUSHPULL::set (raw, v);
      return *this;
    }
    bool
    diag1_poscomp_pushpull () const
    {
      return DIAG1_POSCOMP_PUSHPULL::get (raw);
    }

    Gconf &
    small_hysteresis (bool v)
    {
      SMALL_HYSTERESIS::set (raw, v);
      return *this;
    }
    bool
    small_hysteresis () const
    {
      return SMALL_HYSTERESIS::get (raw);
    }

    Gconf &
    stop_enable (bool v)
    {
      STOP_ENABLE::set (raw, v);
      return *this;
    }
    bool
    stop_enable () const
    {
      return STOP_ENABLE::get (raw);
    }

    Gconf &
    direct_mode (bool v)
    {
      DIRECT_MODE::set (raw, v);
      return *this;
    }
    bool
    direct_mode () const
    {
      return DIRECT_MODE::get (raw);
    }
  };

  struct Gstat
  {
    uint32_t raw{ 0 };

    using RESET = tmc::bits::Bit<0>;
    using DRV_ERR = tmc::bits::Bit<1>;
    using UV_CP = tmc::bits::Bit<2>;

    Gstat &
    reset (bool v)
    {
      RESET::set (raw, v);
      return *this;
    }
    bool
    reset () const
    {
      return RESET::get (raw);
    }

    Gstat &
    drv_err (bool v)
    {
      DRV_ERR::set (raw, v);
      return *this;
    }
    bool
    drv_err () const
    {
      return DRV_ERR::get (raw);
    }

    Gstat &
    uv_cp (bool v)
    {
      UV_CP::set (raw, v);
      return *this;
    }
    bool
    uv_cp () const
    {
      return UV_CP::get (raw);
    }
  };
  Gstat readAndClearGstat ();

  struct Nodeconf
  {
    uint32_t raw{ 0 };

    using NODEADDR = tmc::bits::Field<0, 8>;
    using SENDDELAY = tmc::bits::Field<8, 4>;

    Nodeconf &
    nodeaddr (uint8_t v)
    {
      NODEADDR::set (raw, v);
      return *this;
    }
    uint8_t
    nodeaddr () const
    {
      return static_cast<uint8_t> (NODEADDR::get (raw));
    }

    Nodeconf &
    senddelay (uint8_t v)
    {
      SENDDELAY::set (raw, v);
      return *this;
    }
    uint8_t
    senddelay () const
    {
      return static_cast<uint8_t> (SENDDELAY::get (raw));
    }
  };

  struct Ioin
  {
    uint32_t raw{ 0 };

    using REFL_STEP = tmc::bits::Bit<0>;
    using REFR_DIR = tmc::bits::Bit<1>;
    using ENCB_DCEN_CFG4 = tmc::bits::Bit<2>;
    using ENCA_DCIN_CFG5 = tmc::bits::Bit<3>;
    using DRV_ENN = tmc::bits::Bit<4>;
    using ENC_N_DCO_CFG6 = tmc::bits::Bit<5>;
    using SD_MODE = tmc::bits::Bit<6>;
    using SWCOMP_IN = tmc::bits::Bit<7>;
    using VERSION = tmc::bits::Field<24, 8>;

    Ioin &
    refl_step (bool v)
    {
      REFL_STEP::set (raw, v);
      return *this;
    }
    bool
    refl_step () const
    {
      return REFL_STEP::get (raw);
    }

    Ioin &
    refr_dir (bool v)
    {
      REFR_DIR::set (raw, v);
      return *this;
    }
    bool
    refr_dir () const
    {
      return REFR_DIR::get (raw);
    }

    Ioin &
    encb_dcen_cfg4 (bool v)
    {
      ENCB_DCEN_CFG4::set (raw, v);
      return *this;
    }
    bool
    encb_dcen_cfg4 () const
    {
      return ENCB_DCEN_CFG4::get (raw);
    }

    Ioin &
    enca_dcin_cfg5 (bool v)
    {
      ENCA_DCIN_CFG5::set (raw, v);
      return *this;
    }
    bool
    enca_dcin_cfg5 () const
    {
      return ENCA_DCIN_CFG5::get (raw);
    }

    Ioin &
    drv_enn (bool v)
    {
      DRV_ENN::set (raw, v);
      return *this;
    }
    bool
    drv_enn () const
    {
      return DRV_ENN::get (raw);
    }

    Ioin &
    enc_n_dco_cfg6 (bool v)
    {
      ENC_N_DCO_CFG6::set (raw, v);
      return *this;
    }
    bool
    enc_n_dco_cfg6 () const
    {
      return ENC_N_DCO_CFG6::get (raw);
    }

    Ioin &
    sd_mode (bool v)
    {
      SD_MODE::set (raw, v);
      return *this;
    }
    bool
    sd_mode () const
    {
      return SD_MODE::get (raw);
    }

    Ioin &
    swcomp_in (bool v)
    {
      SWCOMP_IN::set (raw, v);
      return *this;
    }
    bool
    swcomp_in () const
    {
      return SWCOMP_IN::get (raw);
    }

    Ioin &
    version (uint8_t v)
    {
      VERSION::set (raw, v);
      return *this;
    }
    uint8_t
    version () const
    {
      return static_cast<uint8_t> (VERSION::get (raw));
    }
  };

  struct ShortConf
  {
    uint32_t raw{ 0 };

    using S2VS_LEVEL = tmc::bits::Field<0, 4>;
    using S2G_LEVEL = tmc::bits::Field<8, 4>;
    using SHORTFILTER = tmc::bits::Field<16, 2>;
    using SHORTDELAY = tmc::bits::Bit<18>;

    ShortConf &
    s2vs_level (uint8_t v)
    {
      S2VS_LEVEL::set (raw, v);
      return *this;
    }
    uint8_t
    s2vs_level () const
    {
      return static_cast<uint8_t> (S2VS_LEVEL::get (raw));
    }

    ShortConf &
    s2g_level (uint8_t v)
    {
      S2G_LEVEL::set (raw, v);
      return *this;
    }
    uint8_t
    s2g_level () const
    {
      return static_cast<uint8_t> (S2G_LEVEL::get (raw));
    }

    ShortConf &
    shortfilter (uint8_t v)
    {
      SHORTFILTER::set (raw, v);
      return *this;
    }
    uint8_t
    shortfilter () const
    {
      return static_cast<uint8_t> (SHORTFILTER::get (raw));
    }

    ShortConf &
    shortdelay (bool v)
    {
      SHORTDELAY::set (raw, v);
      return *this;
    }
    bool
    shortdelay () const
    {
      return SHORTDELAY::get (raw);
    }
  };

  struct DrvConf
  {
    uint32_t raw{ 0 };

    using BBMTIME = tmc::bits::Field<0, 5>;
    using BBMCLKS = tmc::bits::Field<8, 4>;
    using OTSELECT = tmc::bits::Field<16, 2>;
    using DRVSTRENGTH = tmc::bits::Field<18, 2>;
    using FILT_ISENSE = tmc::bits::Field<20, 2>;

    DrvConf &
    bbmtime (uint8_t v)
    {
      BBMTIME::set (raw, v);
      return *this;
    }
    uint8_t
    bbmtime () const
    {
      return static_cast<uint8_t> (BBMTIME::get (raw));
    }

    DrvConf &
    bbmclks (uint8_t v)
    {
      BBMCLKS::set (raw, v);
      return *this;
    }
    uint8_t
    bbmclks () const
    {
      return static_cast<uint8_t> (BBMCLKS::get (raw));
    }

    DrvConf &
    otselect (uint8_t v)
    {
      OTSELECT::set (raw, v);
      return *this;
    }
    uint8_t
    otselect () const
    {
      return static_cast<uint8_t> (OTSELECT::get (raw));
    }

    DrvConf &
    drvstrength (uint8_t v)
    {
      DRVSTRENGTH::set (raw, v);
      return *this;
    }
    uint8_t
    drvstrength () const
    {
      return static_cast<uint8_t> (DRVSTRENGTH::get (raw));
    }

    DrvConf &
    filt_isense (uint8_t v)
    {
      FILT_ISENSE::set (raw, v);
      return *this;
    }
    uint8_t
    filt_isense () const
    {
      return static_cast<uint8_t> (FILT_ISENSE::get (raw));
    }
  };
  struct IholdIrun
  {
    uint32_t raw{ 0 };

    using IHOLD = tmc::bits::Field<0, 5>;
    using IRUN = tmc::bits::Field<8, 5>;
    using IHOLDDELAY = tmc::bits::Field<16, 4>;

    IholdIrun &
    ihold (uint8_t v)
    {
      IHOLD::set (raw, v);
      return *this;
    }
    uint8_t
    ihold () const
    {
      return static_cast<uint8_t> (IHOLD::get (raw));
    }

    IholdIrun &
    irun (uint8_t v)
    {
      IRUN::set (raw, v);
      return *this;
    }
    uint8_t
    irun () const
    {
      return static_cast<uint8_t> (IRUN::get (raw));
    }

    IholdIrun &
    iholddelay (uint8_t v)
    {
      IHOLDDELAY::set (raw, v);
      return *this;
    }
    uint8_t
    iholddelay () const
    {
      return static_cast<uint8_t> (IHOLDDELAY::get (raw));
    }
  };

  struct SwMode
  {
    uint32_t raw{ 0 };

    using STOP_L_ENABLE = tmc::bits::Bit<0>;
    using STOP_R_ENABLE = tmc::bits::Bit<1>;
    using POL_STOP_L = tmc::bits::Bit<2>;
    using POL_STOP_R = tmc::bits::Bit<3>;
    using SWAP_LR = tmc::bits::Bit<4>;
    using LATCH_L_ACTIVE = tmc::bits::Bit<5>;
    using LATCH_L_INACTIVE = tmc::bits::Bit<6>;
    using LATCH_R_ACTIVE = tmc::bits::Bit<7>;
    using LATCH_R_INACTIVE = tmc::bits::Bit<8>;
    using EN_LATCH_ENCODER = tmc::bits::Bit<9>;
    using SG_STOP = tmc::bits::Bit<10>;
    using EN_SOFTSTOP = tmc::bits::Bit<11>;

    SwMode &
    stop_l_enable (bool v)
    {
      STOP_L_ENABLE::set (raw, v);
      return *this;
    }
    bool
    stop_l_enable () const
    {
      return STOP_L_ENABLE::get (raw);
    }

    SwMode &
    stop_r_enable (bool v)
    {
      STOP_R_ENABLE::set (raw, v);
      return *this;
    }
    bool
    stop_r_enable () const
    {
      return STOP_R_ENABLE::get (raw);
    }

    SwMode &
    pol_stop_l (bool v)
    {
      POL_STOP_L::set (raw, v);
      return *this;
    }
    bool
    pol_stop_l () const
    {
      return POL_STOP_L::get (raw);
    }

    SwMode &
    pol_stop_r (bool v)
    {
      POL_STOP_R::set (raw, v);
      return *this;
    }
    bool
    pol_stop_r () const
    {
      return POL_STOP_R::get (raw);
    }

    SwMode &
    swap_lr (bool v)
    {
      SWAP_LR::set (raw, v);
      return *this;
    }
    bool
    swap_lr () const
    {
      return SWAP_LR::get (raw);
    }

    SwMode &
    latch_l_active (bool v)
    {
      LATCH_L_ACTIVE::set (raw, v);
      return *this;
    }
    bool
    latch_l_active () const
    {
      return LATCH_L_ACTIVE::get (raw);
    }

    SwMode &
    latch_l_inactive (bool v)
    {
      LATCH_L_INACTIVE::set (raw, v);
      return *this;
    }
    bool
    latch_l_inactive () const
    {
      return LATCH_L_INACTIVE::get (raw);
    }

    SwMode &
    latch_r_active (bool v)
    {
      LATCH_R_ACTIVE::set (raw, v);
      return *this;
    }
    bool
    latch_r_active () const
    {
      return LATCH_R_ACTIVE::get (raw);
    }

    SwMode &
    latch_r_inactive (bool v)
    {
      LATCH_R_INACTIVE::set (raw, v);
      return *this;
    }
    bool
    latch_r_inactive () const
    {
      return LATCH_R_INACTIVE::get (raw);
    }

    SwMode &
    en_latch_encoder (bool v)
    {
      EN_LATCH_ENCODER::set (raw, v);
      return *this;
    }
    bool
    en_latch_encoder () const
    {
      return EN_LATCH_ENCODER::get (raw);
    }

    SwMode &
    sg_stop (bool v)
    {
      SG_STOP::set (raw, v);
      return *this;
    }
    bool
    sg_stop () const
    {
      return SG_STOP::get (raw);
    }

    SwMode &
    en_softstop (bool v)
    {
      EN_SOFTSTOP::set (raw, v);
      return *this;
    }
    bool
    en_softstop () const
    {
      return EN_SOFTSTOP::get (raw);
    }
  };

  struct RampStat
  {
    uint32_t raw{ 0 };

    using STATUS_STOP_L = tmc::bits::Bit<0>;
    using STATUS_STOP_R = tmc::bits::Bit<1>;
    using STATUS_LATCH_L = tmc::bits::Bit<2>;
    using STATUS_LATCH_R = tmc::bits::Bit<3>;
    using EVENT_STOP_L = tmc::bits::Bit<4>;
    using EVENT_STOP_R = tmc::bits::Bit<5>;
    using EVENT_STOP_SG = tmc::bits::Bit<6>;
    using EVENT_POS_REACHED = tmc::bits::Bit<7>;
    using VELOCITY_REACHED = tmc::bits::Bit<8>;
    using POSITION_REACHED = tmc::bits::Bit<9>;
    using VZERO = tmc::bits::Bit<10>;
    using T_ZEROWAIT_ACTIVE = tmc::bits::Bit<11>;
    using SECOND_MOVE = tmc::bits::Bit<12>;
    using STATUS_SG = tmc::bits::Bit<13>;

    RampStat &
    status_stop_l (bool v)
    {
      STATUS_STOP_L::set (raw, v);
      return *this;
    }
    bool
    status_stop_l () const
    {
      return STATUS_STOP_L::get (raw);
    }

    RampStat &
    status_stop_r (bool v)
    {
      STATUS_STOP_R::set (raw, v);
      return *this;
    }
    bool
    status_stop_r () const
    {
      return STATUS_STOP_R::get (raw);
    }

    RampStat &
    status_latch_l (bool v)
    {
      STATUS_LATCH_L::set (raw, v);
      return *this;
    }
    bool
    status_latch_l () const
    {
      return STATUS_LATCH_L::get (raw);
    }

    RampStat &
    status_latch_r (bool v)
    {
      STATUS_LATCH_R::set (raw, v);
      return *this;
    }
    bool
    status_latch_r () const
    {
      return STATUS_LATCH_R::get (raw);
    }

    RampStat &
    event_stop_l (bool v)
    {
      EVENT_STOP_L::set (raw, v);
      return *this;
    }
    bool
    event_stop_l () const
    {
      return EVENT_STOP_L::get (raw);
    }

    RampStat &
    event_stop_r (bool v)
    {
      EVENT_STOP_R::set (raw, v);
      return *this;
    }
    bool
    event_stop_r () const
    {
      return EVENT_STOP_R::get (raw);
    }

    RampStat &
    event_stop_sg (bool v)
    {
      EVENT_STOP_SG::set (raw, v);
      return *this;
    }
    bool
    event_stop_sg () const
    {
      return EVENT_STOP_SG::get (raw);
    }

    RampStat &
    event_pos_reached (bool v)
    {
      EVENT_POS_REACHED::set (raw, v);
      return *this;
    }
    bool
    event_pos_reached () const
    {
      return EVENT_POS_REACHED::get (raw);
    }

    RampStat &
    velocity_reached (bool v)
    {
      VELOCITY_REACHED::set (raw, v);
      return *this;
    }
    bool
    velocity_reached () const
    {
      return VELOCITY_REACHED::get (raw);
    }

    RampStat &
    position_reached (bool v)
    {
      POSITION_REACHED::set (raw, v);
      return *this;
    }
    bool
    position_reached () const
    {
      return POSITION_REACHED::get (raw);
    }

    RampStat &
    vzero (bool v)
    {
      VZERO::set (raw, v);
      return *this;
    }
    bool
    vzero () const
    {
      return VZERO::get (raw);
    }

    RampStat &
    t_zerowait_active (bool v)
    {
      T_ZEROWAIT_ACTIVE::set (raw, v);
      return *this;
    }
    bool
    t_zerowait_active () const
    {
      return T_ZEROWAIT_ACTIVE::get (raw);
    }

    RampStat &
    second_move (bool v)
    {
      SECOND_MOVE::set (raw, v);
      return *this;
    }
    bool
    second_move () const
    {
      return SECOND_MOVE::get (raw);
    }

    RampStat &
    status_sg (bool v)
    {
      STATUS_SG::set (raw, v);
      return *this;
    }
    bool
    status_sg () const
    {
      return STATUS_SG::get (raw);
    }
  };

  struct Encmode
  {
    uint32_t raw{ 0 };

    using POL_A = tmc::bits::Bit<0>;
    using POL_B = tmc::bits::Bit<1>;
    using POL_N = tmc::bits::Bit<2>;
    using IGNORE_AB = tmc::bits::Bit<3>;
    using CLR_CONT = tmc::bits::Bit<4>;
    using CLR_ONCE = tmc::bits::Bit<5>;
    using POS_EDGE = tmc::bits::Bit<6>;
    using NEG_EDGE = tmc::bits::Bit<7>;
    using CLR_ENC_X = tmc::bits::Bit<8>;
    using LATCH_X_ACT = tmc::bits::Bit<9>;
    using ENC_SEL_DECIMAL = tmc::bits::Bit<10>;

    Encmode &
    pol_a (bool v)
    {
      POL_A::set (raw, v);
      return *this;
    }
    bool
    pol_a () const
    {
      return POL_A::get (raw);
    }

    Encmode &
    pol_b (bool v)
    {
      POL_B::set (raw, v);
      return *this;
    }
    bool
    pol_b () const
    {
      return POL_B::get (raw);
    }

    Encmode &
    pol_n (bool v)
    {
      POL_N::set (raw, v);
      return *this;
    }
    bool
    pol_n () const
    {
      return POL_N::get (raw);
    }

    Encmode &
    ignore_ab (bool v)
    {
      IGNORE_AB::set (raw, v);
      return *this;
    }
    bool
    ignore_ab () const
    {
      return IGNORE_AB::get (raw);
    }

    Encmode &
    clr_cont (bool v)
    {
      CLR_CONT::set (raw, v);
      return *this;
    }
    bool
    clr_cont () const
    {
      return CLR_CONT::get (raw);
    }

    Encmode &
    clr_once (bool v)
    {
      CLR_ONCE::set (raw, v);
      return *this;
    }
    bool
    clr_once () const
    {
      return CLR_ONCE::get (raw);
    }

    Encmode &
    pos_edge (bool v)
    {
      POS_EDGE::set (raw, v);
      return *this;
    }
    bool
    pos_edge () const
    {
      return POS_EDGE::get (raw);
    }

    Encmode &
    neg_edge (bool v)
    {
      NEG_EDGE::set (raw, v);
      return *this;
    }
    bool
    neg_edge () const
    {
      return NEG_EDGE::get (raw);
    }

    Encmode &
    clr_enc_x (bool v)
    {
      CLR_ENC_X::set (raw, v);
      return *this;
    }
    bool
    clr_enc_x () const
    {
      return CLR_ENC_X::get (raw);
    }

    Encmode &
    latch_x_act (bool v)
    {
      LATCH_X_ACT::set (raw, v);
      return *this;
    }
    bool
    latch_x_act () const
    {
      return LATCH_X_ACT::get (raw);
    }

    Encmode &
    enc_sel_decimal (bool v)
    {
      ENC_SEL_DECIMAL::set (raw, v);
      return *this;
    }
    bool
    enc_sel_decimal () const
    {
      return ENC_SEL_DECIMAL::get (raw);
    }
  };

  struct EncConst
  {
    uint32_t raw{ 0 };

    using FRACTIONAL = tmc::bits::Field<0, 16>;
    using INTEGER = tmc::bits::Field<16, 16>;

    EncConst &
    fractional (uint16_t v)
    {
      FRACTIONAL::set (raw, v);
      return *this;
    }
    uint16_t
    fractional () const
    {
      return static_cast<uint16_t> (FRACTIONAL::get (raw));
    }

    EncConst &
    integer (int16_t v)
    {
      INTEGER::set (raw, static_cast<uint32_t> (static_cast<uint16_t> (v)));
      return *this;
    }
    int16_t
    integer () const
    {
      return static_cast<int16_t> (INTEGER::get (raw));
    }
  };

  struct EncStatus
  {
    uint32_t raw{ 0 };

    using N_EVENT = tmc::bits::Bit<0>;
    using DEVIATION_WARN = tmc::bits::Bit<1>;

    EncStatus &
    n_event (bool v)
    {
      N_EVENT::set (raw, v);
      return *this;
    }
    bool
    n_event () const
    {
      return N_EVENT::get (raw);
    }

    EncStatus &
    deviation_warn (bool v)
    {
      DEVIATION_WARN::set (raw, v);
      return *this;
    }
    bool
    deviation_warn () const
    {
      return DEVIATION_WARN::get (raw);
    }
  };

  struct Mslutstart
  {
    uint32_t raw{ 0 };

    using START_SIN = tmc::bits::Field<0, 8>;
    using START_SIN90 = tmc::bits::Field<16, 8>;

    Mslutstart &
    start_sin (uint8_t v)
    {
      START_SIN::set (raw, v);
      return *this;
    }
    uint8_t
    start_sin () const
    {
      return static_cast<uint8_t> (START_SIN::get (raw));
    }

    Mslutstart &
    start_sin90 (uint8_t v)
    {
      START_SIN90::set (raw, v);
      return *this;
    }
    uint8_t
    start_sin90 () const
    {
      return static_cast<uint8_t> (START_SIN90::get (raw));
    }
  };

  struct Mscuract
  {
    uint32_t raw{ 0 };

    using CUR_B = tmc::bits::Field<0, 9>;
    using CUR_A = tmc::bits::Field<16, 9>;

    Mscuract &
    cur_b (uint16_t v)
    {
      CUR_B::set (raw, v);
      return *this;
    }
    uint16_t
    cur_b () const
    {
      return static_cast<uint16_t> (CUR_B::get (raw));
    }

    Mscuract &
    cur_a (uint16_t v)
    {
      CUR_A::set (raw, v);
      return *this;
    }
    uint16_t
    cur_a () const
    {
      return static_cast<uint16_t> (CUR_A::get (raw));
    }
  };
  struct Chopconf
  {
    uint32_t raw{ 0 };

    using TOFF = tmc::bits::Field<0, 4>;
    using HSTART = tmc::bits::Field<4, 3>;
    using HEND = tmc::bits::Field<7, 4>;
    using FD3 = tmc::bits::Bit<11>;
    using DISFDCC = tmc::bits::Bit<12>;
    using CHM = tmc::bits::Bit<14>;
    using TBL = tmc::bits::Field<15, 2>;
    using VHIGHFS = tmc::bits::Bit<18>;
    using VHIGHCHM = tmc::bits::Bit<19>;
    using TPFD = tmc::bits::Field<20, 4>;
    using MRES = tmc::bits::Field<24, 4>;
    using INTERPOLATION = tmc::bits::Bit<28>;
    using DOUBLE_EDGE = tmc::bits::Bit<29>;
    using DISS2G = tmc::bits::Bit<30>;
    using DISS2VS = tmc::bits::Bit<31>;

    Chopconf &
    toff (uint8_t v)
    {
      TOFF::set (raw, v);
      return *this;
    }
    uint8_t
    toff () const
    {
      return static_cast<uint8_t> (TOFF::get (raw));
    }

    Chopconf &
    hstart (uint8_t v)
    {
      HSTART::set (raw, v);
      return *this;
    }
    uint8_t
    hstart () const
    {
      return static_cast<uint8_t> (HSTART::get (raw));
    }

    Chopconf &
    hend (uint8_t v)
    {
      HEND::set (raw, v);
      return *this;
    }
    uint8_t
    hend () const
    {
      return static_cast<uint8_t> (HEND::get (raw));
    }

    Chopconf &
    fd3 (bool v)
    {
      FD3::set (raw, v);
      return *this;
    }
    bool
    fd3 () const
    {
      return FD3::get (raw);
    }

    Chopconf &
    disfdcc (bool v)
    {
      DISFDCC::set (raw, v);
      return *this;
    }
    bool
    disfdcc () const
    {
      return DISFDCC::get (raw);
    }

    Chopconf &
    chm (bool v)
    {
      CHM::set (raw, v);
      return *this;
    }
    bool
    chm () const
    {
      return CHM::get (raw);
    }

    Chopconf &
    tbl (uint8_t v)
    {
      TBL::set (raw, v);
      return *this;
    }
    uint8_t
    tbl () const
    {
      return static_cast<uint8_t> (TBL::get (raw));
    }

    Chopconf &
    vhighfs (bool v)
    {
      VHIGHFS::set (raw, v);
      return *this;
    }
    bool
    vhighfs () const
    {
      return VHIGHFS::get (raw);
    }

    Chopconf &
    vhighchm (bool v)
    {
      VHIGHCHM::set (raw, v);
      return *this;
    }
    bool
    vhighchm () const
    {
      return VHIGHCHM::get (raw);
    }

    Chopconf &
    tpfd (uint8_t v)
    {
      TPFD::set (raw, v);
      return *this;
    }
    uint8_t
    tpfd () const
    {
      return static_cast<uint8_t> (TPFD::get (raw));
    }

    Chopconf &
    mres (uint8_t v)
    {
      MRES::set (raw, v);
      return *this;
    }
    uint8_t
    mres () const
    {
      return static_cast<uint8_t> (MRES::get (raw));
    }

    Chopconf &
    interpolation (bool v)
    {
      INTERPOLATION::set (raw, v);
      return *this;
    }
    bool
    interpolation () const
    {
      return INTERPOLATION::get (raw);
    }

    Chopconf &
    double_edge (bool v)
    {
      DOUBLE_EDGE::set (raw, v);
      return *this;
    }
    bool
    double_edge () const
    {
      return DOUBLE_EDGE::get (raw);
    }

    Chopconf &
    diss2g (bool v)
    {
      DISS2G::set (raw, v);
      return *this;
    }
    bool
    diss2g () const
    {
      return DISS2G::get (raw);
    }

    Chopconf &
    diss2vs (bool v)
    {
      DISS2VS::set (raw, v);
      return *this;
    }
    bool
    diss2vs () const
    {
      return DISS2VS::get (raw);
    }
  };

  struct Coolconf
  {
    uint32_t raw{ 0 };

    using SEMIN = tmc::bits::Field<0, 4>;
    using SEUP = tmc::bits::Field<5, 2>;
    using SEMAX = tmc::bits::Field<8, 4>;
    using SEDN = tmc::bits::Field<13, 2>;
    using SEIMIN = tmc::bits::Bit<15>;
    using SGT = tmc::bits::Field<16, 7>;
    using SFILT = tmc::bits::Bit<24>;

    Coolconf &
    semin (uint8_t v)
    {
      SEMIN::set (raw, v);
      return *this;
    }
    uint8_t
    semin () const
    {
      return static_cast<uint8_t> (SEMIN::get (raw));
    }

    Coolconf &
    seup (uint8_t v)
    {
      SEUP::set (raw, v);
      return *this;
    }
    uint8_t
    seup () const
    {
      return static_cast<uint8_t> (SEUP::get (raw));
    }

    Coolconf &
    semax (uint8_t v)
    {
      SEMAX::set (raw, v);
      return *this;
    }
    uint8_t
    semax () const
    {
      return static_cast<uint8_t> (SEMAX::get (raw));
    }

    Coolconf &
    sedn (uint8_t v)
    {
      SEDN::set (raw, v);
      return *this;
    }
    uint8_t
    sedn () const
    {
      return static_cast<uint8_t> (SEDN::get (raw));
    }

    Coolconf &
    seimin (bool v)
    {
      SEIMIN::set (raw, v);
      return *this;
    }
    bool
    seimin () const
    {
      return SEIMIN::get (raw);
    }

    Coolconf &
    sgt (int8_t v)
    {
      SGT::set (raw, static_cast<uint32_t> (static_cast<uint8_t> (v)));
      return *this;
    }
    uint8_t
    sgt () const
    {
      return static_cast<uint8_t> (SGT::get (raw));
    }

    Coolconf &
    sfilt (bool v)
    {
      SFILT::set (raw, v);
      return *this;
    }
    bool
    sfilt () const
    {
      return SFILT::get (raw);
    }
  };

  struct Dcctrl
  {
    uint32_t raw{ 0 };

    using DC_TIME = tmc::bits::Field<0, 10>;
    using DC_SG = tmc::bits::Field<16, 8>;

    Dcctrl &
    dc_time (uint16_t v)
    {
      DC_TIME::set (raw, v);
      return *this;
    }
    uint16_t
    dc_time () const
    {
      return static_cast<uint16_t> (DC_TIME::get (raw));
    }

    Dcctrl &
    dc_sg (uint8_t v)
    {
      DC_SG::set (raw, v);
      return *this;
    }
    uint8_t
    dc_sg () const
    {
      return static_cast<uint8_t> (DC_SG::get (raw));
    }
  };

  struct DrvStatus
  {
    uint32_t raw{ 0 };

    using SG_RESULT = tmc::bits::Field<0, 10>;
    using S2VSA = tmc::bits::Bit<12>;
    using S2VSB = tmc::bits::Bit<13>;
    using STEALTH = tmc::bits::Bit<14>;
    using FSACTIVE = tmc::bits::Bit<15>;
    using CS_ACTUAL = tmc::bits::Field<16, 5>;
    using STALLGUARD = tmc::bits::Bit<24>;
    using OT = tmc::bits::Bit<25>;
    using OTPW = tmc::bits::Bit<26>;
    using S2GA = tmc::bits::Bit<27>;
    using S2GB = tmc::bits::Bit<28>;
    using OLA = tmc::bits::Bit<29>;
    using OLB = tmc::bits::Bit<30>;
    using STST = tmc::bits::Bit<31>;

    DrvStatus &
    sg_result (uint16_t v)
    {
      SG_RESULT::set (raw, v);
      return *this;
    }
    uint16_t
    sg_result () const
    {
      return static_cast<uint16_t> (SG_RESULT::get (raw));
    }

    DrvStatus &
    s2vsa (bool v)
    {
      S2VSA::set (raw, v);
      return *this;
    }
    bool
    s2vsa () const
    {
      return S2VSA::get (raw);
    }

    DrvStatus &
    s2vsb (bool v)
    {
      S2VSB::set (raw, v);
      return *this;
    }
    bool
    s2vsb () const
    {
      return S2VSB::get (raw);
    }

    DrvStatus &
    stealth (bool v)
    {
      STEALTH::set (raw, v);
      return *this;
    }
    bool
    stealth () const
    {
      return STEALTH::get (raw);
    }

    DrvStatus &
    fsactive (bool v)
    {
      FSACTIVE::set (raw, v);
      return *this;
    }
    bool
    fsactive () const
    {
      return FSACTIVE::get (raw);
    }

    DrvStatus &
    cs_actual (uint8_t v)
    {
      CS_ACTUAL::set (raw, v);
      return *this;
    }
    uint8_t
    cs_actual () const
    {
      return static_cast<uint8_t> (CS_ACTUAL::get (raw));
    }

    DrvStatus &
    stallguard (bool v)
    {
      STALLGUARD::set (raw, v);
      return *this;
    }
    bool
    stallguard () const
    {
      return STALLGUARD::get (raw);
    }

    DrvStatus &
    ot (bool v)
    {
      OT::set (raw, v);
      return *this;
    }
    bool
    ot () const
    {
      return OT::get (raw);
    }

    DrvStatus &
    otpw (bool v)
    {
      OTPW::set (raw, v);
      return *this;
    }
    bool
    otpw () const
    {
      return OTPW::get (raw);
    }

    DrvStatus &
    s2ga (bool v)
    {
      S2GA::set (raw, v);
      return *this;
    }
    bool
    s2ga () const
    {
      return S2GA::get (raw);
    }

    DrvStatus &
    s2gb (bool v)
    {
      S2GB::set (raw, v);
      return *this;
    }
    bool
    s2gb () const
    {
      return S2GB::get (raw);
    }

    DrvStatus &
    ola (bool v)
    {
      OLA::set (raw, v);
      return *this;
    }
    bool
    ola () const
    {
      return OLA::get (raw);
    }

    DrvStatus &
    olb (bool v)
    {
      OLB::set (raw, v);
      return *this;
    }
    bool
    olb () const
    {
      return OLB::get (raw);
    }

    DrvStatus &
    stst (bool v)
    {
      STST::set (raw, v);
      return *this;
    }
    bool
    stst () const
    {
      return STST::get (raw);
    }
  };
  struct Pwmconf
  {
    uint32_t raw{ 0 };

    using PWM_OFS = tmc::bits::Field<0, 8>;
    using PWM_GRAD = tmc::bits::Field<8, 8>;
    using PWM_FREQ = tmc::bits::Field<16, 2>;
    using PWM_AUTOSCALE = tmc::bits::Bit<18>;
    using PWM_AUTOGRAD = tmc::bits::Bit<19>;
    using FREEWHEEL = tmc::bits::Field<20, 2>;
    using PWM_REG = tmc::bits::Field<24, 4>;
    using PWM_LIM = tmc::bits::Field<28, 4>;

    Pwmconf &
    pwm_ofs (uint8_t v)
    {
      PWM_OFS::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_ofs () const
    {
      return static_cast<uint8_t> (PWM_OFS::get (raw));
    }

    Pwmconf &
    pwm_grad (uint8_t v)
    {
      PWM_GRAD::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_grad () const
    {
      return static_cast<uint8_t> (PWM_GRAD::get (raw));
    }

    Pwmconf &
    pwm_freq (uint8_t v)
    {
      PWM_FREQ::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_freq () const
    {
      return static_cast<uint8_t> (PWM_FREQ::get (raw));
    }

    Pwmconf &
    pwm_autoscale (bool v)
    {
      PWM_AUTOSCALE::set (raw, v);
      return *this;
    }
    bool
    pwm_autoscale () const
    {
      return PWM_AUTOSCALE::get (raw);
    }

    Pwmconf &
    pwm_autograd (bool v)
    {
      PWM_AUTOGRAD::set (raw, v);
      return *this;
    }
    bool
    pwm_autograd () const
    {
      return PWM_AUTOGRAD::get (raw);
    }

    Pwmconf &
    freewheel (uint8_t v)
    {
      FREEWHEEL::set (raw, v);
      return *this;
    }
    uint8_t
    freewheel () const
    {
      return static_cast<uint8_t> (FREEWHEEL::get (raw));
    }

    Pwmconf &
    pwm_reg (uint8_t v)
    {
      PWM_REG::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_reg () const
    {
      return static_cast<uint8_t> (PWM_REG::get (raw));
    }

    Pwmconf &
    pwm_lim (uint8_t v)
    {
      PWM_LIM::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_lim () const
    {
      return static_cast<uint8_t> (PWM_LIM::get (raw));
    }
  };

  struct PwmScale
  {
    uint32_t raw{ 0 };

    using PWM_SCALE_SUM = tmc::bits::Field<0, 8>;
    using PWM_SCALE_AUTO = tmc::bits::Field<16, 9>;

    PwmScale &
    pwm_scale_sum (uint8_t v)
    {
      PWM_SCALE_SUM::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_scale_sum () const
    {
      return static_cast<uint8_t> (PWM_SCALE_SUM::get (raw));
    }

    PwmScale &
    pwm_scale_auto (uint16_t v)
    {
      PWM_SCALE_AUTO::set (raw, v);
      return *this;
    }
    uint16_t
    pwm_scale_auto () const
    {
      return static_cast<uint16_t> (PWM_SCALE_AUTO::get (raw));
    }
  };

  struct PwmAuto
  {
    uint32_t raw{ 0 };

    using PWM_OFS_AUTO = tmc::bits::Field<0, 8>;
    using PWM_GRAD_AUTO = tmc::bits::Field<16, 8>;

    PwmAuto &
    pwm_ofs_auto (uint8_t v)
    {
      PWM_OFS_AUTO::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_ofs_auto () const
    {
      return static_cast<uint8_t> (PWM_OFS_AUTO::get (raw));
    }

    PwmAuto &
    pwm_grad_auto (uint8_t v)
    {
      PWM_GRAD_AUTO::set (raw, v);
      return *this;
    }
    uint8_t
    pwm_grad_auto () const
    {
      return static_cast<uint8_t> (PWM_GRAD_AUTO::get (raw));
    }
  };

private:
  Interface *interface_ptr_;

  uint32_t stored_[AddressCount] = { 0 };
  bool writeable_[AddressCount] = { false };
  bool readable_[AddressCount] = { false };

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  void initialize (Interface &interface);

  friend class ::TMC51X0;
};

}
#endif
