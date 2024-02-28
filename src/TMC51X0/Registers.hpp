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
  union GlobalConfig
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
  GlobalConfig readGlobalConfig();
  void writeGlobalConfig(GlobalConfig global_config);
  GlobalConfig getStoredGlobalConfig();

  union Inputs
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
  Inputs readInputs();

  union ChopperConfig
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
  ChopperConfig readChopperConfig();
  void writeChopperConfig(ChopperConfig chopper_config);
  ChopperConfig getStoredChopperConfig();

private:
  Interface interface_;

  GlobalConfig global_config_;
  ChopperConfig chopper_config_;

  void setup(SPIClass & spi,
    size_t chip_select_pin);

  const static uint8_t ADDRESS_GLOBAL_CONFIG = 0x00;
  const static uint8_t ADDRESS_INPUTS = 0x04;
  const static uint8_t ADDRESS_CHOPPER_CONFIG = 0x6C;

  const static uint32_t DEFAULT_CHOPPER_CONFIG = 0x10410150;

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  friend class ::TMC51X0;
};
}
#endif
