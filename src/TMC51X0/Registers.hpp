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
      uint32_t test_mode : 1;
      uint32_t reserved : 14;
    };
    uint32_t bytes;
  };

  union Input
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
  Input readInput();
private:
  Interface interface_;

  void setup(SPIClass & spi,
    size_t chip_select_pin);

  // General Configuration Registers
  const static uint8_t ADDRESS_GCONF = 0x00;
  GlobalConfig global_config_;

  const static uint8_t ADDRESS_INPUT = 0x04;

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  friend class TMC51X0;
};

#endif
