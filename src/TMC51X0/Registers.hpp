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
  union GlobalConfig
  {
    struct
    {
      uint32_t i_scale_analog : 1;
      uint32_t internal_rsense : 1;
      uint32_t enable_spread_cycle : 1;
      uint32_t shaft : 1;
      uint32_t index_otpw : 1;
      uint32_t index_step : 1;
      uint32_t pdn_disable : 1;
      uint32_t mstep_reg_select : 1;
      uint32_t multistep_filt : 1;
      uint32_t test_mode : 1;
      uint32_t reserved : 22;
    };
    uint32_t bytes;
  };
  GlobalConfig global_config_;

  const static uint8_t ADDRESS_INPUT = 0x04;

  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  friend class TMC51X0;
};

#endif
