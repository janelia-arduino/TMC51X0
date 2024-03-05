// ----------------------------------------------------------------------------
// Controller.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONTROLLER_HPP
#define TMC51X0_CONTROLLER_HPP

#include "TMC51X0/Constants.hpp"
#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
class Controller
{
public:
private:
  Registers * registers_ptr_;

  void setup(Registers & registers);

  friend class ::TMC51X0;
};
}
#endif
