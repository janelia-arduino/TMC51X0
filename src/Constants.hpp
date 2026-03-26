// ----------------------------------------------------------------------------
// Constants.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONSTANTS_HPP
#define TMC51X0_CONSTANTS_HPP

#include <stddef.h>

namespace tmc51x0 {
// Sentinel value for "no pin" / "unused pin" parameters.
//
// We use size_t(-1) rather than a small constant like 255 so it can never
// collide with a valid GPIO number on platforms with large pin counts.
constexpr size_t NO_PIN = static_cast<size_t>(-1);
}

#endif // TMC51X0_CONSTANTS_HPP
