// ----------------------------------------------------------------------------
// Result.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_RESULT_HPP
#define TMC51X0_RESULT_HPP

namespace tmc51x0
{
// UART-specific error codes. Intended to be surfaced for debugging and for
// event-loop / QP-friendly non-blocking APIs.
enum class UartError
{
  None = 0,
  Busy,
  NotInitialized,
  ReplyTimeout,
  CrcMismatch,
  UnexpectedFrame,
  RxGarbage,
};

template <typename T>
struct Result
{
  T value{};
  UartError error{ UartError::None };
  constexpr bool
  ok () const
  {
    return error == UartError::None;
  }
};

// Specialization for "no value".
template <>
struct Result<void>
{
  UartError error{ UartError::None };
  constexpr bool
  ok () const
  {
    return error == UartError::None;
  }
};
} // namespace tmc51x0

#endif // TMC51X0_RESULT_HPP
