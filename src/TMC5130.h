// ----------------------------------------------------------------------------
// TMC5130.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC5130_H
#define TMC5130_H
#include <Arduino.h>


class TMC5130
{
public:
  TMC5130();
  uint8_t getVersion();
private:
};

#endif
