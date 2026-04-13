#pragma once

#include "Globals.h"

// DMP Version
void initFingerChannel(FingerChannel& fc, bool (&status)[4]);

// No DMP Version
// void initFingerChannel(FingerChannel& fc);

void tcaSelectChannel(uint8_t ch);