#include "Globals.h"
#include <Arduino.h>

struct FingerChannel {
  const int tca_channel;
  String label;
  const int adc_channel[2];
};