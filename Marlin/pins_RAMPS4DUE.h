/**
 * Arduino Mega or Due with RAMPS4DUE pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS4DUE_EFB (Extruder, Fan, Bed)
 *  RAMPS4DUE_EEB (Extruder, Extruder, Bed)
 *  RAMPS4DUE_EFF (Extruder, Fan, Fan)
 *  RAMPS4DUE_EEF (Extruder, Extruder, Fan)
 *  RAMPS4DUE_SF  (Spindle, Controller Fan)
 *
 *  Differences between
 *  RAMPS_14 | RAMPS4DUE
 *       A13 | A9/D63 (shares the same pin with AUX2_4PIN)
 *       A14 | A10/D64 (shares the same pin with AUX2_5PIN)
 *       A15 | NC
 */

#if !defined(__SAM3X8E__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Due' or 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define IS_RAMPS4DUE

#include "pins_RAMPS_14.h"

#undef TEMP_0_PIN
#define TEMP_0_PIN          9   // ANALOG NUMBERING

#undef TEMP_1_PIN
#define TEMP_1_PIN         -1   // ANALOG NUMBERING

#undef TEMP_2_PIN
#define TEMP_2_PIN         -1   // ANALOG NUMBERING

#undef TEMP_BED_PIN
#define TEMP_BED_PIN       10   // ANALOG NUMBERING
