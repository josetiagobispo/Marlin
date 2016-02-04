/**
 * RAMPS-SMART
 */

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN     54  // A0
#define X_DIR_PIN      55  // A1
#define X_MIN_PIN      3
#define X_MAX_PIN      2
#define X_ENABLE_PIN   38

#define Y_STEP_PIN     60  // A6 
#define Y_DIR_PIN      61  // A7
#define Y_MIN_PIN      14
#define Y_MAX_PIN      15
#define Y_ENABLE_PIN   56  // A2

#define Z_STEP_PIN     46
#define Z_DIR_PIN      48
#define Z_MIN_PIN      18
#define Z_MAX_PIN      19
#define Z_ENABLE_PIN   62  // A8

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN   10
#define TEMP_0_PIN     11  // Due analog pin #
#define HEATER_1_PIN   8
#define TEMP_1_PIN     12  // Due analog pin #
#define HEATER_2_PIN   9
#define TEMP_2_PIN     13  // Due analog pin #

#define E0_STEP_PIN    26
#define E0_DIR_PIN     28
#define E0_ENABLE_PIN  24

#define E1_STEP_PIN    36
#define E1_DIR_PIN     34
#define E1_ENABLE_PIN  30

#define SDPOWER 	   -1
#define SDSS		   53 // 10 if using HW SPI. 53 if using SW SPI
#define LED_PIN 	   13
#define FAN_PIN 	   9
#define PS_ON_PIN      12
#define KILL_PIN	   -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

