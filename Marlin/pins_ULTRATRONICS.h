/**
 * ULTRATRONICS
 */

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

// X AXIS
#define X_STEP_PIN       35  
#define X_DIR_PIN        34  
#define X_ENABLE_PIN     37  
#define X_MIN_PIN        -1  
#define X_MAX_PIN        -1  
#define X_MS1_PIN             -1  

// Y AXIS
#define Y_STEP_PIN       22  
#define Y_DIR_PIN        23  
#define Y_ENABLE_PIN     9  
#define Y_MIN_PIN        -1  
#define Y_MAX_PIN        -1  
#define Y_MS1_PIN             -1  

// Z AXIS
#define Z_STEP_PIN       25  
#define Z_DIR_PIN        26  
#define Z_ENABLE_PIN     24  
#define Z_MIN_PIN        -1  
#define Z_MAX_PIN        -1  
#define Z_MS1_PIN             -1  

// E0 AXIS
#define E0_STEP_PIN      47  
#define E0_DIR_PIN       46  
#define E0_ENABLE_PIN    48  
#define E0_MS1_PIN            -1  

// E1 AXIS
#define E1_STEP_PIN      44  
#define E1_DIR_PIN       36  
#define E1_ENABLE_PIN    45  

// E2 AXIS
#define E2_STEP_PIN      42 
#define E2_DIR_PIN       41 
#define E2_ENABLE_PIN    43 

// E3 AXIS
#define E3_STEP_PIN      39 
#define E3_DIR_PIN       38 
#define E3_ENABLE_PIN    40 

#define MOTOR_FAULT_PIN       -1 

#define SDPOWER               -1
#define SDSS                  -1 
#define SD_DETECT_PIN         -1 
#define LED_PIN               -1

#define FAN_PIN          -1 
#define FAN2_PIN              -1 

#define PS_ON_PIN        -1
#define KILL_PIN              -1
#define SUICIDE_PIN           -1 //PIN that has to be turned on right after start, to keep power flowing.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_BED_PIN   69 
#define HEATER_0_PIN     68 
#define HEATER_1_PIN      8 
#define HEATER_2_PIN      9 
#define HEATER_3_PIN     97 

#define TEMP_BED_PIN      0 
#define TEMP_0_PIN        1 
#define TEMP_1_PIN       52 
#define TEMP_2_PIN       51 
#define TEMP_3_PIN       50 

#define LED_RED_PIN           40 
#define LED_GREEN_PIN         41 
#define CASE_LIGHTS_PIN       36 

#define EXP_VOLTAGE_LEVEL_PIN 65

#define DAC0_SYNC             53 
#define DAC1_SYNC              6 

//64K SPI EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25 

//2K SPI EEPROM
#define SPI_EEPROM2_CS        26 

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS          23 

/** Display **/

// GLCD on expansion port
#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)

  #define LCD_PINS_RS         18
  #define LCD_PINS_ENABLE     15
  #define LCD_PINS_D4         19
  #define BEEPER_PIN          64

  #define BTN_EN1             14
  #define BTN_EN2             16
  #define BTN_ENC             17
  
  #if UI_VOLTAGE_LEVEL != 1
    #undef UI_VOLTAGE_LEVEL
    #define UI_VOLTAGE_LEVEL  1
  #endif
     
#endif //REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

#if NUM_SERVOS > 0
  #define SERVO0_PIN          -1
  #if NUM_SERVOS > 1
    #define SERVO1_PIN        -1
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      -1
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    -1
      #endif
    #endif
  #endif
#endif
