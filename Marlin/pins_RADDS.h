/**
 * RADDS
 */

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN         24
#define X_DIR_PIN          23
#define X_ENABLE_PIN       26

#define Y_STEP_PIN         17
#define Y_DIR_PIN          16
#define Y_ENABLE_PIN       22

#define Z_STEP_PIN          2
#define Z_DIR_PIN           3
#define Z_ENABLE_PIN       15

#define X_MIN_PIN          28
#define X_MAX_PIN          34
#define Y_MIN_PIN          30
#define Y_MAX_PIN          36
#define Z_MIN_PIN          32
#define Z_MAX_PIN          38

#define E0_STEP_PIN        61
#define E0_DIR_PIN         60
#define E0_ENABLE_PIN      62

#define E1_STEP_PIN        64
#define E1_DIR_PIN         63
#define E1_ENABLE_PIN      65

#define E2_STEP_PIN        51
#define E2_DIR_PIN         53
#define E2_ENABLE_PIN      49

#define E3_STEP_PIN        35
#define E3_DIR_PIN         33
#define E3_ENABLE_PIN      37

#define E4_STEP_PIN        29
#define E4_DIR_PIN         27
#define E4_ENABLE_PIN      31

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 -1

#define BEEPER_PIN              41

#define FAN_PIN 	           9
#define FAN2_PIN            8

#define PS_ON_PIN          40

#define KILL_PIN                -1

#define HEATER_BED_PIN      7    // BED
#define HEATER_0_PIN       13
#define HEATER_1_PIN       12
#define HEATER_2_PIN       11

#define TEMP_BED_PIN        4   // ANALOG NUMBERING
#define TEMP_0_PIN          0   // ANALOG NUMBERING
#define TEMP_1_PIN          1   // ANALOG NUMBERING
#define TEMP_2_PIN          2   // ANALOG NUMBERING
#define TEMP_3_PIN         -1  // 3   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN           5
  #if NUM_SERVOS > 1
    #define SERVO1_PIN         6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      39
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    40
      #endif
    #endif
  #endif
#endif

#if ENABLED(ULTRA_LCD)
  // RADDS LCD panel
  #if ENABLED(RADDS_DISPLAY)
    #define LCD_PINS_RS 		42
    #define LCD_PINS_ENABLE 43
    #define LCD_PINS_D4 		44
    #define LCD_PINS_D5 		45
    #define LCD_PINS_D6 		46
    #define LCD_PINS_D7 		47

    #define BEEPER          41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
			
    #define BTN_BACK        71
    
    #undef SDSS
    #define SDSS            10
    #define SDCARDDETECT    14

  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define LCD_PINS_RS     46
    #define LCD_PINS_ENABLE 47
    #define LCD_PINS_D4     44

    #define BEEPER_PIN      41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

  #elif ENABLED(SSD1306_OLED_I2C_CONTROLLER)
    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
    #define BEEPER          41
    #define LCD_SDSS        10
    #define SDCARDDETECT    14
    #define KILL_PIN        -1

  #elif ENABLED(SPARK_FULL_GRAPHICS)
    #define LCD_PINS_D4     29
    #define LCD_PINS_ENABLE 27
    #define LCD_PINS_RS     25

    #define BTN_EN1         35
    #define BTN_EN2         33
    #define BTN_ENC         37

    #define KILL_PIN        -1
    #undef BEEPER
    #define BEEPER          -1
  #endif // SPARK_FULL_GRAPHICS
#endif // ULTRA_LCD


// SPI for Max6675 Thermocouple

// works with radds???
#if DISABLED(SDSUPPORT)
  // these pins are defined in the SD library if building with SD support
  #define SCK_PIN          52
  #define MISO_PIN         50
  #define MOSI_PIN         51
  #define MAX6675_SS       53
 else
  #define MAX6675_SS       49
#endif

