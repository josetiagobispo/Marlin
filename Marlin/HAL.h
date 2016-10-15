/*
 Contributors:
    Copyright (c) 2015 Nico Tonnhofer wurstnase.reprap@gmail.com
*/
/* **************************************************************************
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

// **************************************************************************
//
// Description:          *** HAL for Arduino Due ***
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef _HAL_H
  #define _HAL_H

  #include <stdint.h>
  #include "Arduino.h"
#if 0
  #include "types.h"
#endif
  #include "fastio.h"

  // --------------------------------------------------------------------------
  // Defines
  // --------------------------------------------------------------------------

  #define MAX_ANALOG_PIN_NUMBER 11

  #define FORCE_INLINE __attribute__((always_inline)) inline

  #define CRITICAL_SECTION_START uint32_t primask = __get_PRIMASK(); __disable_irq();
  #define CRITICAL_SECTION_END if (primask == 0) __enable_irq();

  // On AVR this is in math.h?
  #define square(x) ((x)*(x))

  // On AVR this is in sfr_defs.h
  #ifndef _BV
    #define _BV(b) (1 << (b))
  #endif

  #if ENABLED(DELTA_FAST_SQRT)
    #define ATAN2(y, x) atan2f(y, x)
    #define FABS(x) fabsf(x)
    #define POW(x, y) powf(x, y)
    #define SQRT(x) sqrtf(x)
    #define CEIL(x) ceilf(x)
    #define FLOOR(x) floorf(x)
    #define LROUND(x) lroundf(x)
    #define FMOD(x, y) fmodf(x, y)
  #else
    #define ATAN2(y, x) atan2(y, x)
    #define FABS(x) fabs(x)
    #define POW(x, y) pow(x, y)
    #define SQRT(x) sqrt(x)
    #define CEIL(x) ceil(x)
    #define FLOOR(x) floor(x)
    #define LROUND(x) lround(x)
    #define FMOD(x, y) fmod(x, y)
  #endif
  #define HYPOT(x,y) SQRT(HYPOT2(x,y))

  // timers
  #define NUM_HARDWARE_TIMERS 9

  #define REFERENCE_F_CPU 16000000 // 16MHz MEGA2560

  #define REFERENCE_EXTRUDER_TIMER_PRESCALE 64
  #define HAL_REFERENCE_EXTRUDER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_EXTRUDER_TIMER_PRESCALE) // timer0 of MEGA2560: 16000000 / 64 = 250KHz
  #define REFERENCE_EXTRUDER_TIMER_FREQUENCY (HAL_REFERENCE_EXTRUDER_TIMER_RATE / 200) // note: timer0 is in mode3 (8bit fast PWM), 1.25KHz at start 

  #define REFERENCE_STEPPER_TIMER_PRESCALE 8
  #define HAL_REFERENCE_STEPPER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_STEPPER_TIMER_PRESCALE) // timer1 of MEGA2560: 16000000 / 8 = 2MHz
  #define REFERENCE_STEPPER_TIMER_FREQUENCY (HAL_REFERENCE_STEPPER_TIMER_RATE / 2000) // note: timer0 is in mode2 (CTC), 1KHz at start

  #define REFERENCE_TEMP_TIMER_PRESCALE 64
  #define HAL_REFERENCE_TEMP_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_TEMP_TIMER_PRESCALE) // timer0 of MEGA2560: 16000000 / 64 = 250KHz (sharing with advanced extruder)
  #define REFERENCE_TEMP_TIMER_FREQUENCY (HAL_REFERENCE_TEMP_TIMER_RATE / 256) // note: timer0 is in mode3 (8bit fast PWM), 976.5625Hz always 

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    #define EXTRUDER_TIMER 1
    #define EXTRUDER_TIMER_PRIORITY 6
    #define EXTRUDER_TIMER_FREQUENCY REFERENCE_EXTRUDER_TIMER_FREQUENCY
    #define EXTRUDER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
    #define EXTRUDER_TIMER_PRESCALE 2
    #define HAL_EXTRUDER_TIMER_RATE (F_CPU / EXTRUDER_TIMER_PRESCALE) // = 42MHz
    #define EXTRUDER_TIMER_FACTOR (HAL_EXTRUDER_TIMER_RATE / HAL_REFERENCE_EXTRUDER_TIMER_RATE)
    #define EXTRUDER_TIMER_TICKS_PER_MILLISECOND (HAL_EXTRUDER_TIMER_RATE) / 1000
  #endif

  #define STEPPER_TIMER 2
  #define STEPPER_TIMER_PRIORITY 2
  #define STEPPER_TIMER_FREQUENCY REFERENCE_STEPPER_TIMER_FREQUENCY
  #define STEPPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
  #define STEPPER_TIMER_PRESCALE 2
  #define HAL_STEPPER_TIMER_RATE (F_CPU / STEPPER_TIMER_PRESCALE) // = 42MHz
  #define STEPPER_TIMER_FACTOR (HAL_STEPPER_TIMER_RATE / HAL_REFERENCE_STEPPER_TIMER_RATE)
  #define STEPPER_TIMER_TICKS_PER_MILLISECOND (HAL_STEPPER_TIMER_RATE) / 1000

  #define TEMP_TIMER 3
  #define TEMP_TIMER_PRIORITY 15
  #define TEMP_TIMER_FREQUENCY REFERENCE_TEMP_TIMER_FREQUENCY
  #define TEMP_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK2 // TIMER_CLOCK2 -> 8 divisor
  #define TEMP_TIMER_PRESCALE 8
  #define HAL_TEMP_TIMER_RATE (F_CPU / TEMP_TIMER_PRESCALE) // = 10.5MHz
  #define TEMP_TIMER_FACTOR (HAL_TEMP_TIMER_RATE / HAL_REFERENCE_TEMP_TIMER_RATE)
  #define TEMP_TIMER_TICKS_PER_MILLISECOND (HAL_TEMP_TIMER_RATE) / 1000

#if 0
  #define BEEPER_TIMER 4
  #define BEEPER_TIMER_PRIORITY NVIC_EncodePriority(4, 6, 3)
  #define BEEPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK4 // TIMER_CLOCK4 -> 128 divisor
  #define BEEPER_TIMER_PRESCALE 128
  #define HAL_BEEPER_TIMER_RATE (F_CPU / BEEPER_TIMER_PRESCALE)
  #define BEEPER_TIMER_TICKS_PER_MILLISECOND (HAL_BEEPER_TIMER_RATE) / 1000
#endif

  #define HAL_ISR_WATCHDOG_TIMER void WDT_Handler()
  #define WATCHDOG_TIMER_PRIORITY 1

  #define _HAL_ISR(p) void TC ## p ## _Handler()
  #define HAL_ISR(p) _HAL_ISR(p)
  #define HAL_TIMER_START(n) HAL_timer_start(n, n ## _PRIORITY, n ## _FREQUENCY, n ## _CLOCK, n ## _PRESCALE)

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    #define ENABLE_EXTRUDER_INTERRUPT() HAL_timer_enable_interrupt(EXTRUDER_TIMER)
    #define DISABLE_EXTRUDER_INTERRUPT() HAL_timer_disable_interrupt(EXTRUDER_TIMER)
    #define HAL_TIMER_SET_EXTRUDER_COUNT(n) HAL_timer_set_count(EXTRUDER_TIMER, n)
  #endif

  #define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEPPER_TIMER)
  #define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER)
  #define HAL_TIMER_SET_STEPPER_COUNT(n) HAL_timer_set_count(STEPPER_TIMER, n)

  #define ENABLE_TEMP_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER)
  #define HAL_TIMER_SET_TEMP_COUNT(n) HAL_timer_set_count(TEMP_TIMER, n)

  // --------------------------------------------------------------------------
  // Types
  // --------------------------------------------------------------------------

  #define HAL_TIMER_TYPE uint32_t

  // --------------------------------------------------------------------------
  // Public Variables
  // --------------------------------------------------------------------------

  // reset reason set by bootloader
  extern uint8_t MCUSR;
  volatile static uint32_t debug_counter;
  
  // timers
  typedef struct {
    Tc          *pTimerRegs;
    uint16_t    channel;
    IRQn_Type   IRQ_Id;
  } tTimerConfig;

  // this should not be written in .h, but I can not solve compilation error 
  static const tTimerConfig TimerConfig[NUM_HARDWARE_TIMERS] =
  {
    { TC0, 0, TC0_IRQn},
    { TC0, 1, TC1_IRQn},
    { TC0, 2, TC2_IRQn},
    { TC1, 0, TC3_IRQn},
    { TC1, 1, TC4_IRQn},
    { TC1, 2, TC5_IRQn},
    { TC2, 0, TC6_IRQn},
    { TC2, 1, TC7_IRQn},
    { TC2, 2, TC8_IRQn},
  };

  // --------------------------------------------------------------------------
  // Public functions
  // --------------------------------------------------------------------------

  // Disable interrupts
  void cli(void);

  // Enable interrupts
  void sei(void);

  // Delays
#if 0
  static inline void HAL_delay(millis_t ms) {
    unsigned int del;
    while (ms > 0) {
      del = ms > 100 ? 100 : ms;
      delay(del);
      ms -= del;
    }
  }
#endif

  static FORCE_INLINE void HAL_delayMicroseconds(uint32_t usec) { //usec += 3;
    uint32_t n = usec * (F_CPU / 3000000);
    asm volatile(
      "L2_%=_delayMicroseconds:"       "\n\t"
      "subs   %0, #1"                 "\n\t"
      "bge    L2_%=_delayMicroseconds" "\n"
      : "+r" (n) :  
    );
  }

  // return free memory between end of heap (or end bss) and whatever is current
  int freeMemory(void);

  // SPI
  #ifdef SOFTWARE_SPI
    inline uint8_t spiTransfer(uint8_t b); // using Mode 0
    inline void spiBegin();
    inline void spiInit(uint8_t spiRate);
    inline uint8_t spiRec();
    inline void spiRead(uint8_t*buf, uint16_t nbyte);
    inline void spiSend(uint8_t b);
    inline void spiSend(const uint8_t* buf , size_t n) ;
    inline void spiSendBlock(uint8_t token, const uint8_t* buf);
  #else
    // Hardware setup
    void spiBegin();
    void spiInit(uint8_t spiRate);
    // Write single byte to SPI
    void spiSend(byte b);
    void spiSend(const uint8_t* buf, size_t n);
    void spiSend(uint32_t chan, byte b);
    void spiSend(uint32_t chan , const uint8_t* buf , size_t n);
    // Read single byte from SPI
    uint8_t spiRec();
    uint8_t spiRec(uint32_t chan);
    // Read from SPI into buffer
    void spiRead(uint8_t* buf, uint16_t nbyte);
    // Write from buffer to SPI
    void spiSendBlock(uint8_t token, const uint8_t* buf);
  #endif

  // eeprom
  uint8_t eeprom_read_byte(uint8_t* pos);
  void eeprom_read_block(void* pos, const void* eeprom_address, size_t n);
  void eeprom_write_byte(uint8_t* pos, uint8_t value);
  void eeprom_update_block(const void* pos, void* eeprom_address, size_t n);

  // Timers
  void HAL_timer_start(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale);
  void HAL_timer_enable_interrupt(uint8_t timer_num);
  #if ENABLED(USE_WATCHDOG)
    void watchdogSetup(void);
    #if ENABLED(WATCHDOG_RESET_MANUAL)
      void HAL_watchdog_timer_enable_interrupt(uint32_t timeout);
    #endif
  #endif
  void HAL_timer_disable_interrupt(uint8_t timer_num);

  static FORCE_INLINE void HAL_timer_isr_prologue(uint8_t timer_num) {
    const tTimerConfig *pConfig = &TimerConfig[timer_num];

    TC_GetStatus(pConfig->pTimerRegs, pConfig->channel); // clear status register
  }

  static FORCE_INLINE uint32_t HAL_timer_get_count(uint8_t timer_num) {
    const tTimerConfig *pConfig = &TimerConfig[timer_num];

    return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
  }

  static FORCE_INLINE uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
    const tTimerConfig *pConfig = &TimerConfig[timer_num];

    return TC_ReadCV(pConfig->pTimerRegs, pConfig->channel);
  }
 
  static FORCE_INLINE void HAL_timer_set_count(uint8_t timer_num, uint32_t count) {
    const tTimerConfig *pConfig = &TimerConfig[timer_num];

    TC_SetRC(pConfig->pTimerRegs, pConfig->channel, count);
  }

#if 0
  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    void HAL_extruder_timer_start(void);
  #endif
  void HAL_step_timer_start(void);
  void HAL_temp_timer_start(void);

  static FORCE_INLINE void HAL_timer_isr_status(Tc* tc, uint32_t channel) {
    TC_GetStatus(tc, channel); // clear status register
  }

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    static FORCE_INLINE void HAL_extruder_count(uint32_t count) {
      // Get the ISR from table
      Tc *tc = TimerConfig[EXTRUDER_TIMER].pTimerRegs;
      uint32_t channel = TimerConfig[EXTRUDER_TIMER].channel;
      uint32_t counter_value = TC_ReadCV(tc, channel) + 2 * EXTRUDER_TIMER_FACTOR; // we need time for other stuff!

      //if (count < 105) count = 105;
      TC_SetRC(tc, channel, (counter_value <= count) ? count : counter_value);
    }
  #endif

  static FORCE_INLINE void HAL_timer_stepper_count(uint32_t count) {
    // Get the ISR from table
    Tc *tc = TimerConfig[STEPPER_TIMER].pTimerRegs;
    uint32_t channel = TimerConfig[STEPPER_TIMER].channel;
    uint32_t counter_value = TC_ReadCV(tc, channel) + 2 * STEPPER_TIMER_FACTOR; // we need time for other stuff!

    //if (count < 105) count = 105;
    TC_SetRC(tc, channel, (counter_value <= count) ? count : counter_value);
  }

  void tone(uint8_t pin, int frequency);
  void noTone(uint8_t pin);
  //void tone(uint8_t pin, int frequency, long duration);
#endif

  // A/D converter
  uint16_t getAdcReading(adc_channel_num_t chan);
  void startAdcConversion(adc_channel_num_t chan);
  adc_channel_num_t pinToAdcChannel(int pin);

  uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion = false);
  uint16_t getAdcSuperSample(adc_channel_num_t chan);
  void setAdcFreerun(void);
  void stopAdcFreerun(adc_channel_num_t chan);

#endif // _HAL_H
