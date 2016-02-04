#include "Marlin.h"

#if ENABLED(USE_WATCHDOG)

#include "watchdog.h"

// Initialize watchdog with a 4 sec interrupt time
void watchdog_init() {
  #ifdef __SAM3X8E__
    const uint32_t wdtTicks = 256;	// number of watchdog ticks @ 32768Hz/128 before the watchdog times out (max 4095)
    WDT_Enable(WDT, (wdtTicks << WDT_MR_WDV_Pos) | (wdtTicks << WDT_MR_WDD_Pos) | WDT_MR_WDRSTEN);	// enable watchdog, reset the mcu if it times out
  #else
    #if ENABLED(WATCHDOG_RESET_MANUAL)
      // We enable the watchdog timer, but only for the interrupt.
      // Take care, as this requires the correct order of operation, with interrupts disabled. See the datasheet of any AVR chip for details.
      wdt_reset();
      _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
      _WD_CONTROL_REG = _BV(WDIE) | WDTO_4S;
    #else
      wdt_enable(WDTO_4S);
    #endif
  #endif
}

#ifdef __SAM3X8E__
// Reset watchdog. MUST be called every 1s after init or avr will reset.
void watchdog_reset() {
	WDT_Restart(WDT);
}
#endif

#ifndef __SAM3X8E__
    //===========================================================================
  //=================================== ISR ===================================
  //===========================================================================

  // Watchdog timer interrupt, called if main program blocks >1sec and manual reset is enabled.
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    ISR(WDT_vect) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Something is wrong, please turn off the printer.");
      kill(PSTR("ERR:Please Reset")); //kill blocks //16 characters so it fits on a 16x2 display
      while (1); //wait for user or serial reset
    }
  #endif //WATCHDOG_RESET_MANUAL
#endif

#endif //USE_WATCHDOG
