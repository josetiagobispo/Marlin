/*
   Contributors:
   Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
*/

/*
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
*/

// **************************************************************************
//
// Description:          *** HAL for Arduino Due ***
//
// **************************************************************************
#ifdef __SAM3X8E__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "Marlin.h"
#include <Wire.h>

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

uint8_t MCUSR;

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory;
  int heap_end = (int)_sbrk(0);

  if(heap_end == 0)
    free_memory = ((int)&free_memory) - ((int)&_ebss);
  else
    free_memory = ((int)&free_memory) - heap_end;

  return free_memory;
}

// SPI
#ifdef SOFTWARE_SPI
  // --------------------------------------------------------------------------
  // software SPI
  // --------------------------------------------------------------------------
  // bitbanging transfer
  // run at ~100KHz (necessary for init)
  uint8_t spiTransfer(uint8_t b) { // using Mode 0
    for (int bits = 0; bits < 8; bits++) {
      if (b & 0x80) {
        WRITE(MOSI_PIN, HIGH);
      }
      else {
        WRITE(MOSI_PIN, LOW);
      }
      b <<= 1;

      WRITE(SCK_PIN, HIGH);
      HAL_delayMicroseconds(5U);

      if (READ(MISO_PIN)) {
        b |= 1;
      }
      WRITE(SCK_PIN, LOW);
      HAL_delayMicroseconds(5U);
    }
    return b;
  }

  void spiBegin() {
    SET_OUTPUT(SS_PIN);
    WRITE(SS_PIN, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
  }

  void spiInit(uint8_t spiRate) {
    UNUSED(spiRate);
    WRITE(SS_PIN, HIGH);
    WRITE(MOSI_PIN, HIGH);
    WRITE(SCK_PIN, LOW);
  }

  uint8_t spiRec() {
    WRITE(SS_PIN, LOW);
    uint8_t b = spiTransfer(0xff);
    WRITE(SS_PIN, HIGH);
    return b;
  }

  void spiRead(uint8_t*buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    WRITE(SS_PIN, LOW);
    for (int i = 0; i < nbyte; i++) {
      buf[i] = spiTransfer(0xff);
    }
    WRITE(SS_PIN, HIGH);
  }

  void spiSend(uint8_t b) {
    WRITE(SS_PIN, LOW);
    uint8_t response = spiTransfer(b);
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void spiSend(const uint8_t* buf , size_t n) {
    uint8_t response;
    if (n == 0) return;
    WRITE(SS_PIN, LOW);
    for (uint16_t i = 0; i < n; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void spiSendBlock(uint8_t token, const uint8_t* buf) {
    uint8_t response;

    WRITE(SS_PIN, LOW);
    response = spiTransfer(token);

    for (uint16_t i = 0; i < 512; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }
#else
  // --------------------------------------------------------------------------
  // hardware SPI
  // --------------------------------------------------------------------------
  int spiDueDividors[] = { 10, 21, 42, 84, 168, 255, 255 };
  bool spiInitMaded = false;

  void spiBegin() {
    if(spiInitMaded == false) {
      // Configre SPI pins
      PIO_Configure(
         g_APinDescription[SCK_PIN].pPort,
         g_APinDescription[SCK_PIN].ulPinType,
         g_APinDescription[SCK_PIN].ulPin,
         g_APinDescription[SCK_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[MOSI_PIN].pPort,
         g_APinDescription[MOSI_PIN].ulPinType,
         g_APinDescription[MOSI_PIN].ulPin,
         g_APinDescription[MOSI_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[MISO_PIN].pPort,
         g_APinDescription[MISO_PIN].ulPinType,
         g_APinDescription[MISO_PIN].ulPin,
         g_APinDescription[MISO_PIN].ulPinConfiguration);

      // set master mode, peripheral select, fault detection
      SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS);
      SPI_Enable(SPI0);

      #if MB(ALLIGATOR)
        SET_OUTPUT(DAC0_SYNC);
        #if EXTRUDERS > 1
          SET_OUTPUT(DAC1_SYNC);
          WRITE(DAC1_SYNC, HIGH);
        #endif
        SET_OUTPUT(SPI_EEPROM1_CS);
        SET_OUTPUT(SPI_EEPROM2_CS);
        SET_OUTPUT(SPI_FLASH_CS);
        WRITE(DAC0_SYNC, HIGH);
        WRITE(SPI_EEPROM1_CS, HIGH );
        WRITE(SPI_EEPROM2_CS, HIGH );
        WRITE(SPI_FLASH_CS, HIGH );
        WRITE(SS_PIN, HIGH );
      #endif // MB(ALLIGATOR)
      PIO_Configure(
        g_APinDescription[SPI_PIN].pPort,
        g_APinDescription[SPI_PIN].ulPinType,
        g_APinDescription[SPI_PIN].ulPin,
        g_APinDescription[SPI_PIN].ulPinConfiguration);
      spiInit(1);
      spiInitMaded = true;
    }
  }

  void spiInit(uint8_t spiRate) {
    if(spiInitMaded == false) {
      if(spiRate > 4) spiRate = 1;
      #if MB(ALLIGATOR)
        // Set SPI mode 1, clock, select not active after transfer, with delay between transfers  
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                          SPI_CSR_DLYBCT(1));
        // Set SPI mode 0, clock, select not active after transfer, with delay between transfers 
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA |
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                          SPI_CSR_DLYBCT(1));
      #endif//MB(ALLIGATOR)

      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                        SPI_CSR_DLYBCT(1));
      SPI_Enable(SPI0);
      spiInitMaded = true;
    }
  }

  // Write single byte to SPI
  void spiSend(byte b) {
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // clear status
    SPI0->SPI_RDR;
    //HAL_delayMicroseconds(1U);
  }

  void spiSend(const uint8_t* buf, size_t n) {
    if (n == 0) return;
    for (size_t i = 0; i < n - 1; i++) {
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      SPI0->SPI_RDR;
      //HAL_delayMicroseconds(1U);
    }
    spiSend(buf[n - 1]);
  }

  void spiSend(uint32_t chan, byte b) {
    uint8_t dummy_read = 0;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // clear status
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      dummy_read = SPI0->SPI_RDR;
    UNUSED(dummy_read);
  }

  void spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
    uint8_t dummy_read = 0;
    if (n == 0) return;
    for (int i = 0; i < (int)n - 1; i++) {
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        dummy_read = SPI0->SPI_RDR;
      UNUSED(dummy_read);
    }
    spiSend(chan, buf[n - 1]);
  }

  // Read single byte from SPI
  uint8_t spiRec() {
    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    //HAL_delayMicroseconds(1U);
    return SPI0->SPI_RDR;
  }

  uint8_t spiRec(uint32_t chan) {
    uint8_t spirec_tmp;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      spirec_tmp =  SPI0->SPI_RDR;
      UNUSED(spirec_tmp);

    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    return SPI0->SPI_RDR;
  }

  // Read from SPI into buffer
  void spiRead(uint8_t*buf, uint16_t nbyte) {
    if (nbyte-- == 0) return;

    for (int i = 0; i < nbyte; i++) {
      //while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      buf[i] = SPI0->SPI_RDR;
      //HAL_delayMicroseconds(1U);
    }
    buf[nbyte] = spiRec();
  }

  // Write from buffer to SPI
  void spiSendBlock(uint8_t token, const uint8_t* buf) {
    SPI0->SPI_TDR = (uint32_t)token | SPI_PCS(SPI_CHAN);
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    //while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    //SPI0->SPI_RDR;
    for (int i = 0; i < 511; i++) {
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      SPI0->SPI_RDR;
      //HAL_delayMicroseconds(1U);
    }
    spiSend(buf[511]);
  }
#endif // DISABLED(SOFTWARE_SPI)

// eeprom
static bool eeprom_initialised = false;
static uint8_t eeprom_device_address = 0x50;

static void eeprom_init(void) {
  #if MB(ALLIGATOR)
  #else
    if (!eeprom_initialised) {
      Wire.begin();
      eeprom_initialised = true;
    }
  #endif// MB(ALLIGATOR)
}

uint8_t eeprom_read_byte(uint8_t* pos) {
  #if MB(ALLIGATOR)
    return eprGetValue((unsigned) pos);
  #else
    byte data = 0xFF;
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address >> 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)1);
    if (Wire.available())
      data = Wire.read();
    return data;
  #endif// MB(ALLIGATOR)
}

// maybe let's not read more than 30 or 32 bytes at a time!
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n) {
  eeprom_init();

  Wire.beginTransmission(eeprom_device_address);
  Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
  Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(eeprom_device_address, (byte)n);
  for (byte c = 0; c < n; c++ )
    if (Wire.available()) *((uint8_t*)pos + c) = Wire.read();
}

void eeprom_write_byte(uint8_t* pos, uint8_t value) {
  #if MB(ALLIGATOR)
    eprBurnValue((unsigned) pos, 1, &value);
  #else
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address >> 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.write(value);
    Wire.endTransmission();

    // wait for write cycle to complete
    // this could be done more efficiently with "acknowledge polling"
    delay(5);
  #endif// MB(ALLIGATOR)
}

// WARNING: address is a page address, 6-bit end will wrap around
// also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n) {
  uint8_t eeprom_temp[32] = {0};
  uint8_t flag = 0;

  eeprom_init();

  Wire.beginTransmission(eeprom_device_address);
  Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
  Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(eeprom_device_address, (byte)n);
  for (byte c = 0; c < n; c++) {
    if (Wire.available()) eeprom_temp[c] = Wire.read();
    flag |= (eeprom_temp[c] ^ *((uint8_t*)pos + c));
  }

  if (flag) {
    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
    Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
    Wire.write((uint8_t*)(pos), n);
    Wire.endTransmission();

    // wait for write cycle to complete
    // this could be done more efficiently with "acknowledge polling"
    delay(5);
  }
}

#if MB(ALLIGATOR)
  static void eprBurnValue(unsigned int pos, int size, unsigned char * newvalue) {
    uint8_t eeprom_temp[3];

    /*write enable*/
    eeprom_temp[0] = 6;//WREN
    digitalWrite( SPI_EEPROM1_CS, LOW );
    spiSend(SPI_CHAN_EEPROM1, eeprom_temp , 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    delay(1);

    /*write addr*/
    eeprom_temp[0] = 2;//WRITE
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite(SPI_EEPROM1_CS, LOW);
    spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);        

    spiSend(SPI_CHAN_EEPROM1 ,newvalue , 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    delay(7);   // wait for page write to complete
  }

  // Read any data type from EEPROM that was previously written by eprBurnValue
  static uint8_t eprGetValue(unsigned int pos) {
    int i = 0;
    uint8_t v;
    uint8_t eeprom_temp[3];
    // set read location
    // begin transmission from device

    eeprom_temp[0] = 3;//READ
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM1_CS, LOW);
    spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    v = spiRec(SPI_CHAN_EEPROM1); 
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    return v;
  }
#endif

// Timers
/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/

// new timer by Ps991
// thanks for that work
// http://forum.arduino.cc/index.php?topic=297397.0

void HAL_timer_set(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale) {
  // Get the ISR from table
  Tc *tc = TimerConfig[timer_num].pTimerRegs;
  uint32_t channel = TimerConfig[timer_num].channel;
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, priority);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);

  TC_SetRC(tc, channel, VARIANT_MCK / prescale / frequency);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IDR = TC_IER_CPCS; // disable interrupt

  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt
}

#if ENABLED(USE_WATCHDOG)
  // this function has to be present, otherwise watchdog won't work
  void watchdogSetup(void) {
    // do what you want here
  }

  #if ENABLED(WATCHDOG_RESET_MANUAL)
    void HAL_watchdog_timer_enable_interrupt(uint32_t timeout) {
      /* this assumes the slow clock is running at 32.768 kHz
         watchdog frequency is therefore 32768 / 128 = 256 Hz */
      timeout = timeout * 256 / 1000;
      if (timeout == 0)
        timeout = 1;
      else if (timeout > 0xFFF)
        timeout = 0xFFF;

      /* Enable WDT interrupt line from the core */
      NVIC_DisableIRQ(WDT_IRQn);
      NVIC_ClearPendingIRQ(WDT_IRQn);
      NVIC_SetPriority(WDT_IRQn, WATCHDOG_TIMER_PRIORITY);
      NVIC_EnableIRQ(WDT_IRQn);

      WDT_Enable(WDT, WDT_MR_WDFIEN | WDT_MR_WDV(timeout) | WDT_MR_WDD(timeout));
    }
  #endif
#endif

void HAL_timer_disable_interrupt(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = TC_IER_CPCS; // disable interrupt
}

#if 0
// from DueTimer by Ivan Seidel
// https://github.com/ivanseidel/DueTimer
uint8_t bestClock(double frequency, uint32_t& retRC){
  /*
    Pick the best Clock, thanks to Ogle Basil Hall!
    Timer         Definition
    TIMER_CLOCK1  MCK /  2
    TIMER_CLOCK2  MCK /  8
    TIMER_CLOCK3  MCK / 32
    TIMER_CLOCK4  MCK /128
  */
  const struct {
    uint8_t flag;
    uint8_t divisor;
  } clockConfig[] = {
    { TC_CMR_TCCLKS_TIMER_CLOCK1,   2 },
    { TC_CMR_TCCLKS_TIMER_CLOCK2,   8 },
    { TC_CMR_TCCLKS_TIMER_CLOCK3,  32 },
    { TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
  };
  float ticks;
  float error;
  int clkId = 3;
  int bestClock = 3;
  float bestError = 9.999e99;
  do
  {
    ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[clkId].divisor;
    // error = abs(ticks - round(ticks));
    error = clockConfig[clkId].divisor * abs(ticks - round(ticks));  // Error comparison needs scaling
    if (error < bestError)
    {
      bestClock = clkId;
      bestError = error;
    }
  } while (clkId-- > 0);
  ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[bestClock].divisor;
  retRC = (uint32_t) round(ticks);
  return clockConfig[bestClock].flag;
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  void HAL_extruder_timer_start() {
    // Get the ISR from table
    Tc *tc = TimerConfig[EXTRUDER_TIMER].pTimerRegs;
    uint32_t channel = TimerConfig[EXTRUDER_TIMER].channel;
    IRQn_Type irq = TimerConfig[EXTRUDER_TIMER].IRQ_Id;

    pmc_set_writeprotect(false); // remove write protection on registers
    pmc_enable_periph_clk((uint32_t)irq);
    NVIC_SetPriority(irq, EXTRUDER_TIMER_PRIORITY);

    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | EXTRUDER_TIMER_CLOCK);

    TC_SetRC(tc, channel, VARIANT_MCK / EXTRUDER_TIMER_PRESCALE / EXTRUDER_TIMER_FREQUENCY); // start with EXTRUDER_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
    TC_Start(tc, channel);

    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

    NVIC_EnableIRQ(irq); // enable Nested Vector Interrupt Controller
  }
#endif

void HAL_step_timer_start() {
  // Timer for stepper
  // Timer 3 HAL.h STEPPER_TIMER
  // uint8_t timer_num = STEPPER_TIMER;

  // Get the ISR from table
  Tc *tc = TimerConfig[STEPPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[STEPPER_TIMER].channel;
  IRQn_Type irq = TimerConfig[STEPPER_TIMER].IRQ_Id;

  pmc_set_writeprotect(false); //remove write protection on registers
  pmc_enable_periph_clk((uint32_t)irq); //we need a clock?
  NVIC_SetPriority(irq, STEPPER_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | STEPPER_TIMER_CLOCK);

  TC_SetRC(tc, channel, VARIANT_MCK / STEPPER_TIMER_PRESCALE / STEPPER_TIMER_FREQUENCY); // start with STEPPER_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

  NVIC_EnableIRQ(irq); //enable Nested Vector Interrupt Controller
}

void HAL_temp_timer_start() {
  // Get the ISR from table
  Tc *tc = TimerConfig[TEMP_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[TEMP_TIMER].channel;
  IRQn_Type irq = TimerConfig[TEMP_TIMER].IRQ_Id;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, TEMP_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_CPCTRG | TEMP_TIMER_CLOCK);

  TC_SetRC(tc, channel, VARIANT_MCK / TEMP_TIMER_PRESCALE / TEMP_TIMER_FREQUENCY); // start with TEMP_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

  NVIC_EnableIRQ(irq);
}

// Due have no tone, this is from Repetier 0.92.3
static uint32_t tone_pin = 0;

void tone(uint8_t pin, int frequency) {
  // set up timer counter 1 channel 0 to generate interrupts for
  // toggling output pin.  

  /*TC1, 1, TC4_IRQn*/
  Tc *tc = TimerConfig[BEEPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[BEEPER_TIMER].channel;
  IRQn_Type irq = TimerConfig[BEEPER_TIMER].IRQ_Id;

  uint32_t rc = VARIANT_MCK / BEEPER_TIMER_PRESCALE / frequency; 

  SET_OUTPUT(pin);
  tone_pin = pin;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  // set interrupt to lowest possible priority
  NVIC_SetPriority(irq, BEEPER_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | BEEPER_TIMER_CLOCK);

  TC_SetRA(tc, channel, rc / 2); // 50% duty cycle
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

  NVIC_EnableIRQ(irq);
}

void noTone(uint8_t pin) {
  const tTimerConfig *pConfig = &TimerConfig[BEEPER_TIMER];

  TC_Stop(pConfig->pTimerRegs, pConfig->channel); 
  WRITE_VAR(pin, LOW);
}


// IRQ handler for tone generator
HAL_ISR(BEEPER_TIMER) {
    static bool toggle = 0;

    HAL_timer_isr_prologue(BEEPER_TIMER);
    WRITE_VAR(tone_pin, toggle);
    toggle = !toggle;
}
#endif

// A/D converter
uint16_t getAdcReading(adc_channel_num_t chan) {
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    SBI(ADC->ADC_CHDR, chan);
    return rslt;
  }
  else {
    SERIAL_ECHOLN("error getAdcReading");
    return 0;
  }
}

void startAdcConversion(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHER, chan);
}

// Convert an Arduino Due pin number to the corresponding ADC channel number
adc_channel_num_t pinToAdcChannel(int pin) {
  if (pin < A0) pin += A0;
  return (adc_channel_num_t) (int)g_APinDescription[pin].ulADCChannelNumber;
}

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion) {
  if (wait_for_conversion) while (!((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)));
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    return rslt;
  }
  else {
    SERIAL_ECHOLN("wait freerun");
    return 0;
  }
}

uint16_t getAdcSuperSample(adc_channel_num_t chan) {
  uint16_t rslt = 0;
  for (int i = 0; i < 8; i++) rslt += getAdcFreerun(chan, true);
  return rslt / 4;
}

void setAdcFreerun(void) {
  // ADC_MR_FREERUN_ON: Free Run Mode. It never waits for any trigger.
  ADC->ADC_MR |= ADC_MR_FREERUN_ON | ADC_MR_LOWRES_BITS_12;
}

void stopAdcFreerun(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHDR, chan);
}
#endif // __SAM3X8E__
