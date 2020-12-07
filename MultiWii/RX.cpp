#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#if defined(SPEKTRUM)
  #include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner. 
#endif

//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[RC_CHANS] = {SBUS};
#elif defined(SUMD)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4};
#elif defined(SPEKTRUM)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#else // Standard Channel order
  static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
  static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

void rxInt(void);

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  #if defined(STANDARD_RX)
    #if defined(MEGA)
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
    #endif
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
    
    /*************    atmega328P's Specific Aux2 Pin Setup    *********************/
    #if defined(PROMINI)
     #if defined(RCAUXPIN)
        PCICR  |= (1 << 0) ; // PCINT activated also for PINS [D8-D13] on port B
        #if defined(RCAUXPIN8)
          PCMSK0 = (1 << 0);
        #endif
        #if defined(RCAUXPIN12)
          PCMSK0 = (1 << 4);
        #endif
      #endif
    #endif
    
    /***************   atmega32u4's Specific RX Pin Setup   **********************/
    #if defined(PROMICRO)
      //Trottle on pin 7
      DDRE &= ~(1 << 6); // pin 7 to input
      PORTE |= (1 << 6); // enable pullups
      EICRB |= (1 << ISC60);
      EIMSK |= (1 << INT6); // enable interuppt
      // Aux2 pin on PBO (D17/RXLED)
      #if defined(RCAUX2PIND17)
        DDRB &= ~(1 << 0); // set D17 to input 
      #endif
      // Aux2 pin on PD2 (RX0)
      #if defined(RCAUX2PINRXO)
        DDRD &= ~(1 << 2); // RX to input
        PORTD |= (1 << 2); // enable pullups
        EICRA |= (1 << ISC20);
        EIMSK |= (1 << INT2); // enable interuppt
      #endif
    #endif
    
  /*************************   Special RX Setup   ********************************/
  #endif
  #if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT; 
  #endif
  #if defined (SPEKTRUM) || defined(SUMD)
    SerialOpen(RX_SERIAL_PORT,115200);
  #endif
  #if defined(SBUS)
    SerialOpen(RX_SERIAL_PORT,100000);
    switch (RX_SERIAL_PORT) { //parity
      #if defined(MEGA)
        case 0: UCSR0C |= (1<<UPM01)|(1<<USBS0); break;
        case 1: UCSR1C |= (1<<UPM11)|(1<<USBS1); break;
        case 2: UCSR2C |= (1<<UPM21)|(1<<USBS2); break;
        case 3: UCSR3C |= (1<<UPM31)|(1<<USBS3); break;
      #endif
    }
  #endif
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
#if defined(STANDARD_RX)

#if defined(FAILSAFE) && !defined(PROMICRO)
   // predefined PC pin block (thanks to lianj)  - Version with failsafe
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
          if((rc_value_pos==THROTTLEPIN || rc_value_pos==YAWPIN ||   \
              rc_value_pos==PITCHPIN || rc_value_pos==ROLLPIN)       \
              && dTime>FAILSAFE_DETECT_TRESHOLD)                     \
                GoodPulses |= (1<<rc_value_pos);                     \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
#else
   // predefined PC pin block (thanks to lianj)  - Version without failsafe
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
#endif

  // port change Interrupt
  ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  #if defined(FAILSAFE) && !defined(PROMICRO)
    static uint8_t GoodPulses;
  #endif
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,2);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,4);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,5);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,6);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,7);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,0);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,1);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,3);
    #endif
    
    #if defined(FAILSAFE) && !defined(PROMICRO)
      if (GoodPulses==(1<<THROTTLEPIN)+(1<<YAWPIN)+(1<<ROLLPIN)+(1<<PITCHPIN)) {  // If all main four chanells have good pulses, clear FailSafe counter
        GoodPulses = 0;
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; 
      }
    #endif
  }
  /*********************      atmega328P's Aux2 Pins      *************************/
  #if defined(PROMINI)
    #if defined(RCAUXPIN)
    /* this ISR is a simplification of the previous one for PROMINI on port D
       it's simplier because we know the interruption deals only with one PIN:
       bit 0 of PORT B, ie Arduino PIN 8
       or bit 4 of PORTB, ie Arduino PIN 12
     => no need to check which PIN has changed */
    ISR(PCINT0_vect) {
      uint8_t pin;
      uint16_t cTime,dTime;
      static uint16_t edgeTime;
    
      pin = PINB;
      cTime = micros();
      sei();
      #if defined(RCAUXPIN8)
       if (!(pin & 1<<0)) {     //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
      #if defined(RCAUXPIN12)
       if (!(pin & 1<<4)) {     //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
        dTime = cTime-edgeTime; if (900<dTime && dTime<2200) rcValue[0] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
      } else edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
    }
    #endif
  #endif
  
  /****************      atmega32u4's Throttle & Aux2 Pin      *******************/
  #if defined(PROMICRO)
    // throttle
    ISR(INT6_vect){ 
      static uint16_t now,diff;
      static uint16_t last = 0;
      now = micros();  
      if(!(PINE & (1<<6))){
        diff = now - last;
        if(900<diff && diff<2200){
          rcValue[3] = diff;
          #if defined(FAILSAFE)
           if(diff>FAILSAFE_DETECT_TRESHOLD) {        // if Throttle value is higher than FAILSAFE_DETECT_TRESHOLD
             if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
           }
          #endif 
        }
      }else last = now; 
    }
    // Aux 2
    #if defined(RCAUX2PINRXO)
      ISR(INT2_vect){
        static uint16_t now,diff;
        static uint16_t last = 0; 
        now = micros();  
        if(!(PIND & (1<<2))){
          diff = now - last;
          if(900<diff && diff<2200) rcValue[7] = diff;
        }else last = now;
      }
    #endif  
  #endif
#endif


/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// attachInterrupt fix for promicro
#if defined(PROMICRO) && defined(SERIAL_SUM_PPM)
  ISR(INT6_vect){rxInt();}
#endif

// PPM_SUM at THROTTLE PIN on MEGA boards
#if defined(PPM_ON_THROTTLE) && defined(MEGA) && defined(SERIAL_SUM_PPM)
  ISR(PCINT2_vect) { if(PINK & (1<<0)) rxInt(); }
#endif

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)
  void rxInt(void) {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
  #if defined(FAILSAFE)
    static uint8_t GoodPulses;
  #endif
  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<RC_CHANS ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
        #if defined(FAILSAFE)
          if(chan<4 && diff>FAILSAFE_DETECT_TRESHOLD) GoodPulses |= (1<<chan); // if signal is valid - mark channel as OK
          if(GoodPulses==0x0F) {                                               // If first four chanells have good pulses, clear FailSafe counter
            GoodPulses = 0;
            if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
          }
        #endif
      }
    chan++;
  }
}
#endif

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)

#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
static uint16_t sbusIndex=0;
static uint16_t sbus[25]={0};

void readSerial_RX(){
  while(SerialAvailable(RX_SERIAL_PORT)){
    int val = SerialRead(RX_SERIAL_PORT);
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      spekFrameFlags = 0x00;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;
      spekFrameDone = 0x01;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif

      // For some reason the SBUS data provides only about 75% of the actual RX output pulse width
      // Adjust the actual value by +/-25%.  Sign determined by pulse width above or below center
      uint8_t adj_index;
      for(adj_index=0; adj_index<16; adj_index++) {
        if (rcValue[adj_index] < MIDRC)
          rcValue[adj_index] -= (MIDRC - rcValue[adj_index]) >> 2;
        else
          rcValue[adj_index] += (rcValue[adj_index] - MIDRC) >> 2;
      }
    }
  }        
}
#endif

/**************************************************************************************/
/*************** SUMD ********************/
/**************************************************************************************/

#if defined(SUMD)
#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAXCHAN 8
#define SUMD_BUFFSIZE SUMD_MAXCHAN*2 + 5 // 6 channels + 5 -> 17 bytes for 6 channels
static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t sumd[SUMD_BUFFSIZE]={0};

void readSerial_RX(void) {
  while (SerialAvailable(RX_SERIAL_PORT)) {
    int val = SerialRead(RX_SERIAL_PORT);
    if(sumdIndex == 0 && val != SUMD_SYNCBYTE) continue;
    if(sumdIndex == 2) sumdSize = val;
    if(sumdIndex < SUMD_BUFFSIZE) sumd[sumdIndex] = val;
    sumdIndex++;

    if(sumdIndex == sumdSize*2+5) {
      sumdIndex = 0;
      spekFrameFlags = 0x00;
      debug[1] = sumd[1];
      if (sumdSize > SUMD_MAXCHAN) sumdSize = SUMD_MAXCHAN;
      for (uint8_t b = 0; b < sumdSize; b++)
        rcValue[b] = ((sumd[2*b+3]<<8) | sumd[2*b+4])>>3;
      spekFrameDone = 0x01; // havent checked crc at all

      #if defined(FAILSAFE)
      if (sumd[1] == 0x01)
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;} // clear FailSafe counter
      #endif
    }
  }
}
#endif

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
#if defined(SPEKTRUM)
void readSerial_RX(void) {
  if ((!f.ARMED) && 
     #if defined(FAILSAFE) || (RX_SERIAL_PORT != 0) 
        (failsafeCnt > 5) &&
     #endif
      ( SerialPeek(RX_SERIAL_PORT) == '$')) {
    while (SerialAvailable(RX_SERIAL_PORT)) {
      serialCom();
      delay (10);
    }
    return;
  } //End of: Is it the GUI?
  while (SerialAvailable(RX_SERIAL_PORT) > SPEK_FRAME_SIZE) { // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++) {SerialRead(RX_SERIAL_PORT);}  //Toss one full frame of bytes.
  }  
  if (spekFrameFlags == 0x01) {   //The interrupt handler saw at least one valid frame start since we were last here. 
    if (SerialAvailable(RX_SERIAL_PORT) == SPEK_FRAME_SIZE) {  //A complete frame? If not, we'll catch it next time we are called. 
      SerialRead(RX_SERIAL_PORT); SerialRead(RX_SERIAL_PORT);        //Eat the header bytes 
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2) {
        uint8_t bh = SerialRead(RX_SERIAL_PORT);
        uint8_t bl = SerialRead(RX_SERIAL_PORT);
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
        if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
      }
      spekFrameFlags = 0x00;
      spekFrameDone = 0x01;
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
      #endif
    } else { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
      uint32_t spekInterval = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()) - spekTimeLast;
      if (spekInterval > 2500) {spekFrameFlags = 0;}  //If it has been a while, make the interrupt handler start over. 
    }
  }
}
#endif

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  #if defined(SPEKTRUM) || defined(SBUS) || defined(SUMD)
    if (chan < RC_CHANS) {
      data = rcValue[rcChannel[chan]];
    } else data = 1500;
  #else
    uint8_t oldSREG;
    oldSREG = SREG; cli(); // Let's disable interrupts
    data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
    SREG = oldSREG;        // Let's restore interrupt state
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
#define AVERAGING_ARRAY_LENGTH 4
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH-1];
  uint16_t rcDataMean,rcDataTmp;
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  uint8_t failsafeGoodCondition = 1;

  #if !defined(OPENLRSv2MULTI)
    rc4ValuesIndex++;
    if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH-1) rc4ValuesIndex = 0;
    for (chan = 0; chan < RC_CHANS; chan++) {
      rcDataTmp = readRawRC(chan);
      #if defined(FAILSAFE)
        failsafeGoodCondition = rcDataTmp>FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED; // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
      #endif                                                                                // In disarmed state allow always update for easer configuration.
      #if defined(SPEKTRUM) || defined(SBUS) || defined(SUMD) // no averaging for Spektrum & SBUS & SUMD signal
        if(failsafeGoodCondition)  rcData[chan] = rcDataTmp;
      #else
        if(failsafeGoodCondition) {
          rcDataMean = rcDataTmp;
          for (a=0;a<AVERAGING_ARRAY_LENGTH-1;a++) rcDataMean += rcData4Values[chan][a];
          rcDataMean = (rcDataMean+(AVERAGING_ARRAY_LENGTH/2))/AVERAGING_ARRAY_LENGTH;
          if ( rcDataMean < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean+2;
          if ( rcDataMean > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean-2;
          rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
        }
      #endif
      if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
        rcSerialCount --;
        #if defined(FAILSAFE)
          failsafeCnt = 0;
        #endif
        if (rcSerial[chan] >900) {rcData[chan] = rcSerial[chan];} // only relevant channels are overridden
      }
    }
  #endif
}


/**************************************************************************************/
/***************                     OPENLRS                       ********************/
/**************************************************************************************/

//note: this dont feels right in RX.pde

#if defined(OPENLRSv2MULTI) 
// **********************************************************
// ******************   OpenLRS Rx Code   *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2012  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.11
// Latest Code Update : 2012-03-25
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
// # PROJECT DEVELOPERS # 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
//

//######### TRANSMISSION VARIABLES ##########
#define CARRIER_FREQUENCY 435000  // 435Mhz startup frequency
#define FREQUENCY_HOPPING 1 // 1 = Enabled  0 = Disabled

//###### HOPPING CHANNELS #######
//Select the hopping channels between 0-255
// Default values are 13,54 and 23 for all transmitters and receivers, you should change it before your first flight for safety.
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static uint8_t hop_list[3] = {13,54,23}; 

//###### RF DEVICE ID HEADERS #######
// Change this 4 byte values for isolating your transmission, RF module accepts only data with same header
static uint8_t RF_Header[4] = {'O','L','R','S'};  
     
//########## Variables #################

static uint32_t last_hopping_time;
static uint8_t RF_Rx_Buffer[17];
static uint16_t temp_int;
static uint16_t Servo_Buffer[10] = {3000,3000,3000,3000,3000,3000,3000,3000};   //servo position values from RF
static uint8_t hopping_channel = 1;


// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx/Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

//***************************************************************************** 
//*****************************************************************************  
unsigned char ItStatus1, ItStatus2;  

//-------------------------------------------------------------- 
void Write0( void ) { 
  SCK_off;  
  NOP(); 
  SDI_off; 
  NOP(); 
  SCK_on;  
  NOP(); 
} 
//-------------------------------------------------------------- 
void Write1( void ) { 
  SCK_off;
  NOP(); 
  SDI_on;
  NOP(); 
  SCK_on; 
  NOP(); 
} 
//-------------------------------------------------------------- 
void Write8bitcommand(uint8_t command) {  // keep sel to low 
  uint8_t n=8; 
  nSEL_on;
  SCK_off;
  nSEL_off; 
  while(n--) { 
    if(command&0x80) 
      Write1(); 
    else 
      Write0();    
    command = command << 1; 
  } 
  SCK_off;
}  

//-------------------------------------------------------------- 
void send_read_address(uint8_t i) { 
  i &= 0x7f; 
  Write8bitcommand(i); 
}  

//-------------------------------------------------------------- 
void send_8bit_data(uint8_t i) { 
  uint8_t n = 8; 
  SCK_off;
  while(n--) { 
    if(i&0x80) 
      Write1(); 
    else 
      Write0();    
    i = i << 1; 
  } 
  SCK_off;
}  
//-------------------------------------------------------------- 

uint8_t read_8bit_data(void) {
  uint8_t Result, i; 
  
  SCK_off;
  Result=0; 
  for(i=0;i<8;i++) {                    //read fifo data byte 
    Result=Result<<1; 
    SCK_on;
    NOP(); 
    if(SDO_1) { 
      Result|=1;
    } 
    SCK_off;
    NOP(); 
  } 
  return(Result); 
}  

//-------------------------------------------------------------- 
uint8_t _spi_read(uint8_t address) { 
  uint8_t result; 
  send_read_address(address); 
  result = read_8bit_data();  
  nSEL_on; 
  return(result); 
}  

//-------------------------------------------------------------- 
void _spi_write(uint8_t address, uint8_t data) { 
  address |= 0x80; 
  Write8bitcommand(address); 
  send_8bit_data(data);  
  nSEL_on;
}  

//-------Defaults 38.400 baud---------------------------------------------- 
void RF22B_init_parameter(void) { 
  ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
  ItStatus2 = _spi_read(0x04); 
  _spi_write(0x06, 0x00);    // no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  _spi_write(0x09, 0x7f);  // c = 12.5p   
  _spi_write(0x0a, 0x05); 
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State 
  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  _spi_write(0x70, 0x00);    // disable manchest 
    
  // 57.6Kbps data rate
  _spi_write(0x1c, 0x05); // case RATE_57.6K
  _spi_write(0x20, 0x45);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x01); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0xD7); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0xDC); // 0x23 
  _spi_write(0x24, 0x03); // 0x24 
  _spi_write(0x25, 0xB8); // 0x25 
  _spi_write(0x2a, 0x1e); 
  
  _spi_write(0x6e, 0x0E); //case RATE_57.6K 
  _spi_write(0x6f, 0xBF); //case RATE_57.6K 
 
  _spi_write(0x30, 0x8c);    // enable packet handler, msb first, enable crc, 

  _spi_write(0x32, 0xf3);    // 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x42);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x07);    // 7 default value or   // 64 nibble = 32byte preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 
  _spi_write(0x37, 0xd4); 
  _spi_write(0x38, 0x00); 
  _spi_write(0x39, 0x00); 
  _spi_write(0x3a, RF_Header[0]);    // tx header 
  _spi_write(0x3b, RF_Header[1]); 
  _spi_write(0x3c, RF_Header[2]); 
  _spi_write(0x3d, RF_Header[3]); 
  _spi_write(0x3e, 17);    // total tx 17 byte 
  
  //RX HEADER
  _spi_write(0x3f, RF_Header[0]);   // check hearder 
  _spi_write(0x40, RF_Header[1]); 
  _spi_write(0x41, RF_Header[2]); 
  _spi_write(0x42, RF_Header[3]); 
  _spi_write(0x43, 0xff);    // all the bit to be checked 
  _spi_write(0x44, 0xff);    // all the bit to be checked 
  _spi_write(0x45, 0xff);    // all the bit to be checked 
  _spi_write(0x46, 0xff);    // all the bit to be checked 
   
  _spi_write(0x6d, 0x07); // 7 set power max power 
  _spi_write(0x79, 0x00);    // no hopping 
  _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping 

  _spi_write(0x71, 0x23); // Gfsk, fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  //_spi_write(0x72, 0x1F); // frequency deviation setting to 19.6khz (for 38.4kbps)
  _spi_write(0x72, 0x2E); // frequency deviation setting to 28.8khz(for 57.6kbps)
  _spi_write(0x73, 0x00);   
  _spi_write(0x74, 0x00);    // no offset 
 
  //band 435.000
  _spi_write(0x75, 0x53);    
  _spi_write(0x76, 0x7D);    
  _spi_write(0x77, 0x00); 
}


void checkPots() {
  ////Flytron OpenLRS Multi Pots
  pot_P = analogRead(7);
  pot_I = analogRead(6);
  
  pot_P = pot_P - 512;
  pot_I = pot_I - 512;
  
  pot_P = pot_P / 25; //+-20
  pot_I = pot_I / 25; //+-20
}

void initOpenLRS(void) {
  pinMode(GREEN_LED_pin, OUTPUT);  
  pinMode(RED_LED_pin, OUTPUT);
      
  //RF module pins
  pinMode(SDO_pin, INPUT); //SDO
  pinMode(SDI_pin, OUTPUT); //SDI        
  pinMode(SCLK_pin, OUTPUT); //SCLK
  pinMode(IRQ_pin, INPUT); //IRQ
  pinMode(nSel_pin, OUTPUT); //nSEL
  checkPots(); // OpenLRS Multi board hardware pot check;
} 

//----------------------------------------------------------------------- 
void rx_reset(void) { 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
  _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received 
  _spi_write(0x08, 0x03);    //clear fifo disable multi packet 
  _spi_write(0x08, 0x00);    // clear fifo, disable multi packet 
  _spi_write(0x07,RF22B_PWRSTATE_RX );  // to rx mode 
  _spi_write(0x05, RF22B_Rx_packet_received_interrupt); 
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);  
}  
//-----------------------------------------------------------------------    

//-------------------------------------------------------------- 
void to_ready_mode(void) { 
  ItStatus1 = _spi_read(0x03);   
  ItStatus2 = _spi_read(0x04); 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
}  

void to_rx_mode(void) {  
  to_ready_mode(); 
  delay(50); 
  rx_reset(); 
  NOP(); 
}  

//-------------------------------------------------------------- 
void to_sleep_mode(void) { 
  //  TXEN = RXEN = 0; 
  //LED_RED = 0; 
  _spi_write(0x07, RF22B_PWRSTATE_READY);  
   
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);    
  _spi_write(0x07, RF22B_PWRSTATE_POWERDOWN); 
} 
//--------------------------------------------------------------   

void frequency_configurator(uint32_t frequency) {
  // frequency formulation from Si4432 chip's datasheet
  // original formulation is working with mHz values and floating numbers, I replaced them with kHz values.
  frequency = frequency / 10;
  frequency = frequency - 24000;
  frequency = frequency - 19000; // 19 for 430-439.9 MHz band from datasheet
  frequency = frequency * 64; // this is the Nominal Carrier Frequency (fc) value for register setting
  
  uint8_t byte0 = (uint8_t) frequency;
  uint8_t byte1 = (uint8_t) (frequency >> 8);
  
  _spi_write(0x76, byte1);    
  _spi_write(0x77, byte0); 
}

//############# FREQUENCY HOPPING FUNCTIONS #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void) {
  hopping_channel++;
  if (hopping_channel>2) hopping_channel = 0;
  _spi_write(0x79, hop_list[hopping_channel]);
}
#endif

void Config_OpenLRS() {
  RF22B_init_parameter(); // Configure the RFM22B's registers
  frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.
  to_rx_mode(); 
  #if (FREQUENCY_HOPPING==1)
    Hopping(); //Hop to the next frequency
  #endif  
}

//############ MAIN LOOP ##############
void Read_OpenLRS_RC() {
  uint8_t i,tx_data_length;
  uint8_t first_data = 0;

  if (_spi_read(0x0C)==0) {RF22B_init_parameter(); to_rx_mode(); }// detect the locked module and reboot                         
  if ((currentTime-last_hopping_time > 25000)) {//automatic hopping for clear channel when rf link down for 25ms. 
    Red_LED_ON;
    last_hopping_time = currentTime;  
    #if (FREQUENCY_HOPPING==1)
      Hopping(); //Hop to the next frequency
    #endif   
  }
  if(nIRQ_0) { // RFM22B INT pin Enabled by received Data
    Red_LED_ON;                                 
    send_read_address(0x7f); // Send the package read command
    for(i = 0; i<17; i++) {//read all buffer 
      RF_Rx_Buffer[i] = read_8bit_data(); 
    }  
    rx_reset();
    if (RF_Rx_Buffer[0] == 'S') {// servo control data
      for(i = 0; i<8; i++) {//Write into the Servo Buffer                                                       
        temp_int = (256*RF_Rx_Buffer[1+(2*i)]) + RF_Rx_Buffer[2+(2*i)];
        if ((temp_int>1500) && (temp_int<4500)) Servo_Buffer[i] = temp_int/2;                                                                                                           
      }
      rcData[ROLL] = Servo_Buffer[0];
      rcData[PITCH] = Servo_Buffer[1];
      rcData[THROTTLE] = Servo_Buffer[2];
      rcData[YAW] = Servo_Buffer[3]; 
      rcData[AUX1] = Servo_Buffer[4]; 
      rcData[AUX2] = Servo_Buffer[5]; 
      rcData[AUX3] = Servo_Buffer[6]; 
      rcData[AUX4] = Servo_Buffer[7];  
    }
    #if (FREQUENCY_HOPPING==1)
      Hopping(); //Hop to the next frequency
    #endif  
    delay(1);              
    last_hopping_time = currentTime;    
    Red_LED_OFF;
  }
  Red_LED_OFF;
}
#endif


#if defined(SPEK_BIND)  // Bind Support
void spekBind() {
  pinMode(SPEK_BIND_DATA, INPUT);     // Data line from sat
  digitalWrite(SPEK_BIND_DATA,LOW);   // Turn off internal Pull Up resistor

  pinMode(SPEK_BIND_GROUND, INPUT);
  digitalWrite(SPEK_BIND_GROUND,LOW);
  pinMode(SPEK_BIND_GROUND, OUTPUT);
  digitalWrite(SPEK_BIND_GROUND,LOW);

  pinMode(SPEK_BIND_POWER, INPUT);
  digitalWrite(SPEK_BIND_POWER,LOW);
  pinMode(SPEK_BIND_POWER,OUTPUT);
  
  while(1) {  //Do not return.  User presses reset button to return to normal. 
    blinkLED(4,255,1);
    digitalWrite(SPEK_BIND_POWER,LOW); // Power off sat
    pinMode(SPEK_BIND_DATA, OUTPUT); 
    digitalWrite(SPEK_BIND_DATA,LOW); 
    delay(1000); 
    blinkLED(4,255,1);
    
    digitalWrite(SPEK_BIND_POWER,HIGH); // Power on sat
    delay(10);
    digitalWrite(SPEK_BIND_DATA,HIGH); 
    delay(60);                 // Keep data pin steady for 20 to 120ms after power up
  
    noInterrupts();    
    for (byte i = 0; i < SPEK_BIND_PULSES; i++) { 
      digitalWrite(SPEK_BIND_DATA,LOW); 
        delayMicroseconds(118);
      digitalWrite(SPEK_BIND_DATA,HIGH);
        delayMicroseconds(122);
    }
    interrupts();
    delay(60000);         //Allow one full minute to bind, then try again.
  }
}
#endif
