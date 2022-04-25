/*
  ##########################################
  #####   MultiProtocol nRF24L01 Tx   ######
  ##########################################
  #        by goebish on rcgroups          #
  #                                        #
  #   Parts of this project are derived    #
  #     from existing work, thanks to:     #
  #                                        #
  #   - PhracturedBlue for DeviationTX     #
  #   - victzh for XN297 emulation layer   #
  #   - Hasi for Arduino PPM decoder       #
  #   - hexfet, midelic, closedsink ...    #
  ##########################################


  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License.
  If not, see <http://www.gnu.org/licenses/>.

  /// Update 2016.oct.06 ///
  Added:
  Servo0 - D6 that is actually signal for ESC, "throttle".
  Servo1 - D7
  Servo2 - D8
  Servo3 - D9
  On/Off from "FLIP" button      - D10
  On/Off from "RETURN" button    - D11
  On/Off from "HEADLESS" button  - D12
  On/Off from "INVERT" button    - D13
*/

#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

#define Debug 0


/*
  #include <Servo.h>
  Servo servo0;
  Servo servo1;
  Servo servo2;
  Servo servo3;
*/
// uncomment the below define to use nRF24_multipro as a receiver
// instead of a transmitter. you must also explicity set the
// 'current_protocol' variable below the the protocol you wish
// the receiver to use.
// Currently only the following RX protocols have been implemented:
// PROTO_BAYANG
#define RX_MODE



#define MOSI_on PORTA.OUT |= PIN6_bm  // PA6 out=1
#define MOSI_off PORTA.OUT &=~PIN6_bm// PA6 out=0
#define SCK_on PORTA.OUT |= PIN7_bm  // PA7 out=1
#define SCK_off PORTA.OUT &=~PIN7_bm // PA7 out=0
#define CE_on PORTB.OUT |= PIN4_bm  // PB4 out=1
#define CE_off PORTB.OUT &=~PIN4_bm  // PB4 out=0
#define CS_on PORTB.OUT |= PIN5_bm    // PB5 out=1
#define CS_off PORTB.OUT &=~PIN5_bm  // PB5 out=0
// SPI input
//#define  MISO_on PORTA.IN |=(PIN5_bm) // PA5 in=1?
#define  MISO_on  PORTA.IN & PIN5_bm // PA5 in=1?

/**
act1 = pb0/pb1
act2 = pb2/pb3
mot=pc0
*/

#define PWM1_on PORTB.OUTSET =_BV(0) // PB0 out=1
#define PWM1_off PORTB.OUTCLR =_BV(0)   // PB0 out=0

#define PWM2_on PORTB.OUTSET =_BV(1)   // PB1 out=1
#define PWM2_off PORTB.OUTCLR =_BV(1)   // PC0-1 out=0

#define PWM3_on PORTB.OUTSET =_BV(2)  // PB2 out=1
#define PWM3_off PORTB.OUTCLR =_BV(2)   // PB2out=0

#define PWM4_on PORTB.OUTSET =_BV(3)   // PB3 out=1
#define PWM4_off PORTB.OUTCLR =_BV(3)   // PB3 out=0

#define PWM5_on PORTC.OUTSET =_BV(0)  // PC0 out=1
#define PWM5_off PORTC.OUTCLR =_BV(0)   // PC0 out=0

#define LedPin_on PORTA.OUTSET =_BV(4)   // PA4 out=1
#define LedPin_off PORTA.OUTCLR =_BV(4)   // PA4 out=1
/*Pwm count*/
volatile unsigned char _time_count=0;      // time++
int steps = 64; // lower steps control but higher frequency output motor 16,32,64,128
int PWM_value1 = 0;
int PWM_value2 = 0;
int PWM_value3 = 0;
int PWM_value4 = 0;
int PWM_value_throttle = 0;
/******/

int mid_value = 512;
int roll_out=0;
int pitch_out=0;
int delta =20;


#define RF_POWER TX_POWER_80mW

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order {
  THROTTLE,
  AILERON,
  ELEVATOR,
  RUDDER,
  AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
  AUX2,  // (CH6)  flip control
  AUX3,  // (CH7)  still camera (snapshot)
  AUX4,  // (CH8)  video camera
  AUX5,  // (CH9)  headless
  AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
  AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
  AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
  /*
    }
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,  // EAchine H7, MoonTop M99xx
  */
  PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
  /*   PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
     PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
     PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
     PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
     PROTO_SYMAXOLD,     // Syma X5C, X2
     PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
     PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
     PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
     PROTO_END
  */
};

// EEPROM locationss
enum {
  ee_PROTOCOL_ID = 0,
  ee_TXID0,
  ee_TXID1,
  ee_TXID2,
  ee_TXID3
};

uint8_t transmitterID[4];
uint8_t current_protocol = PROTO_BAYANG;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset = true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN, PPM_MIN, PPM_MIN, PPM_MIN, PPM_MID, PPM_MID,
                           PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID,
                          };


                          /*Interrupt PWM*/
ISR(TCB1_INT_vect)
{

  if(_time_count<steps)
  {
    
//TCB1.INTFLAGS= TCB_CAPT_bm; //clear interrupt flag


if ( _time_count < PWM_value_throttle ) PWM5_on;
else            PWM5_off;

if ( _time_count < PWM_value1 ) PWM1_on;
else            PWM1_off;

if ( _time_count < PWM_value2 ) PWM2_on;
else            PWM2_off;

if ( _time_count < PWM_value3 ) PWM3_on;
else            PWM3_off;

if ( _time_count < PWM_value4 ) PWM4_on;
else            PWM4_off;

_time_count++;           // 변수 증가  0~255 > 0~255 반복
  

  }
  

  else 
  {
    _time_count=0;

  }
TCB1.INTFLAGS= TCB_CAPT_bm; //clear interrupt flag

}


void setup()
{

/*spi*/
PORTA.DIRSET = PIN6_bm; // use PA6 as an output
PORTA.DIRSET = PIN7_bm; // use PA7 as an output
PORTB.DIRSET = PIN4_bm; // use PA4 as an output
PORTB.DIRSET = PIN5_bm; // use PA6 as an output
       // E5 as an input with pull-up
PORTA.DIRCLR = PIN5_bm;
//       PORTA.PIN5CTRL   =    PORT_OPC_PULLUP_gc;

/*
PORTB.DIRSET = PIN1_bm; // use PB1 as an output PWM1
PORTB.DIRSET = PIN0_bm; // use PB0 as an output PWM1
PORTC.DIRSET = PIN1_bm; // use PC1 as an output PWM2
PORTC.DIRSET = PIN0_bm; // use PC0 as an output PWM2
PORTA.DIRSET = PIN3_bm; // use PA3 as an output PWMthrottle
PORTC.DIRSET = PIN2_bm; // use PC2 as an output PWM3
PORTC.DIRSET = PIN3_bm; // use PC3 as an output PWM3
PORTA.DIRSET = PIN1_bm; // use PA1 as an output PWM4
PORTA.DIRSET = PIN2_bm; // use PA2 as an output PWM4
*/

/*version actuateur*/

PORTB.DIRSET = PIN0_bm; // use PB1 as an output PWM1
PORTB.DIRSET = PIN1_bm; // use PB1 as an output PWM2
PORTB.DIRSET = PIN2_bm; // use PB1 as an output PWM3
PORTB.DIRSET = PIN3_bm; // use PB1 as an output PWM4
PORTC.DIRSET = PIN0_bm; // use PB1 as an output PWM5

/*
 #define PWM1_on PORTB.OUTSET =_BV(0) // PB0 out=1
#define PWM1_off PORTB.OUTCLR =_BV(0)   // PB0 out=0

#define PWM2_on PORTB.OUTSET =_BV(1)   // PB1 out=1
#define PWM2_off PORTB.OUTCLR =_BV(1)   // PC0-1 out=0

#define PWM3_on PORTB.OUTSET =_BV(2)  // PB2 out=1
#define PWM3_off PORTB.OUTCLR =_BV(2)   // PB2out=0

#define PWM4_on PORTB.OUTSET =_BV(3)   // PB3 out=1
#define PWM4_off PORTB.OUTCLR =_BV(3)   // PB3 out=0

#define PWM5_on PORTC.OUTSET =_BV(0)  // PC0 out=1
#define PWM5_off PORTC.OUTCLR =_BV(0)   // PC0 out=0
 */


/*Led indicator*/
PORTA.DIRSET = PIN4_bm; // use PA4 as LedPin
  set_txid(false);




/*configure timer TCB1 for pwm */

TCB1.CTRLA= TCB_ENABLE_bm;
TCB1.CNT=0;
TCB1.CTRLB=0; // periodic interrupt mode
TCB1.CCMP = 78*2; // 
TCB1.INTCTRL=TCB_CAPT_bm;

if (Debug ==1)
{
  Serial.begin( 115200 );
  Serial.println( "Start" );
  PWM_value1=32;
PWM_value2=32;
PWM_value3=32;
PWM_value4=32;
LedPin_on;
delay(1500);
PWM_value1=0;
PWM_value2=0;
PWM_value3=0;
PWM_value4=0;
LedPin_off;
  
}

}

void loop()
{
  uint32_t timeout = 0;
  // reset / rebind
#ifndef RX_MODE
  if (reset || ppm[AUX8] > PPM_MAX_COMMAND) {
#else
  if (reset) {
#endif
    reset = false;
#ifndef RX_MODE
    selectProtocol();
#endif
    NRF24L01_Reset();
    NRF24L01_Initialize();
    init_protocol();
  }
  // process protocol
  timeout = process_Bayang();

  // wait before sending next packet
  while (micros() < timeout)
  {   };
}

void set_txid(bool renew)
{
  uint8_t i;
  for (i = 0; i < 4; i++)
    transmitterID[i] = EEPROM.read(ee_TXID0 + i);
  if (renew || (transmitterID[0] == 0xFF && transmitterID[1] == 0x0FF)) {
    for (i = 0; i < 4; i++) {
      transmitterID[i] = random() & 0xFF;
      EEPROM.update(ee_TXID0 + i, transmitterID[i]);
    }
  }
}

void init_protocol()
{

  Bayang_init();
  Bayang_bind();
}


