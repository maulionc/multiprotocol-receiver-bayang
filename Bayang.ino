/*
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
 */

#define BAYANG_BIND_COUNT       1000
#define BAYANG_PACKET_PERIOD    2000
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5

static uint8_t Bayang_rf_chan = 0;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];


enum{
    // flags going to packet[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
    BAYANG_FLAG_FLIP     = 0x08,
    BAYANG_FLAG_VIDEO    = 0x10,
    BAYANG_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    BAYANG_FLAG_INVERT   = 0x80,
};

uint32_t process_Bayang()
{
    uint32_t timeout = micros() + BAYANG_PACKET_PERIOD;

    // timeout = micros() + BAYANG_PACKET_PERIOD/10;
    Bayang_recv_packet();
    return timeout;
}

void Bayang_init()
{
    uint8_t bind_address[] = {0,0,0,0,0};

    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(RX_EN);

    XN297_SetTXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
#ifdef RX_MODE
    XN297_SetRXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
#endif
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
#ifdef RX_MODE
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE); // rx pipe 0
#endif
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // address size
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_250K);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);

#ifdef RX_MODE
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
#endif
}

void Bayang_bind()
{
 Bayang_bind_rx();
}

void Bayang_bind_rx()
{
    int bind_count = 0;
    uint8_t bind_packet[BAYANG_PACKET_SIZE] = {0};

    uint32_t timeout;

    //digitalWrite(ledPin, LOW);
    LedPin_off;

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, BAYANG_RF_BIND_CHANNEL);

    while(bind_count < 10) {
      if(Debug==1)
      {
      Serial.println("bind_count");
      Serial.println(bind_count);
      }

        timeout = millis()+5;

        while(millis()<timeout) {
          if (Debug==1)
          {
          Serial.println("timeout");
          Serial.println(timeout); 
          }

            delay(1);
            if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from tx

              if (Debug ==1)
              {
               Serial.println("NRF24L01_07_STATUS");
Serial.println(NRF24L01_07_STATUS);
Serial.println("NRF24L01_07_RX_DR");
Serial.println(NRF24L01_07_RX_DR); 
              }

              
                XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();

                if( packet[0] == 0xA4)
                {
                  if (0 == bind_count)
                  {
                    memcpy(bind_packet, packet, BAYANG_PACKET_SIZE);
                    ++bind_count;
                  }
                  else
                  {
                    if (0 == memcmp(bind_packet, packet, BAYANG_PACKET_SIZE))
                      ++bind_count;
                  }
                }
                break;
            }
        }
    }

    memcpy(Bayang_rx_tx_addr, &packet[1], 5);
    memcpy(Bayang_rf_channels, &packet[6], 4);
    transmitterID[0] = packet[10];
    transmitterID[1] = packet[11];

    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    XN297_SetRXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, Bayang_rf_channels[Bayang_rf_chan++]);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
   //digitalWrite(ledPin, HIGH);
    LedPin_on;
}

#define DYNTRIM(chval) ((uint8_t)((chval >> 2) & 0xfc))
void Bayang_recv_packet()
{
  static bool is_bound = false;
  static uint16_t failsafe_counter = 0;
  if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from tx
    is_bound = true;
    failsafe_counter = 0;

    int sum = 0;
    uint16_t roll, pitch, yaw, throttle, output1, output2, output3, output4;
    XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

    if( packet[0] == 0xA4)
    {
      // bind packet

      if (Debug==1)
      {
            Serial.println("bind packet"); 
      }
 
      
    }
    else if (packet[0] == 0xA5)
    {
      // data packet
      for(int i=0; i<14; i++)
      {
        sum += packet[i];
      }
      if ( (sum&0xFF) == packet[14] )
      {
        // checksum OK
        roll = (packet[4] & 0x0003) * 256 + packet[5];
        pitch = (packet[6] & 0x0003) * 256 + packet[7];
        yaw = (packet[10] & 0x0003) * 256 + packet[11];
        throttle = (packet[8] & 0x0003) * 256 + packet[9];
        output1 = (packet[2] & 0x08);
        output2 = (packet[2] & 0x01);
        output3 = (packet[2] & 0x02);
        output4 = (packet[3] & 0x80);

              if(Debug==1)
      {
         Serial.print(roll, DEC); Serial.print(",");
         Serial.print(pitch, DEC); Serial.print(",");
         Serial.print(yaw, DEC); Serial.print(",");
         Serial.print(throttle, DEC); Serial.println("");
      }


       //  PWM_value = throttle/32;
       PWM_value_throttle =   map(throttle, 0, 1023, 0, steps);

 //channel roll
      if(roll < (mid_value - delta)){
        roll_out = map(roll, 0, mid_value - delta, steps, 0);
        //if(roll < off_value) roll = 0;
        PWM_value1 = roll_out;
        PWM_value2 = 0;
        if(Debug==1)
        {
          Serial.print(roll_out, DEC); Serial.println(",");
        }
      }
      if(roll > (mid_value + delta)){
        roll_out = map(roll, mid_value + delta, 1023, 0, steps);
      //  if(roll < off_value) roll = 0;       
        PWM_value1 = 0;
        PWM_value2 = roll_out;
          if(Debug==1)
        {
          Serial.print(roll_out, DEC); Serial.println(",");
        }
     }

           if(roll <(mid_value + delta) && roll >(mid_value - delta))
           {      
        PWM_value1 = 0;
        PWM_value2 = 0;
           }
    

     //channel pitch
      if(pitch < (mid_value - delta)){
        pitch_out = map(pitch, 0, mid_value - delta, steps, 0);
        //if(roll < off_value) roll = 0;
        PWM_value3 = pitch_out;
        PWM_value4 = 0;
        if(Debug==1)
        {
          Serial.print(pitch_out, DEC); Serial.println(",");
        }
      }
      if(pitch > (mid_value + delta)){
        pitch_out = map(pitch, mid_value + delta, 1023, 0, steps);
      //  if(roll < off_value) roll = 0;       
        PWM_value3 = 0;
        PWM_value4 = pitch_out;
          if(Debug==1)
        {
          Serial.print(pitch_out, DEC); Serial.println(",");
        }
     }

           if(pitch <(mid_value + delta) && pitch >(mid_value - delta))
           {      
        PWM_value3 = 0;
        PWM_value4 = 0;
           }
     
/*
        // roll: 0 .. 1023; servo 0 .. 180 but stop at +-80ï¿½ due to reduced holding current
        servo0.write( - asin( throttle / 511.5 - 1.0 ) / ( 3.1416 / 2 ) * 180.0 + 90 );  //signal for motor ESC, forward/reverse.
        servo1.write( - asin( roll / 511.5 - 1.0 ) / ( 3.1416 / 2 ) * 80.0 + 90 );     //change "80.0" to other servo degrees what you need.
		    servo2.write( - asin( pitch / 511.5 - 1.0 ) / ( 3.1416 / 2 ) * 60.0 + 90 );   //change "60.0" to other servo degrees what you need.
        servo3.write( - asin( yaw / 511.5 - 1.0 ) / ( 3.1416 / 2 ) * 40.0 + 90 );   //change "40.0" to other servo degrees what you need.
        // throttle: 0 .. 1023; pwm 0 .. 255
        // analogWrite( 6, throttle / 4  );  //uncomment and use this, if you need simple one direction motor speed.
        digitalWrite ( 10, output1 );
        digitalWrite ( 11, output2 );
        digitalWrite ( 12, output3 );
        digitalWrite ( 13, output4 );
        */
      }
      else
      {
      if(Debug==1)
      {
                //checksum FAIL
        Serial.println("checksum FAIL");
      }

      }
      NRF24L01_WriteReg(NRF24L01_05_RF_CH, Bayang_rf_channels[Bayang_rf_chan++]);
      Bayang_rf_chan %= sizeof(Bayang_rf_channels);
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
      NRF24L01_FlushRx();
    }
    else
    {
      if(Debug==1)
      {
        Serial.print("Unrecognized packet: ");
        Serial.println(packet[0], HEX);
      }


    }
  } else if ( is_bound ) {
        ++failsafe_counter;
        if ( failsafe_counter > 1000 ) {
            is_bound = false;
            failsafe_counter = 0;
            /*
            // neutral steering position; motor off:
            servo1.write( 90 );
            servo2.write( 90 );
            servo3.write( 90 );
            servo0.write( 90 );
            analogWrite( 6, 0 );
            */
            PWM_value1=0;
            PWM_value2=0;
            PWM_value3=0;
            PWM_value4=0;
            PWM_value_throttle=0;
            // enable rebinding:
            extern bool reset;
            reset = true;
            Serial.println("reset");
        }
  }
}

void Bayang_send_packet(uint8_t bind) //fonctionne
//void Bayang_send_packet(u8 bind)
{
    union {
        uint16_t value;
        struct {
            uint8_t lsb;
            uint8_t msb;
        } bytes;
    } chanval;

    if (bind) {
        packet[0] = 0xa4;
        memcpy(&packet[1], Bayang_rx_tx_addr, 5);
        memcpy(&packet[6], Bayang_rf_channels, 4);
        packet[10] = transmitterID[0];
        packet[11] = transmitterID[1];
    } else {
        packet[0] = 0xa5;
        packet[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
        packet[2] = GET_FLAG(AUX2, BAYANG_FLAG_FLIP)
                  | GET_FLAG(AUX5, BAYANG_FLAG_HEADLESS)
                  | GET_FLAG(AUX6, BAYANG_FLAG_RTH)
                  | GET_FLAG(AUX3, BAYANG_FLAG_SNAPSHOT)
                  | GET_FLAG(AUX4, BAYANG_FLAG_VIDEO);
        packet[3] = GET_FLAG(AUX1, BAYANG_FLAG_INVERT);
        chanval.value = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 0x3ff);   // aileron
        packet[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[5] = chanval.bytes.lsb;
        chanval.value = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 0x3ff);   // elevator
        packet[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[7] = chanval.bytes.lsb;
        chanval.value = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 0x3ff);   // throttle
        packet[8] = chanval.bytes.msb + 0x7c;
        packet[9] = chanval.bytes.lsb;
        chanval.value = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 0x3ff);   // rudder
        packet[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[11] = chanval.bytes.lsb;
    }
    packet[12] = transmitterID[2];
    packet[13] = 0x0a;
    packet[14] = 0;
    for(uint8_t i=0; i<BAYANG_PACKET_SIZE-1; i++) {
        packet[14] += packet[i];
    }

    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? BAYANG_RF_BIND_CHANNEL : Bayang_rf_channels[Bayang_rf_chan++]);
    Bayang_rf_chan %= sizeof(Bayang_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WritePayload(packet, BAYANG_PACKET_SIZE);
}

