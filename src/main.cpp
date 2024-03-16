/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "Arduino.h"
#include "HardwareSerial.h"
#include "sbus.h"

template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }

#define SBUS_RX_PIN 14
#define ODRIVE_RX_PIN 17
#define ODRIVE_TX_PIN 16

#define DEBUG
// #define DEBUG_ODRIVE
#define DEBUG_ODRIVE_STARTUP
// #define DEBUG_MISSING

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX_PIN, 03, true);
/* SBUS data */
bfs::SbusData data;

/* ODRIVE */
HardwareSerial odriveSerial = Serial2;

float scalingFactor = 250.0f;
float deadZone = 30.0f;

int i = 0;
int no_comms = 0;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  //while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  /* ODRIVE */
  // odriveSerial.setPins();
  // odriveSerial.begin(115200, , ODRIVE_RX_PIN, ODRIVE_TX_PIN);
  odriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    odriveSerial << "r vbus_voltage\n";
   #ifdef DEBUG_ODRIVE_STARTUP
      while (!odriveSerial.available()) {
        Serial.println("waiting for vbus...");
        sleep(2);
      }
      while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }
      #endif
  odriveSerial << "w axis0.requested_state 8\n";
  odriveSerial << "w axis1.requested_state 8\n";
  #ifdef DEBUG_ODRIVE_STARTUP
  Serial.println("sent state change...");
      while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }
      #endif
  odriveSerial << "r axis0.requested_state\n";
  odriveSerial << "r axis1.requested_state\n";
   #ifdef DEBUG_ODRIVE_STARTUP
   Serial.println("reading state change...");
      while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }
      #endif
}

void loop () {
  // Serial.print("HELLO");
  // Serial.print("\n");
  
  if (sbus_rx.Read()) {
    no_comms=0;
    /* Grab the received data */
    data = sbus_rx.data();
    #ifdef DEBUG
    /* Display the received data */
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    /* Display lost frames and failsafe data */
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
    Serial.print("\n");
    #endif
    int stopped = data.ch[2] > 900;
    if (stopped) {
      odriveSerial << "v 0 0\n";
      odriveSerial << "v 1 0\n";
    } else {
      float mot0 = (data.ch[0] - 960.0f);
    if (abs(mot0) > deadZone) {
      odriveSerial << "v 0 " << -(mot0-(mot0>0?1:-1)*deadZone)/scalingFactor << "\n";
      #ifdef DEBUG_ODRIVE
      Serial.print("Sent command to odrive v 0 ");
      Serial.println((mot0-(mot0>0?-1:1)*deadZone)/scalingFactor);
      #endif
    } else {
      odriveSerial << "v 0 0\n";
      #ifdef DEBUG_ODRIVE
      Serial.print("Sent command to odrive v 0 ");
      Serial.println("0");
      #endif
    }
    #ifdef DEBUG_ODRIVE
      while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }
      #endif
    float mot1 = (data.ch[1] - 960.0f);
    if (abs(mot1) > deadZone) {
      odriveSerial << "v 1 " << (mot1-(mot1>0?1:-1)*deadZone)/scalingFactor << "\n";
      #ifdef DEBUG_ODRIVE
      Serial.print("Sent command to odrive v 1 ");
      Serial.println((mot1-(mot1>0?-1:1)*deadZone)/scalingFactor);
      #endif
    } else {
      odriveSerial << "v 1 0\n";
      #ifdef DEBUG_ODRIVE
      Serial.print("Sent command to odrive v 1 ");
      Serial.println("0");
      #endif
    }
    #ifdef DEBUG_ODRIVE
      while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }
      #endif
    }
    
  } else {
    #ifdef DEBUG_MISSING
    if (i++ % 10 == 0) {
      Serial.println("NO DATA");
    }
    #endif
    while (odriveSerial.available()) {
        Serial.write(odriveSerial.read());
      }

    if (++no_comms % 1000 == 0) {
      odriveSerial << "v 0 0\n";
      odriveSerial << "v 1 0\n";
       Serial.println("NO frame recieved; stopping");
      }
  }
  
}