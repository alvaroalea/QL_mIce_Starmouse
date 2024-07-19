/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This code is based on the example that demonstrates use of both device and host, where
 * - Device run on native usb controller (roothub port0)
 * - Host depending on MCUs run on either:
 *   - rp2040: bit-banging 2 GPIOs with the help of Pico-PIO-USB library (roothub port1)
 *
 * Requirements:
 * - For rp2040:
 *   - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 *   - 2 consecutive GPIOs: D+ is defined by PIN_USB_HOST_DP, D- = D+ +1
 *   - Provide VBus (5v) and GND for peripheral
 *   - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

/* 
THIS CODE IS ALSO BASED ON:

     RP2040 - USB to quadrature mouse converter
    Copyright (C) 2023 Darren Jones
    Copyright (C) 2017-2020 Simon Inns

  This file is part of RP2040 Mouse based on the original SmallyMouse from Simon Inns.

    RP2040 Mouse is free software: you can redistribute it and/or modify
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

/*
 * Compile it on arduino using this toolchain: https://github.com/earlephilhower/arduino-pico
 * config in Tools-> CPU Speed: 120Mhz
 *           Tools-> USB Stack: Adafruit TinyUSB
 
 */

// USBHost is defined in usbh_helper.h
#define PIN_USB_HOST_DP  11
//#define PIN_5V_EN        6
//#define PIN_5V_EN_STATE  1
#include "usbh_helper.h"


volatile int8_t mouseDirectionX = 0;    // X direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseX = 0; // X Quadrature phase (0-3)

volatile int8_t mouseDirectionY = 0;    // Y direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseY = 0; // Y Quadrature phase (0-3)

volatile int16_t mouseDistanceX = 0; // Distance left for mouse to move
volatile int16_t mouseDistanceY = 0; // Distance left for mouse to move

#define XA_PIN 13
#define XB_PIN 26
#define YA_PIN 15
#define YB_PIN 14
#define JOY_F_PIN 1
#define JOY_U_PIN 2
#define JOY_D_PIN 3
#define JOY_R_PIN 4
#define JOY_L_PIN 5

static bool config;

#if defined(ARDUINO_ARCH_RP2040)
//--------------------------------------------------------------------+
// For RP2040 use both core0 for device stack, core1 for host stack
//--------------------------------------------------------------------+



// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "RPi_Pico_ISR_Timer.h"
#define TIMER_INTERVAL_MS 1L
// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);
RPI_PICO_ISR_Timer ISR_timer;

struct repeating_timer timer1;

//------------- Core0 -------------//

bool timer1_callback(struct repeating_timer *t)
{
  // Silence compilation warning
  (void)t;
  if (config) {
    // Process X output
    if (mouseDistanceX != 0) {
      if (mouseDistanceX > 0) {
        mouseDistanceX--;
        Serial.println("X- mov");
        mouseEncoderPhaseX++;
        if (mouseEncoderPhaseX > 3) mouseEncoderPhaseX = 0;
      } else if (mouseDistanceX < 0) {
        mouseDistanceX++;
        Serial.println("X+ mov");
        mouseEncoderPhaseX--;
        if (mouseEncoderPhaseX < 0) mouseEncoderPhaseX = 3;
      }
      // Set the output pins according to the current phase
      if (mouseEncoderPhaseX == 0) digitalWrite(XA_PIN, 1); // Set X1 to 1
      if (mouseEncoderPhaseX == 1) digitalWrite(XB_PIN, 1); // Set X2 to 1
      if (mouseEncoderPhaseX == 2) digitalWrite(XA_PIN, 0); // Set X1 to 0
      if (mouseEncoderPhaseX == 3) digitalWrite(XB_PIN, 0); // Set X2 to 0
    }
    // Process Y output
    if (mouseDistanceY != 0) {
      if (mouseDistanceY > 0) {
        mouseDistanceY--;
        Serial.println("Y- mov");
        mouseEncoderPhaseY++;
        if (mouseEncoderPhaseY > 3) mouseEncoderPhaseY = 0;
      } else if (mouseDistanceY < 0) {
        mouseDistanceY++;
        Serial.println("Y+ mov");
        mouseEncoderPhaseY--;
        if (mouseEncoderPhaseY < 0) mouseEncoderPhaseY = 3;
      }
      // Set the output pins according to the current phase
      if (mouseEncoderPhaseY == 0) digitalWrite(YA_PIN, 1); // Set Y1 to 1
      if (mouseEncoderPhaseY == 1) digitalWrite(YB_PIN, 1); // Set Y2 to 1
      if (mouseEncoderPhaseY == 2) digitalWrite(YA_PIN, 0); // Set Y1 to 0
      if (mouseEncoderPhaseY == 3) digitalWrite(YB_PIN, 0); // Set Y2 to 0
    }
  } else {
    if (mouseDistanceX > 0) {
      mouseDistanceX--;
      digitalWrite(JOY_L_PIN,0); // left
      digitalWrite(JOY_R_PIN,1); // right
    } else if (mouseDistanceX < 0) {
      mouseDistanceX++;
      digitalWrite(JOY_L_PIN,1); // left
      digitalWrite(JOY_R_PIN,0); // right
    } else {
      digitalWrite(JOY_L_PIN,0); // left
      digitalWrite(JOY_R_PIN,0); // right
    }
    if (mouseDistanceY > 0) {
      mouseDistanceY--;
      digitalWrite(JOY_U_PIN,0); // left
      digitalWrite(JOY_D_PIN,1); // right
    } else if (mouseDistanceY < 0) {
      mouseDistanceY++;
      digitalWrite(JOY_U_PIN,1); // left
      digitalWrite(JOY_D_PIN,0); // right
    } else {
      digitalWrite(JOY_U_PIN,0); // left
      digitalWrite(JOY_D_PIN,0); // right
    }
  }
  return true;
}

void setup() {
  //mice buttons
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  digitalWrite(27,1);//left button, buttons are inverted
  digitalWrite(28,1); //right button
  digitalWrite(29,1);//central button

  //mice xy
  pinMode(XA_PIN, OUTPUT);
  pinMode(XB_PIN, OUTPUT);
  pinMode(YA_PIN, OUTPUT);
  pinMode(YB_PIN, OUTPUT);
  //joystick output
  pinMode(JOY_L_PIN, OUTPUT);
  pinMode(JOY_R_PIN, OUTPUT);
  pinMode(JOY_D_PIN, OUTPUT);
  pinMode(JOY_U_PIN, OUTPUT);
  pinMode(JOY_F_PIN, OUTPUT);
  digitalWrite(JOY_L_PIN,0);
  digitalWrite(JOY_R_PIN,0);
  digitalWrite(JOY_D_PIN,0);
  digitalWrite(JOY_U_PIN,0);
  digitalWrite(JOY_F_PIN,0);

  // CFG switch
  pinMode(0, INPUT);

  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("USB Mouse to BusMouse/Joystick Adapter, (C) A.Alea 2024 V0.1");

  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, timer1_callback))
    {
      Serial.print(F("Starting ITimer1 OK, millis() = ")); Serial.println(millis());
    } else {
      Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    }
}

void loop() {
  Serial.flush();
  bool oldcfg = config;
  config=digitalRead(0);
  if (oldcfg!=config) {
    if (config==0) Serial.println("CONFIG: Joystick Emulation");
    if (config==1) {
        Serial.println("CONFIG: mICE Mouse");
        digitalWrite(JOY_L_PIN,0);
        digitalWrite(JOY_R_PIN,0);
        digitalWrite(JOY_D_PIN,0);
        digitalWrite(JOY_U_PIN,0);
        digitalWrite(JOY_F_PIN,0);
    }
  }
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

void loop1() {
  USBHost.task();
}

#endif

extern "C" {

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void) desc_report;
  (void) desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  Serial.printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  Serial.printf("VID = %04x, PID = %04x\r\n", vid, pid);
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    Serial.printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  Serial.printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  Serial.printf("HIDreport : ");
  for (uint16_t i = 0; i < len; i++) {
    Serial.printf("0x%02X ", report[i]);
  }
  Serial.println();
  if (len==3){
      mouseDistanceX+=(int8_t)report[1];
      mouseDistanceY+=(int8_t)report[2];
    if (config){ //mice
      digitalWrite(27,(report[0]&0x1)?0:1);//left button
      digitalWrite(28,(report[0]&0x2)?0:1); //right button
      digitalWrite(29,(report[0]&0x4)?0:1);//central button
    } else { //joystick or 1 button mouse
      digitalWrite(JOY_F_PIN,(report[0]!=0)?1:0); // Any button
    }
  }
  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    Serial.printf("Error: cannot request to receive report\r\n");
  }
}

} // extern C