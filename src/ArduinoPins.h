#include <Arduino.h>
// These constants won't change.  They're used to give names
// to the pins used:
const int SEL2_pin = A0; // the number of the Laser select 2 pin
const int SEL1_pin = A1; // the number of the Laser select 1 pin

const int pot_rv1_pin = A8;      // Analog input pin for top-left potentiometer
const int pot_rv2_pin = A9;     // Analog input pin for top-right potentiometer
const int pot_rv3_pin = A10;  // Analog input pin for bottom-left potentiometer
const int pot_rv4_pin = A11; // Analog input pin for bottom-right potentiometer

#define  LEDPin 7     // Analog output pin that the D1 LED is attached to
#define  buttonPin 11 // the number of the pushbutton pin