/* 
*/

#include "DVDPinDefs.h"
#include "mcpDac.h"
#include "ArduinoPins.h"
#include <Arduino.h>
#include "freeMemory.h"
void parseControlMessage();
boolean validateMessage();
void watchdogUSB();
void indicateLowSignal(uint16_t pd);
boolean yIsEven(uint16_t y);
void selectLaser();
void readPotentiometers();
void reportPotentiometers();
void checkUSB();
void increment_y();
void increment_z();
void increment_xyz_scan();
void laserOff();
void update_laser_focus();
void update_x_DAC(uint16_t position);
void update_y_DAC(uint16_t position);
void update_z_DAC(uint16_t position);
bool is_timed_out(unsigned long startTime_ms, unsigned long time_out_time);
boolean received_USB_message;
// number of uint16_t words of control data to send back to Processing
#define control_data_out_size 8
// number of uint18_t bytes of control data to read from Processing
#define control_data_in_size  64
// if we don't get anything on USB for 5000 msec, send x_data anyway
const unsigned long USB_DATA_TIMEOUT = 5000;
#define MAX_UNSIGNED_LONG 4294967295ULL

// pd_channel is the photodiode channel number, referenced to A0
// pd_channel 5 means input A5 which is PD_FE or focus error or (PD_AC - PD_BD)
// pd_channel 4 means input A4 which is PD_RF or photodiode rf which is OFFSET - either RF1 or RF2
// pd_channel 3 means input A3 which is PD_BD or B_SIG + D_SIG, relative to VREF (2.1v +/- .1)
// pd_channel 2 means input A2 which is PD_AC or A_SIG + C_SIG, relative to VREF (2.1v +/- .1)
uint8_t pd_channel = 5;

uint8_t control_data_in[control_data_in_size];
uint16_t control_data_out[control_data_out_size];
boolean doScan = false;
 // current value read from the pot
uint16_t laser_potValue, coarse_focus_potValue, z_potValue, fine_focus_potValue = 0;
uint16_t z_scan_step_index = 0;
uint16_t y_scan_step_index = 0;
uint16_t focus_scan_step_index = 0;
uint16_t num_z_scan_steps = 0;
uint16_t num_x_scan_steps = 216;
uint16_t num_y_scan_steps = 216;
// number of 64-bytes buffers worth of control and signal data to send
uint8_t num_data_blocks = 7;
uint16_t x_scan_center_posn = 2170;
uint16_t y_scan_center_posn = 2050;
uint8_t xy_scan_step_factor = 8;
uint8_t z_scan_step_factor = 2;
unsigned long usbTime_ms;

void setup() {
  usbTime_ms = millis();
  // initialize serial communications at 57600 bps:
  Serial.begin(9600);
  mcpDacInit();

  pinMode(LEDPin, OUTPUT);
  pinMode(SEL1_pin, OUTPUT);
  pinMode(SEL2_pin, OUTPUT);
  digitalWrite(SEL1_pin, LOW); // SEL1/SEL2 HIGH= UV SEL1/SEL2 LOW = OFF
  digitalWrite(SEL2_pin, LOW); //  SEL1 HIGH / SEL2 LOW = RED, SEL1 LOW / SEL2 HIGH = IR
  received_USB_message = false;
  mcpDac3BSend(1675); //Set Amp Offset for photodiode RF
  // flush Serial port buffer
  while (Serial.available() > 0) {Serial.read();}
}

void loop(){
  watchdogUSB();
  if(received_USB_message){

    update_laser_focus();   // Send the new laser and focus control data to the DACs
    readPotentiometers();   // read the focus, laser intensity and z potentiometers
    // we're scanning in x
    // TODO only increment if space available in output buffer
    //if (Serial.availableForWrite() > 2){
      increment_xyz_scan();
    //}
    received_USB_message = false;
  } else {
    checkUSB();            // Read control message from USB Serial port
  }
}

boolean validateMessage(){
  // test received message for start and stop tags
  return   control_data_in[0] == 0 && control_data_in[1] == 123
            && control_data_in[18] == 0 && control_data_in[19] == 123;
}

void watchdogUSB() {
  if (is_timed_out(usbTime_ms, USB_DATA_TIMEOUT)){
    increment_xyz_scan();

  }
}

void checkUSB() {

  byte _serialInput = 0;
  // Read bytes if available
  if (Serial.available() > 63 ) {
    _serialInput = Serial.readBytes(control_data_in, 64);
    //control_data_in[0] = y_scan_step_index;
  }
  // Set message received true if we have all words and
  // message is validated with 
  if (_serialInput == 64){

    received_USB_message = true;
    parseControlMessage();
  } else {
    // haven't yet read enough bytes for this to be the complete message
    received_USB_message = false;
  }
}

void parseControlMessage(){
  uint16_t temp = 0;
  // we have a validated message
  // Parse message, check limits
  temp = control_data_in[0] * 256 + control_data_in[1];
  if (temp != num_z_scan_steps){
    // reset scan
    y_scan_step_index = 0;
    z_scan_step_index = 0;
  }
  num_z_scan_steps = temp;
  temp = control_data_in[2] * 256 + control_data_in[3];
  if (temp != num_x_scan_steps){
    // reset scan
    y_scan_step_index = 0;
    z_scan_step_index = 0;
  }
  num_x_scan_steps = temp;
  temp = control_data_in[4] * 256 + control_data_in[5];
  if (temp != num_y_scan_steps){
    // reset scan
    y_scan_step_index = 0;
    z_scan_step_index = 0;
  }
  num_y_scan_steps = temp;
  num_data_blocks = (num_x_scan_steps + 8) / 32 ;
  x_scan_center_posn = control_data_in[6] * 256 + control_data_in[7];
  y_scan_center_posn = control_data_in[8] * 256 + control_data_in[9];
  xy_scan_step_factor = control_data_in[10] * 256 + control_data_in[11];
  pd_channel = control_data_in[12] * 256 + control_data_in[13];
  doScan = control_data_in[15] == 1;
  z_scan_step_factor = control_data_in[18] * 256 + control_data_in[19];
  // if num_x_scan_steps changed, or num_z_scan_steps changed, restart scan from 0, 0, 0
}

void increment_xyz_scan(){
  // low/high byte of photodiode signal versus x_scan_step_indx
  uint8_t signal[2 * num_x_scan_steps];
  // buffer containing 64-byte chunks of the signal array
  uint8_t signalOut[64];
  uint16_t x_scan_posn = 0;
  for (uint16_t x_scan_step_index = 0; x_scan_step_index < num_x_scan_steps; x_scan_step_index++){
    if (yIsEven(y_scan_step_index)){
      x_scan_posn = x_scan_center_posn + ((x_scan_step_index - (num_x_scan_steps / 2)) * xy_scan_step_factor);
    } else {
      x_scan_posn = x_scan_center_posn - ((x_scan_step_index - (num_x_scan_steps / 2)) * xy_scan_step_factor);
    }
    delayMicroseconds(100);
    // move x_scanner
    //modulate laser
    if (x_scan_step_index % 8 < 4){
      laserOff();
    } else {
      selectLaser();
    }
    if (doScan){
      update_x_DAC(x_scan_posn);
    }
    uint16_t photodiode_intensity = analogRead(A0 + pd_channel);
    //indicateLowSignal(photodiode_intensity);
    // temporary "data" for debugging
    uint16_t data_out = x_scan_step_index + 200 + y_scan_step_index;
    data_out = photodiode_intensity;
    signal[2 * x_scan_step_index + 1] = (data_out & 0xff);
    signal[2 * x_scan_step_index] = (data_out >> 8);
  }

  //Sending data back to Processing
  uint8_t byte_num = 0;
  uint8_t block_num = 0;
  uint16_t x_scan_data_index = 0;
  for (uint8_t word_number = 0; word_number < control_data_out_size; word_number++){ // for control_data_out
    byte_num = 2 * word_number; // to put in signalOut[] buffer
    signalOut[byte_num + 1] = (control_data_out[word_number] & 0xff); // hi byte
    signalOut[byte_num] = (control_data_out[word_number] >> 8); // lo byte
  }
  for (uint8_t word_number = control_data_out_size; word_number < 32; word_number++){
    // for first signal data
    byte_num = 2 * word_number; // to put in signalOut[] buffer
    // we've already converted the signal to bytes
    x_scan_data_index = 2 * (word_number - control_data_out_size);
    signalOut[byte_num] = signal[x_scan_data_index]; // hi byte
    signalOut[byte_num + 1] = signal[x_scan_data_index + 1]; // lo byte
  }
  Serial.write(signalOut, 64);
// now send out the rest of the signal data

  for (block_num = 1; block_num < num_data_blocks; block_num++){
    //memcpy(dst, src, sizeof(src[0])*len)
    memcpy(signalOut, signal + 64 * block_num - 2 * control_data_out_size, 64);
    Serial.write(signalOut, 64);
  }
  increment_y();
  usbTime_ms = millis();
}

void increment_y(){
  y_scan_step_index++;
  if(y_scan_step_index == num_y_scan_steps){// y_scan is ramp
    y_scan_step_index = 0;
    increment_z();
  }
  uint16_t y_scan_posn = y_scan_center_posn + ((y_scan_step_index - (num_y_scan_steps / 2)) * xy_scan_step_factor);
  if (doScan){
    update_y_DAC(y_scan_posn);
  }
  // TODO include z-scan values, but only if initScan == true
  update_z_DAC(z_potValue << 2);// this will only respond to potentiometer value
}

void increment_z(){}

// On even y-scan rows, the x-scan reverses direction. 
// Perhaps this speeds up x-scanning or we don't have to 'de=bouce' the x-scan
boolean yIsEven(uint16_t y) {
  return (y % 2 == 0);
}

//if photodiode intensity is low, turn on LED. Then we would increase laser intensity, perhaps
void indicateLowSignal(uint16_t pd){
  if (pd < 350) {
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void laserOff(){
  digitalWrite(SEL1_pin, LOW);
  digitalWrite(SEL2_pin, LOW);
}

void selectLaser(){
  // SEL1/SEL2 HIGH= UV SEL1/SEL2 LOW = OFF
  // SEL1 HIGH / SEL2 LOW = RED, SEL1 LOW / SEL2 HIGH = IR
  switch(control_data_in[17]){
    case 0:
    // laser off
      laserOff();
    break;
    case 1:
    // laser IR
      digitalWrite(SEL1_pin, LOW);
      digitalWrite(SEL2_pin, HIGH);
    break;
    case 2:
    // laser Red
      digitalWrite(SEL1_pin, HIGH);
      digitalWrite(SEL2_pin, LOW);
    break;
    case 3:
    // laser UV
      digitalWrite(SEL1_pin, HIGH);
      digitalWrite(SEL2_pin, HIGH);
    break;
    default:
    // laser off
      digitalWrite(SEL1_pin, LOW);
      digitalWrite(SEL2_pin, LOW);    
  }
}

void update_x_DAC(uint16_t _x_scan_position){
  mcpDac2BSend(_x_scan_position); //Drive reading head X
}

void update_y_DAC(uint16_t _y_scan_position){
  mcpDac1ASend(_y_scan_position); //Drive stage Y
}

void update_z_DAC(uint16_t _z_scan_position){
  mcpDac1BSend(_z_scan_position); //Drive stage Z
}

void update_laser_focus(){
  selectLaser();
  mcpDac3ASend(laser_potValue << 2); //Current for Laser Diode

  // '<< 2' multiplies coarse_focus_pot by 4; range of coarse pot is 0 - 4096 with step of 4
  // '>> 2' divides fine_focus_pot by 4; range of fine pot is 0 - 256
  mcpDac2ASend((coarse_focus_potValue << 2) + (fine_focus_potValue >> 2)); //Drive reading head focus
}

// read the value of the 4 potentiometers according to which input port they're attached to
void readPotentiometers() {
    // if (current time - last_pot_readtime > 1) {}
    // potentiometers mounted so clockwise rotation decreases value; 
    // invert by subtracting analog value from 1024
  uint16_t laser_potValueRaw = 1024 - analogRead(pot_rv1_pin);
  uint16_t coarse_focus_potValueRaw = 1024 - analogRead(pot_rv2_pin);
  uint16_t z_potValueRaw = 1024 - analogRead(pot_rv3_pin);
  uint16_t fine_focus_potValueRaw = 1024 - analogRead(pot_rv4_pin);
 // add hysteresis
  if ((laser_potValueRaw - laser_potValue) > 2) {
    laser_potValue = laser_potValueRaw - 2;
  }
  if ((laser_potValue - laser_potValueRaw) > 2) {
    laser_potValue = laser_potValueRaw + 2;
  }

  if ((coarse_focus_potValueRaw - coarse_focus_potValue) > 2) {
    coarse_focus_potValue = coarse_focus_potValueRaw - 2;
  }
  if ((coarse_focus_potValue - coarse_focus_potValueRaw) > 2) {
    coarse_focus_potValue = coarse_focus_potValueRaw + 2;
  }

  if ((z_potValueRaw - z_potValue) > 2) {
    z_potValue = z_potValueRaw - 2;
  }
  if ((z_potValue - z_potValueRaw) > 2) {
    z_potValue = z_potValueRaw + 2;
  }

  if ((fine_focus_potValueRaw - fine_focus_potValue) > 2) {
    fine_focus_potValue = fine_focus_potValueRaw - 2;
  }
  if ((fine_focus_potValue - fine_focus_potValueRaw) > 2) {
    fine_focus_potValue = fine_focus_potValueRaw + 2;
  }
  control_data_out[0] = laser_potValue;
  control_data_out[1] = coarse_focus_potValue;
  control_data_out[2] = fine_focus_potValue;
  control_data_out[3] = z_potValue;                                    
  control_data_out[4] = y_scan_step_index; // current y_scan_index
  control_data_out[5] = z_scan_step_index; // current z_scan_index
  control_data_out[6] = num_x_scan_steps;
  control_data_out[7] = control_data_in[17];

}

bool is_timed_out(unsigned long startTime_ms, unsigned long time_out_time) {
  unsigned long delta_time = millis() - startTime_ms;
  if (delta_time < 0) {
    // rolled-over since we started! startTime_ms is really big, millis() is very small.
    // delta_time is negative, have to add the maximum unsigned long
    delta_time += MAX_UNSIGNED_LONG;
  }
  return (delta_time > time_out_time);
}
