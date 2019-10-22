// all servo angles/positions are written in terms of the value you would pass Servo.writeMicroseconds() to achieve that angle
// we store a copy of the most recently specified home position in EEPROM so that it persists through power cycling and code uploads

#include <Servo.h>
#include <EEPROM.h>

Servo pan_servo;
Servo tilt_servo;

uint16_t pan_home;
uint16_t tilt_home;

// hardcoded default home position (centered)
const uint16_t pan_home_default = 1410;
const uint16_t tilt_home_default = 760;

// boundaries for how far the servos should move
const uint16_t pan_min = 960; // difference between pan_min and pan_max is 90 degrees
const uint16_t pan_max = 1860;
const uint16_t tilt_min = 550; // as far as the scanner can safely tilt down
const uint16_t tilt_max = 970; // that same angle but mirrored above the default home tilt value

uint16_t pan_home_eeprom_buffer; // a buffer for storing the home pan value read from EEPROM
uint16_t tilt_home_eeprom_buffer; // a buffer for storing the home tilt value read from EEPROM
const uint8_t pan_home_eeprom_addr = 0; // the address of the stored home pan value in EEPROM
const uint8_t tilt_home_eeprom_addr = pan_home_eeprom_addr + sizeof(pan_home_eeprom_buffer); // the address of the stored home tilt value in EEPROM

// call this function ONCE
void configure_servos(int pan_pin, int tilt_pin) {
  pan_servo.attach(pan_pin);
  tilt_servo.attach(tilt_pin);

  // set the initial home to the hardcoded default on startup
  pan_home = pan_home_default;
  tilt_home = tilt_home_default;

  // attempt to read the last stored home position from EEPROM
  EEPROM.get(pan_home_eeprom_addr, pan_home_eeprom_buffer);
  EEPROM.get(tilt_home_eeprom_addr, tilt_home_eeprom_buffer);

  if (is_in_pan_range(pan_home_eeprom_buffer) && is_in_tilt_range(tilt_home_eeprom_buffer)) { // if EEPROM contained a valid home position, switch to it
    pan_home = pan_home_eeprom_buffer;
    tilt_home = tilt_home_eeprom_buffer;
  } else { // if not, throw an error
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("No home position found in EEPROM"));
    send_message(error_header, temp_body_buffer);
  }

  // move the scanner to the home position
  pan_servo.writeMicroseconds(pan_home);
  tilt_servo.writeMicroseconds(tilt_home);
}

void goto_abs_pos(uint16_t pan, uint16_t tilt) {
  if (!is_in_pan_range(pan)) { // throw an error if the specified absolute pan position is out of bounds
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Absolute pan position out of range. pos: %d, min: %d, max: %d"), pan, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!is_in_tilt_range(tilt)) { // throw an error if the specified absolute tilt position is out of bounds
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Absolute tilt position out of range. pos: %d, min: %d, max: %d"), tilt, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }

  // move the scanner to the specified position
  pan_servo.writeMicroseconds(pan);
  tilt_servo.writeMicroseconds(tilt);
}

void goto_pos(int16_t pan, int16_t tilt) {
  if (!is_in_pan_range(pan + pan_home)) { // throw an error if the specified pan position relative to home is out of bounds
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Pan position out of range. pos: %d, home: %d, min: %d, max: %d"), pan, pan_home, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!is_in_tilt_range(tilt + tilt_home)) { // throw an error if the specified tilt position relative to home is out of bounds
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Tilt position out of range. pos: %d, home: %d, min: %d, max: %d"), tilt, tilt_home, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }

  // move the scanner to home + the specified position
  goto_abs_pos(pan + pan_home, tilt + tilt_home);
}

void goto_rel_pos(int16_t pan, int16_t tilt) {
  if (!is_in_pan_range(pan + pan_servo.readMicroseconds())) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Relative pan position out of range. pos: %d, home: %d, min: %d, max: %d"), pan, pan_home, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!is_in_tilt_range(tilt + tilt_servo.readMicroseconds())) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Relative tilt position out of range. pos: %d, home: %d, min: %d, max: %d"), tilt, tilt_home, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }

  // move the scanner to the current position + the specified position
  goto_abs_pos(pan + pan_servo.readMicroseconds(), tilt + tilt_servo.readMicroseconds());
}

void gohome() {
  goto_abs_pos(pan_home, tilt_home);
}

void sethome() {
  // set the new home to the current absolute position
  pan_home = pan_servo.readMicroseconds();
  tilt_home = tilt_servo.readMicroseconds();
  // store the new home in EEPROM
  EEPROM.put(pan_home_eeprom_addr, pan_home);
  EEPROM.put(tilt_home_eeprom_addr, tilt_home);
}

void clearhome() {
  // reset the home position to be the hardcoded default
  pan_home = pan_home_default;
  tilt_home = tilt_home_default;
  // wipe the last stored home position from EEPROM
  EEPROM.put(pan_home_eeprom_addr, 0xFFFF);
  EEPROM.put(tilt_home_eeprom_addr, 0xFFFF);
}

bool is_in_pan_range(uint16_t pan) {
  return (pan >= pan_min && pan <= pan_max);
}

bool is_in_tilt_range(uint16_t tilt) {
  return (tilt >= tilt_min && tilt <= tilt_max);
}
