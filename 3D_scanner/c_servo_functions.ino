#include <Servo.h>
#include <EEPROM.h>

Servo pan_servo;
Servo tilt_servo;

uint16_t pan_home;
uint16_t tilt_home;

const uint16_t pan_home_default = 1410;
const uint16_t tilt_home_default = 760;

const uint16_t pan_min = 960;
const uint16_t pan_max = 1860;
const uint16_t tilt_min = 550;
const uint16_t tilt_max = 970;

uint16_t pan_home_eeprom_buffer;
uint16_t tilt_home_eeprom_buffer;
const uint8_t pan_home_eeprom_addr = 0;
const uint8_t tilt_home_eeprom_addr = pan_home_eeprom_addr + sizeof(pan_home_eeprom_buffer);

void configure_servos(int pan_pin, int tilt_pin) {
  pan_servo.attach(pan_pin);
  tilt_servo.attach(tilt_pin);

  pan_home = pan_home_default;
  tilt_home = tilt_home_default;

  EEPROM.get(pan_home_eeprom_addr, pan_home_eeprom_buffer);
  EEPROM.get(tilt_home_eeprom_addr, tilt_home_eeprom_buffer);

  if (is_in_pan_range(pan_home_eeprom_buffer) && is_in_tilt_range(tilt_home_eeprom_buffer)) {
    pan_home = pan_home_eeprom_buffer;
    tilt_home = tilt_home_eeprom_buffer;
  } else {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("No home position found in EEPROM"));
    send_message(error_header, temp_body_buffer);
  }

  pan_servo.writeMicroseconds(pan_home);
  tilt_servo.writeMicroseconds(tilt_home);
}

void goto_abs_pos(uint16_t pan, uint16_t tilt) {
  if (!is_in_pan_range(pan)) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Absolute pan position out of range. pos: %d, min: %d, max: %d"), pan, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!is_in_tilt_range(tilt)) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Absolute tilt position out of range. pos: %d, min: %d, max: %d"), tilt, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  
  pan_servo.writeMicroseconds(pan);
  tilt_servo.writeMicroseconds(tilt);
}

void goto_pos(int16_t pan, int16_t tilt) {
  if (!is_in_pan_range(pan + pan_home)) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Pan position out of range. pos: %d, home: %d, min: %d, max: %d"), pan, pan_home, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!is_in_tilt_range(tilt + tilt_home)) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Tilt position out of range. pos: %d, home: %d, min: %d, max: %d"), tilt, tilt_home, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  
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
  
  goto_abs_pos(pan + pan_servo.readMicroseconds(), tilt + tilt_servo.readMicroseconds());
}

void gohome() {
  goto_abs_pos(pan_home, tilt_home);
}

void sethome() {
  pan_home = pan_servo.readMicroseconds();
  tilt_home = tilt_servo.readMicroseconds();
  EEPROM.put(pan_home_eeprom_addr, pan_home);
  EEPROM.put(tilt_home_eeprom_addr, tilt_home);
}

void clearhome() {
  pan_home = pan_home_default;
  tilt_home = tilt_home_default;
  EEPROM.put(pan_home_eeprom_addr, 0xFFFF);
  EEPROM.put(tilt_home_eeprom_addr, 0xFFFF);
}

bool is_in_pan_range(uint16_t pan) {
  return (pan >= pan_min && pan <= pan_max);
}

bool is_in_tilt_range(uint16_t tilt) {
  return (tilt >= tilt_min && tilt <= tilt_max);
}
