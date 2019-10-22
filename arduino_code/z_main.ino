// This acts as our true main file setup() and loop() are both called.
// It is also where we implement the scanning function, as this utilizes the servos, sensor, and serial concurrently

// variables for storing the parameters of an active scan
int16_t scan_pan_start; // the starting pan angle of the current scan
int16_t scan_tilt_start; // the starting tilt angle "
int16_t scan_pan_end; // the ending pan angle "
int16_t scan_tilt_end; // the ending tilt angle "
uint16_t scan_pan_step; // the panning step size "
uint16_t scan_tilt_step; // the tilting step size "
bool reverse_pan = false; // whether the scanner should currently be panning backwards (scan_pan_end to scan_pan_start)

// variables used for timing, namely to implement non-blocking delays
const uint8_t ms_per_us = 30; // the number of milliseconds to delay per change in Servo.writeMicroseconds() value
uint16_t stored_time = 0; // the start of the delay period
uint16_t delay_time = 0; // the length of the delay

void setup() {
  // call each respective configuration/initialization function for the "modules" we wrote
  configure_serial(9600);
  configure_sensor();
  configure_servos(9,10);
  
  send_message(startup_header, ""); // send a message to the computer on startup
}

void loop() {
  update_rx(); // check for and process incoming instructions from the computer

  if (busy) { // if currently in the process of scanning:
    if (millis() - stored_time >= delay_time) { // only proceed if the most recently set delay has elapsed
      if (tilt_servo.readMicroseconds() <= tilt_home + scan_tilt_end) { // if the scanner hasn't tilted up past the maximum desired angle:
        if (!reverse_pan && (pan_servo.readMicroseconds() >= pan_home + scan_pan_end)) { // if the scanner is panning left and has hit the end of the current scan row
          reverse_pan = true; // set the scanner to pan the other way across the next row
          goto_rel_pos(0,scan_tilt_step); // tilt the scanner up by the desired tilt_step
          delay_time = scan_tilt_step * 2; // set a delay to give the tilt servo time to move to the next row
          stored_time = millis(); // start the delay timer
        } else if (reverse_pan && (pan_servo.readMicroseconds() <= pan_home + scan_pan_start)) { // if the scanner is panning right and has hit the end of the current scan row
          reverse_pan = false; // "
          goto_rel_pos(0,scan_tilt_step); // "
          delay_time = scan_tilt_step * ms_per_us; // "
          stored_time = millis(); // "
        } else { // if the scanner has not yet made it to the end of the current row
          // take a sensor reading and stream it to the computer, along with the current position relative to home
          static char temp_body_buffer[max_tx_body_len + 1];
          memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
          snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("%d,%d,%d"), pan_servo.readMicroseconds() - pan_home, tilt_servo.readMicroseconds() - tilt_home, read_sensor());
          send_message(scan_reading_header, temp_body_buffer);
          
          goto_rel_pos(scan_pan_step * ((!reverse_pan*2)-1), 0); // pan the scanner to the next position in the row
          delay_time = scan_pan_step * ms_per_us; // set a delay to give the pan servo time to move to the next position in the row
          stored_time = millis(); // start the delay timer
        }
      } else { // if the scanner HAS reached the end of the scan
        send_message(scan_stop_header, "");
        gohome();
        reset_scan_params();
        busy = false; // free up the scanner for new instructions
      }
    }
  }
}

void start_scan(uint16_t pan_width, uint16_t tilt_width, uint16_t pan_resolution, uint16_t tilt_resolution) {
  reset_scan_params();
  
  if (!(is_in_pan_range(pan_home - (pan_width/2)) && is_in_pan_range(pan_home + (pan_width/2)))) { // throw an error if the specified panning range goes out of safe bounds for the scanner
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Pan width out of range. pos: %d, home: %d, min: %d, max: %d"), pan_width, pan_home, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!(is_in_tilt_range(tilt_home - (tilt_width/2)) && is_in_tilt_range(tilt_home + (tilt_width/2)))) { // throw an error if the specified tilting range goes out of safe bounds for the scanner
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Tilt width out of range. pos: %d, home: %d, min: %d, max: %d"), tilt_width, tilt_home, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }

  // configure the scan parameters based on what the computer sent
  scan_pan_start = (pan_width/2)*-1;
  scan_tilt_start = (tilt_width/2)*-1;
  scan_pan_end = (pan_width/2);
  scan_tilt_end = (tilt_width/2);
  scan_pan_step = pan_resolution;
  scan_tilt_step = tilt_resolution;

  busy = true; // block the scanner from receiving new instructions / signal that a scan is in progress
  send_message(scan_start_header, "");
  goto_pos(scan_pan_start, scan_tilt_start); // move the scanner to the start position for the scan
  delay_time = 1000; // set a delay to give the servos time to move to the starting position
  stored_time = millis(); // start the delay timer
}

void reset_scan_params() {
  scan_pan_start = scan_tilt_start = scan_pan_end = scan_tilt_end = scan_pan_step = scan_tilt_step = NULL;
  reverse_pan = false;
  stored_time = delay_time = 0;
}
