int16_t scan_pan_start;
int16_t scan_tilt_start;
int16_t scan_pan_end;
int16_t scan_tilt_end;
uint16_t scan_pan_step;
uint16_t scan_tilt_step;

uint16_t stored_time = 0;
uint16_t delay_time = 0;

bool reverse_pan = false;

void setup() {
  configure_serial(9600);
  configure_sensor();
  configure_servos(9,10);
  
  send_message(startup_header, "");
}

void loop() {
  update_rx();

  if (busy) {
    if (millis() - stored_time >= delay_time) {
      if (tilt_servo.readMicroseconds() <= tilt_home + scan_tilt_end) {
        if (!reverse_pan && (pan_servo.readMicroseconds() >= pan_home + scan_pan_end)) {
          reverse_pan = true;
          goto_rel_pos(0,scan_tilt_step);
          delay_time = scan_tilt_step * 2;
          stored_time = millis();
        } else if (reverse_pan && (pan_servo.readMicroseconds() <= pan_home + scan_pan_start)) {
          reverse_pan = false;
          goto_rel_pos(0,scan_tilt_step);
          delay_time = scan_tilt_step * 2;
          stored_time = millis();
        } else {
          static char temp_body_buffer[max_tx_body_len + 1];
          memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
          snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("%d,%d,%d"), pan_servo.readMicroseconds(), tilt_servo.readMicroseconds(), read_sensor());
          send_message(scan_reading_header, temp_body_buffer);

          goto_rel_pos(scan_pan_step * ((!reverse_pan*2)-1), 0);
          delay_time = scan_pan_step * 2;
          stored_time = millis();
        }
      } else {
        send_message(scan_stop_header, "");
        gohome();
        reset_scan_params();
        busy = false;
      }
    }
  }
}

void start_scan(uint16_t pan_width, uint16_t tilt_width, uint16_t pan_resolution, uint16_t tilt_resolution) {
  reset_scan_params();
  
  if (!(is_in_pan_range(pan_home - (pan_width/2)) && is_in_pan_range(pan_home + (pan_width/2)))) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Pan width out of range. pos: %d, home: %d, min: %d, max: %d"), pan_width, pan_home, pan_min, pan_max);
    send_message(error_header, temp_body_buffer);
    return;
  }
  if (!(is_in_tilt_range(tilt_home - (tilt_width/2)) && is_in_tilt_range(tilt_home + (tilt_width/2)))) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Tilt width out of range. pos: %d, home: %d, min: %d, max: %d"), tilt_width, tilt_home, tilt_min, tilt_max);
    send_message(error_header, temp_body_buffer);
    return;
  }

  scan_pan_start = (pan_width/2)*-1;
  scan_tilt_start = (tilt_width/2)*-1;
  scan_pan_end = (pan_width/2);
  scan_tilt_end = (tilt_width/2);
  scan_pan_step = pan_resolution;
  scan_tilt_step = tilt_resolution;

  busy = true;
  send_message(scan_start_header, "");
  goto_pos(scan_pan_start, scan_tilt_start);
  stored_time = millis();
  delay_time = 1000;
}

void reset_scan_params() {
  scan_pan_start = scan_tilt_start = scan_pan_end = scan_tilt_end = scan_pan_step = scan_tilt_step = NULL;
  reverse_pan = false;
  stored_time = delay_time = 0;
}
