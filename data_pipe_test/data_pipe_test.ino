#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>

// declare arduino I/O pins
// sensor -> Analog 0, servos -> Digital 9, 10
const int sensor_pin = A0;
const int pan_pin = 9;
const int tilt_pin = 10;

// declare servos
Servo pan_servo;
Servo tilt_servo;

// declare pan and tilt width and resolution (us)
const int pan_width = 800; // max width -> 960
const int tilt_width = 600; // max width -> 840
const int pan_res = 10;
const int tilt_res = 10;

// declare 0 deg/center positions for pan and tilt servos (found manually)
// default neutral pos is 1502 us (93deg)
const int pan_0 = 1420;
const int tilt_0 = 1180;

// declare variables to track pan and tilt offset from center
int d_pan, d_tilt;

// declare boolean to track pan direction
bool pan_right = true;


int read_sensor() {
  // use signal from distance sensor to adjust blink speed of LEDs
  int signal = analogRead(sensor_pin); // signal value 0-1023 corresponding to signal voltage 0-5V

  // Serial.println(signal);

  return signal;
}

void move(){
    if(pan_right){
        if(d_pan < pan_width/2){
            d_pan += pan_res;
        } else {
            d_tilt += tilt_res;
            pan_right = false;
        }
    } else {
        if(d_pan > -pan_width/2){
            d_pan -= pan_res;
        } else {
            d_tilt += tilt_res;
            pan_right = true;
        }
    }

    pan_servo.writeMicroseconds(pan_0 + d_pan);
    tilt_servo.writeMicroseconds(tilt_0 + d_tilt);
    delay(25);
}

void setup() {
    // initialize button and sensor pins as INPUT
    pinMode(sensor_pin, INPUT);

    pan_servo.attach(pan_pin);
    tilt_servo.attach(tilt_pin);

    d_pan = -pan_width/2;
    d_tilt = -tilt_width/2;

    pan_servo.writeMicroseconds(pan_0 + d_pan);
    tilt_servo.writeMicroseconds(tilt_0 + d_tilt);

    delay(100);

    Serial.begin(9600);

    delay(1000);
}

void loop() {
    if(d_tilt < tilt_width/2){
        move();
        Serial.println(d_pan);
        Serial.println(d_tilt);
        Serial.println(read_sensor());
    } else {
        d_pan = -pan_width/2;
        d_tilt = -tilt_width/2;
    }
}
