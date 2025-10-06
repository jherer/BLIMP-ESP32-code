// At the top we have to include the libraries we want to use. 
#include <ESP32Servo.h>
#include <Ps3Controller.h>


#include "ultrasonic_sensor.hpp"

// Here I define the name of my servos, you can name them something else later
Servo lwing;
//Servo rwing;
//Servo tail;

// Listing what pin on the board they are attached to. Might be different for you
#define lwing_pin 22
//#define rwing_pin 13
//#define tail_pin  21

const int deadzone = 20;  // How much do I have to move the analog sticks to consider it as an input
const int minAngle = 0;   // Minimum flap angle
const int maxAngle = 180;  // Maximum flap angle
int currentAngle = 90;     // Current position of wings (setting neutral)
bool increasing = true;    // Flap direction (checking if moving up or down at some instance)

unsigned long lastUpdate = 0;
const unsigned long flapSpeed = 5; // Speed control for flapping (milliseconds between each flap)

void setup() {
  Serial.begin(115200);

  lwing.attach(lwing_pin);
  //rwing.attach(rwing_pin);
  //tail.attach(tail_pin);

  // .write is a function from the ESP32Servo library that makes the servo move to some position.
  lwing.write(90);
  //rwing.write(90);
  //tail.write(90);

  // PASTE YOUR MAC ADDRESS HERE!!!!!
  //Ps3.begin("08:b6:1f:7c:65:96"); // In the form "01:a2:b3:04:05:0f"

  //Serial.println("Waiting for PS3 Controller...");
  startUltrasonic();
}

void loop() {
  int distance = (int)getDistance();
  Serial.println(distance);
  lwing.write(map(distance, 0, 60, 0, 180));
  // lwing.write(0);
  delay(50);
  // lwing.write(180);
  // delay(500);
}
