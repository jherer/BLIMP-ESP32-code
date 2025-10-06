#include "Arduino.h"
#include "ultrasonic_sensor.hpp"

#define trigPin 12
#define echoPin 27

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
float distanceCm;

void startUltrasonic() {
  //Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

/* 
 * Get distance in centimeter
 */
float getDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // if (distanceCm < 1100) {
  //   Serial.println(distanceCm);
  // }
  return distanceCm;
  //delay(500);
}