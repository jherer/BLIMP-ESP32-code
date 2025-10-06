// At the top we have to include the libraries we want to use. 

#include <ESP32Servo.h>
#include <Ps3Controller.h>

// Here I define the name of my servos, you can name them something else later
Servo lwing;
Servo rwing;
Servo tail;

// Listing what pin on the board they are attached to. Might be different for you
#define lwing_pin 5
#define rwing_pin 13
#define tail_pin  21

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
  rwing.attach(rwing_pin);
  tail.attach(tail_pin);

  // .write is a function from the ESP32Servo library that makes the servo move to some position.
  lwing.write(90);
  rwing.write(90);
  tail.write(90);

  // PASTE YOUR MAC ADDRESS HERE!!!!!
  Ps3.begin("08:b6:1f:7c:65:96"); // In the form "01:a2:b3:04:05:0f"

  Serial.println("Waiting for PS3 Controller...");
}

void loop() {
  if (Ps3.isConnected()) {

    // Defining some variables that are the analog values from the sticks
    int lx = Ps3.data.analog.stick.lx;  
    int ly = Ps3.data.analog.stick.ly;  
    int ry = Ps3.data.analog.stick.ry;  

    // The analog values from the sticks are from -128 to 128
    // Here I am mapping those analog values to a value from 0 to 180
    // 0 to 180 is related to a servo angle position
    
    int tailpos = map(ry, -128, 128, 0, 180);
    tail.write(tailpos);

    // Get the current time for non-blocking movement
    unsigned long currentMillis = millis();

    // Non-blocking flapping logic. Compare current_Millis to last_update
    if (currentMillis - lastUpdate >= flapSpeed) {
      lastUpdate = currentMillis;

      // Update wing angle Going up or down by 2 degrees
      if (increasing) {
        currentAngle += 2;
        if (currentAngle >= maxAngle) {
          increasing = false; // reverse at the top
        }
      } else {
        currentAngle -= 2;
        if (currentAngle <= minAngle) {
          increasing = true; // reverse at the bottom
        }
      }

      // If forward (ly < -deadzone), flap both wings
      // Your signs +-<> may be different depending on controller readings
      if (ly < -deadzone) {
        lwing.write(currentAngle);
        rwing.write(currentAngle);
      } 
      // If left (lx < -deadzone), flap only left wing
      else if (lx < -deadzone) {
        lwing.write(currentAngle);
        rwing.write(90); // Keep right wing neutral
      } 
      // If right (lx > deadzone), flap only right wing
      else if (lx > deadzone) {
        rwing.write(currentAngle);
        lwing.write(90); // Keep left wing neutral
      } 
      // If no input, reset wings to neutral
      else {
        lwing.write(90);
        rwing.write(90);
      }
    }
  }
}
