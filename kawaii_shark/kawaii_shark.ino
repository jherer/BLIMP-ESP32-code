// At the top we have to include the libraries we want to use. 

#include <ESP32Servo.h>
#include <Ps3Controller.h>

// left/right, wing/tail
Servo lwing;
Servo rwing;
Servo ltail;
Servo rtail;

// Listing what pin on the board they are attached to. Might be different for you
#define lwing_pin 22
#define rwing_pin 23
#define ltail_pin  14
#define rtail_pin  32

const int DEADZONE = 20;  // How much do I have to move the analog sticks to consider it as an input

const int MIN_WING_ANGLE = 0;   // Minimum flap angle
const int MAX_WING_ANGLE = 180;  // Maximum flap angle

const int MAX_TAIL_ANGLE = 180;
const int MIN_TAIL_ANGLE = 0;
const int DEFAULT_TAIL_ANGLE = 90;

// Speed control for flapping (milliseconds between each flap)
const unsigned long UPDATE_TIME_MS = 20;
unsigned long lastUpdate = 0;

int wingAngle = 90;   // Current position of wings (setting neutral)
bool wingIncreasing = true;    // Flap direction (checking if moving up or down at some instance)
const unsigned int WING_SPEED = 2;

int tailAngle = 90;
bool flappingTail = false;
const int TAIL_SPEED = 2;


void setup() {
  Serial.begin(115200);

  lwing.attach(lwing_pin);
  rwing.attach(rwing_pin);
  ltail.attach(ltail_pin);
  rtail.attach(rtail_pin);

  // .write is a function from the ESP32Servo library that makes the servo move to some position.
  lwing.write(90);
  rwing.write(90);
  ltail.write(90);
  rtail.write(90);

  // PASTE YOUR MAC ADDRESS HERE!!!!!
  Ps3.begin("08:b6:1f:7c:65:96"); // In the form "01:a2:b3:04:05:0f"
  Serial.println("Waiting for PS3 Controller...");
}
/*  ps3.data.button.cross
    ps3.data.button.cross
    ps3.analog.stick.rx
    ps3.analog.button.l2 // trigger  */

void loop() {
  if (Ps3.isConnected()) {

    // Get the current time for non-blocking movement
    unsigned long currentMillis = millis();

    // Non-blocking flapping logic. Compare current_Millis to last_update
    if (currentMillis - lastUpdate >= UPDATE_TIME_MS) {
      lastUpdate = currentMillis;

            // Defining some variables that are the analog values from the sticks
            int lx = Ps3.data.analog.stick.lx;  
            int rx = Ps3.data.analog.stick.rx;  
            int ly = Ps3.data.analog.stick.ly;  
            int ry = Ps3.data.analog.stick.ry; 

            Serial.print("lx:");
            Serial.print(lx);
            Serial.print(",");
            Serial.print("ly:");
            Serial.print(ly);
            Serial.print(",");
            Serial.print("rx:");
            Serial.print(rx);
            Serial.print(",");
            Serial.print("ry:");
            Serial.println(ry);
            // Left and Top is negative


      // Update wing angle Going up or down by 2 degrees
      if (wingIncreasing) {
        wingAngle += 2;
        if (wingAngle >= MAX_WING_ANGLE) {
          wingIncreasing = false; // reverse at the top
        }
      } else {
        wingAngle -= 2;
        if (wingAngle <= MIN_WING_ANGLE) {
          wingIncreasing = true; // reverse at the bottom
        }
      }

      if (ry > DEADZONE || ry < -DEADZONE) {
        tailAngle = map(ry, 127, -127, 0, 180);
        flappingTail = false;
      } else {
        tailAngle = 90;
        flappingTail = true;
      }


      // If forward (ly < -DEADZONE), flap both wings
      // Your signs +-<> may be different depending on controller readings
      if (ly < -DEADZONE) {
        lwing.write(wingAngle);
        rwing.write(wingAngle);
        
        if (flappingTail) {
          ltail.write(wingAngle);
          rtail.write(wingAngle);
        }
      } 
      // If left (lx < -DEADZONE), flap only left wing
      else if (lx < -DEADZONE) {
        lwing.write(wingAngle);
        rwing.write(90); // Keep right wing neutral
        
        if (flappingTail) {
          ltail.write(wingAngle);
          rtail.write(90);
        }
      } 
      // If right (lx > DEADZONE), flap only right wing
      else if (lx > DEADZONE) {
        lwing.write(90); // Keep left wing neutral
        rwing.write(wingAngle);

        if (flappingTail) {
          ltail.write(90);
          rtail.write(wingAngle);
        }
      } 
      // If no input, reset wings to neutral
      else {
        lwing.write(90);
        rwing.write(90);
        
        if (flappingTail) {
          ltail.write(90);
          rtail.write(90);
        }
      }
      if (!flappingTail) {
        ltail.write(tailAngle);
        rtail.write(tailAngle);
      }
    
      /*if (ry > DEADZONE) {
        tailAngle += TAIL_SPEED;
      } else if (ry < -DEADZONE) {
        tailAngle -= TAIL_SPEED;
      }
      tailAngle = max(tailAngle, 0); // Keep within 0
      tailAngle = min(tailAngle, 180); // to 180 degrees
      */

    }
  }
}
