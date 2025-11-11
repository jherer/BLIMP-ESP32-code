// At the top we have to include the libraries we want to use. 

#include <ESP32Servo.h>
#include <Ps3Controller.h>

// left/right, wing/tail
Servo lwing;
Servo rwing;
Servo ltail;
Servo rtail;

/*

POWER:
- USB on microcontroller
- Servo * 4

GROUND:
- GND on microcontroller
- 4 * Servo

SERVO SIGNAL:
- 22 (SDA)
- 20 (SCL)
- 14
- 32

*/

// Listing what pin on the board they are attached to. Might be different for you
#define lwing_pin 23
#define rwing_pin 22
#define ltail_pin  14
#define rtail_pin  32

const int DEADZONE = 20;  // How much do I have to move the analog sticks to consider it as an input


// MAKE THESE RELATIVE TO LEFT SIDE (0 IS DOWN)
const int MIN_WING_ANGLE = 55;   // Minimum flap angle
const int MAX_WING_ANGLE = 170;  // Maximum flap angle
const int CENTER_WING_ANGLE = 95;

const int MIN_TAIL_ANGLE = 60;
const int MAX_TAIL_ANGLE = 125;
const int CENTER_TAIL_ANGLE = 95;


const int ASSIST_UP = 60;
const int ASSIST_DOWN = 90;

// ALL ACCORDING TO RIGHT SIDE

/* 
 L: 0 is down
  R: 0 is up ****
*/

// Speed control for flapping (milliseconds between each flap)
const unsigned long UPDATE_TIME_MS = 10;
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
  lwing.write(CENTER_WING_ANGLE);
  rwing.write(CENTER_WING_ANGLE);
  ltail.write(CENTER_TAIL_ANGLE);
  rtail.write(CENTER_TAIL_ANGLE);

  // PASTE YOUR MAC ADDRESS HERE!!!!!
  Ps3.begin("08:b6:1f:7c:65:96"); // In the form "01:a2:b3:04:05:0f"
  Serial.println("Waiting for PS3 Controller...");
  
}
/*  ps3.data.button.cross
    ps3.data.button.cross
    ps3.analog.stick.rx
    ps3.analog.button.l2 // trigger
    Ps3.data.button.l2
  */

bool backTurnAssist = false;

void loop() {
  if (Ps3.isConnected()) {
    // Get the current time for non-blocking movement
    unsigned long currentMillis = millis();
    if (Ps3.event.button_down.r1) {
      backTurnAssist = true;
    } else if (Ps3.event.button_down.l1) {
      backTurnAssist = false;
    }

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
        tailAngle = 180 - map(ry, 127, -127, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE);
        flappingTail = false;
      } else {
        tailAngle = CENTER_TAIL_ANGLE;
        flappingTail = true;
      }

      // If forward (ly < -DEADZONE), flap both wings
      // Your signs +-<> may be different depending on controller readings
      if (ly < -DEADZONE) {
        lwing.write(wingAngle);
        rwing.write(180 - wingAngle);
        
        if (flappingTail) {
          ltail.write(180 - map(wingAngle, MIN_WING_ANGLE, MAX_WING_ANGLE, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE));
          rtail.write(map(wingAngle, MIN_WING_ANGLE, MAX_WING_ANGLE, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE));
        }
      } 
      // If right (lx > DEADZONE), flap only left wing
      else if (lx > DEADZONE) {
        lwing.write(wingAngle);
        rwing.write(180 - CENTER_WING_ANGLE); // Keep right wing neutral
        
        if (flappingTail) {
          if (backTurnAssist) {
            ltail.write(180 - ASSIST_UP);
            rtail.write(ASSIST_DOWN);
          } else {
            ltail.write(180 - map(wingAngle, MIN_WING_ANGLE, MAX_WING_ANGLE, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE));
            rtail.write(CENTER_TAIL_ANGLE);
          }
        }
      } 
      // If left (lx < -DEADZONE), flap only right wing
      else if (lx < -DEADZONE) {
        lwing.write(CENTER_WING_ANGLE); // Keep left wing neutral
        rwing.write(180 - wingAngle);

        if (flappingTail) {
          if (backTurnAssist) {
            ltail.write(180 - ASSIST_DOWN);
            rtail.write(ASSIST_UP);
          } else {
            ltail.write(180 - CENTER_TAIL_ANGLE);
            rtail.write(map(wingAngle, MIN_WING_ANGLE, MAX_WING_ANGLE, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE));
          }
        }
      }
      // If no input, reset wings to neutral
      else {
        lwing.write(CENTER_WING_ANGLE);
        rwing.write(CENTER_WING_ANGLE);
        
        if (flappingTail) {
          ltail.write(180 - CENTER_TAIL_ANGLE);
          rtail.write(CENTER_TAIL_ANGLE);
        }
      }
      if (!flappingTail) {
        ltail.write(180 - tailAngle);
        rtail.write(tailAngle);
      }


    }
  }
}
