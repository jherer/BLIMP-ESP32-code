// At the top we have to include the libraries we want to use.

#include <ESP32Servo.h>
#include <Ps3Controller.h>

// left/right, wing/tail
Servo leftFrontWing;
Servo rightFrontWing;
Servo leftTailWing;
Servo rightTailWing;

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

// Servo Signal Pins
#define lwing_pin 23
#define rwing_pin 22
#define ltail_pin 14
#define rtail_pin 32

const int DEADZONE = 20;  // How much do I have to move the analog sticks to consider it as an input


// MAKE THESE RELATIVE TO LEFT SIDE (0 IS DOWN)
const int DOWN_FWING_ANGLE = 55;  // 55 Minimum flap angle, 0 is straight down
const int UP_FWING_ANGLE = 170;   // Maximum flap angle, 180 is straight up
const int CENTER_WING_ANGLE = 95;

const int DOWN_TAIL_ANGLE = 50;  // 0 is straight down
const int UP_TAIL_ANGLE = 130;   //180 is straight up
const int CENTER_TAIL_ANGLE = 95;

const int ASSIST_DOWN = 120;  // 180 is straight down
const int ASSIST_UP = 60;     // 0 is straight up

bool isBackTurnAssistMode = false;

// ALL ACCORDING TO RIGHT SIDE
/* 
  L: 0 is down
  R: 0 is up ****
*/

// Speed control for flapping (milliseconds between each flap)
const unsigned long UPDATE_TIME_MS = 10;
unsigned long lastUpdate = 0;

int wingAngle = 90;            // Current position of wings (setting neutral)
bool isWingIncreasing = true;  // Flap direction (checking if moving up or down at some instance)
const unsigned int WING_SPEED = 2;

int tailsAngle = 90;
bool isTailsFlappingMode = false;
const int TAIL_SPEED = 2;

void updateWingsAngle();
void goForward();
void turnLeft();
void turnRight();
void passiveTurnLeft();
void passiveTurnRight();
void wingsNeutral();

void setup() {
  Serial.begin(115200);

  leftFrontWing.attach(lwing_pin);
  rightFrontWing.attach(rwing_pin);
  leftTailWing.attach(ltail_pin);
  rightTailWing.attach(rtail_pin);

  // .write is a function from the ESP32Servo library that makes the servo move to some position.
  leftFrontWing.write(CENTER_WING_ANGLE);
  rightFrontWing.write(CENTER_WING_ANGLE);
  leftTailWing.write(CENTER_TAIL_ANGLE);
  rightTailWing.write(CENTER_TAIL_ANGLE);

  // PASTE YOUR MAC ADDRESS HERE!!!!!
  Ps3.begin("08:b6:1f:7c:65:96");  // In the form "01:a2:b3:04:05:0f"
  // Serial.println("Waiting for PS3 Controller...");
}
/*  ps3.data.button.cross
    ps3.data.button.cross
    ps3.analog.stick.rx
    ps3.analog.button.l2 // trigger
    Ps3.data.button.l2
  */

void loop() {
  if (Ps3.isConnected()) {
    // Get the current time for non-blocking movement
    unsigned long currentMillis = millis();
    if (Ps3.event.button_down.r1) {
      isBackTurnAssistMode = true;
    } else if (Ps3.event.button_down.l1) {
      isBackTurnAssistMode = false;
    }

    // Non-blocking flapping logic. Compare current_Millis to last_update
    if (currentMillis - lastUpdate >= UPDATE_TIME_MS) {
      lastUpdate = currentMillis;

      // analog stick value
      int leftStickX = Ps3.data.analog.stick.lx;
      int leftStickY = Ps3.data.analog.stick.ly;
      int rightStickX = Ps3.data.analog.stick.rx;
      int rightStickY = Ps3.data.analog.stick.ry;

      // update the wings angle continuously,
      // only when you use the joystick does the wings angle get written to the servo
      updateWingsAngle();

      // If forward (ly < -DEADZONE), flap both wings
      // Your signs +-<> may be different depending on controller readings
      if (leftStickY < -DEADZONE) {
        goForward();
      }
      // If left (lx < -DEADZONE), flap only right wing
      else if (leftStickX < -DEADZONE) {
        turnLeft();
      }
      // If right (lx > DEADZONE), flap only left wing
      else if (leftStickX > DEADZONE) {
        turnRight();
      }
      // If no input, reset wings to neutral
      else {
        wingsNeutral();
      }

      // aileron control. i.e. elevation control
      if (rightStickY > DEADZONE || rightStickY < -DEADZONE) {
        tailsAngle = 180 - map(rightStickY, 127, -127, DOWN_TAIL_ANGLE, UP_TAIL_ANGLE);
        isTailsFlappingMode = false;
      } else {
        tailsAngle = CENTER_TAIL_ANGLE;
        isTailsFlappingMode = true;
      }
      if (!isTailsFlappingMode) {
        leftTailWing.write(180 - tailsAngle);
        rightTailWing.write(tailsAngle);
      }

      // passive turn control
      // if right stick is left
      // if (rightStickX < -DEADZONE) {
      //   passiveTurnLeft();
      // } else if (rightStickX > DEADZONE) {
      //   passiveTurnRight();
      // }
    }
  }
}



void updateWingsAngle() {
  // Update wing angle going up or down by WING_SPEED degrees
  if (isWingIncreasing) {
    wingAngle += WING_SPEED;
    if (wingAngle >= UP_FWING_ANGLE) {
      isWingIncreasing = false;  // reverse at the top
    }
  } else {
    wingAngle -= WING_SPEED;
    if (wingAngle <= DOWN_FWING_ANGLE) {
      isWingIncreasing = true;  // reverse at the bottom
    }
  }
}

void goForward() {
  leftFrontWing.write(wingAngle);
  rightFrontWing.write(180 - wingAngle);

  if (isTailsFlappingMode) {
    leftTailWing.write(180 - map(wingAngle, DOWN_FWING_ANGLE, UP_FWING_ANGLE, DOWN_TAIL_ANGLE, UP_TAIL_ANGLE));
    rightTailWing.write(map(wingAngle, DOWN_FWING_ANGLE, UP_FWING_ANGLE, DOWN_TAIL_ANGLE, UP_TAIL_ANGLE));
  }
}

void turnLeft() {
  leftFrontWing.write(CENTER_WING_ANGLE);  // Keep left wing neutral
  rightFrontWing.write(180 - wingAngle);

  if (isTailsFlappingMode) {
    if (isBackTurnAssistMode) {
      leftTailWing.write(180 - ASSIST_UP);
      rightTailWing.write(ASSIST_DOWN);
    } else {
      leftTailWing.write(180 - CENTER_TAIL_ANGLE);
      rightTailWing.write(map(wingAngle, DOWN_FWING_ANGLE, UP_FWING_ANGLE, DOWN_TAIL_ANGLE, UP_TAIL_ANGLE));
    }
  }
}

void turnRight() {
  leftFrontWing.write(wingAngle);
  rightFrontWing.write(180 - CENTER_WING_ANGLE);  // Keep right wing neutral

  if (isTailsFlappingMode) {
    if (isBackTurnAssistMode) {
      leftTailWing.write(180 - ASSIST_DOWN);
      rightTailWing.write(ASSIST_UP);
    } else {
      leftTailWing.write(180 - map(wingAngle, DOWN_FWING_ANGLE, UP_FWING_ANGLE, DOWN_TAIL_ANGLE, UP_TAIL_ANGLE));
      rightTailWing.write(CENTER_TAIL_ANGLE);
    }
  }
}

void passiveTurnLeft() {
  leftTailWing.write(180 - ASSIST_UP);
  rightTailWing.write(ASSIST_DOWN);
}

void passiveTurnRight() {
  leftTailWing.write(180 - ASSIST_DOWN);
  rightTailWing.write(ASSIST_UP);
}

void wingsNeutral() {
  leftFrontWing.write(CENTER_WING_ANGLE);
  rightFrontWing.write(CENTER_WING_ANGLE);

  if (isTailsFlappingMode) {
    leftTailWing.write(180 - CENTER_TAIL_ANGLE);
    rightTailWing.write(CENTER_TAIL_ANGLE);
  }
}
