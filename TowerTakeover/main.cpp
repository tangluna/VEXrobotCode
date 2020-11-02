/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Created:      Fri Jan 24 2020                                           */
/*    Description:  6374A Tower Takeover                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 2            
// Controller1          controller                    
// ARM_LEFT             motor         3               
// ARM_RIGHT            motor         4               
// INTAKE_LEFT          motor         5               
// INTAKE_RIGHT         motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "cmath"
#include "vex.h"

using namespace vex;

void userInterface();
void haltArm();
void haltIntake();
void calibrate();

competition Competition;

int ARM_SPEED = 40;
int ARM_THRESH = 3;
int ARM_STOP = 615;
int ARM_LOW = 10;
int ARM_TRIM = 3; // applied to left arm
bool ARM_STOPPING = false;
float ARM_CORRECT = 0.1;
bool ARM_MAX = false;
bool ARM_MIN = false;

int INTAKE_SPEED = 100;
bool INTAKE_IN = false;

motor lift = motor(PORT7);

void userInterface() {
  Controller1.Screen.clearScreen();
  
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Current: %.2f", Brain.Battery.current());

  if (ARM_MAX) {
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("ARM HIGH");
  }

  if (ARM_MIN) {
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("ARM LOW");
  }
}

void autonomous(void) {
  // auton pass
  motor left = motor(PORT1, ratio18_1, false);
  motor right = motor(PORT2, ratio18_1, true);
  left.spin(forward, -50, percent);
  right.spin(forward, -50, percent);
  wait(4500, msec);
  left.spin(forward, 50, percent);
  right.spin(forward, 50, percent);
  wait(3000, msec);
  left.stop();
  right.stop();
}

void trimUp() { ARM_TRIM += 5; }
void trimDown() { ARM_TRIM -= 5; }

void usercontrol(void) {

  float Wheel = 10.16; 
  float WB = 36.75; 
  float EncPerCm = 360.0 / (Wheel* M_PI); 
  float totalEnc = -27* EncPerCm;

  lift.resetPosition();
	lift.resetRotation();


  bool arm_up = false;
  bool arm_down = false;
  while (true) {
          if (Controller1.ButtonA.pressing())
    {
        lift.rotateTo(totalEnc, deg, 20.0, velocityUnits::pct, false);
    }

    if (Controller1.ButtonB.pressing())
    {
      lift.spin(forward, 30, percent);
    }
    
    if (Controller1.ButtonY.pressing())
    {
      lift.stop();
    }

    if (Controller1.ButtonR1.pressing() && !ARM_MAX) {
      ARM_MIN = false;
      ARM_STOPPING = false; arm_up = true; arm_down = false;
      ARM_LEFT.spin(directionType::fwd, ARM_SPEED, pct);
      ARM_RIGHT.spin(directionType::fwd, ARM_SPEED, pct);
    } else if (Controller1.ButtonR2.pressing() && !ARM_MIN) {
      ARM_MAX = false; arm_down = true;
      ARM_STOPPING = false; arm_up = false;
      ARM_LEFT.spin(directionType::rev, ARM_SPEED, pct);
      ARM_RIGHT.spin(directionType::rev, ARM_SPEED, pct);
    } else {
      arm_up = false; arm_down = false;
      float angle_l = ARM_LEFT.position(deg) - ARM_TRIM;
      float angle_r = ARM_RIGHT.position(deg);
      float diff = angle_l - angle_r;
      float correct = ARM_CORRECT * std::abs(diff);

      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print(angle_l);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print(angle_r);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print(diff);

      if (diff < -ARM_THRESH) {
        ARM_LEFT.spin(directionType::fwd, correct, pct);
        ARM_RIGHT.spin(directionType::rev, correct, pct);
      } else if (diff > ARM_THRESH) {
        ARM_LEFT.spin(directionType::rev, correct, pct);
        ARM_RIGHT.spin(directionType::fwd, correct, pct);
      } else {
        ARM_STOPPING = true;
        directionType dir = ARM_LEFT.direction();
        ARM_LEFT.spin(dir, ARM_SPEED * 0.2, pct);
        ARM_RIGHT.spin(dir, ARM_SPEED * 0.2, pct);
        Brain.Timer.event(haltArm, 200);
      }
    }

    // subtract 150 to account for stopping distance
    if (ARM_LEFT.position(deg) > ARM_STOP - 150 && arm_up) { ARM_MAX = true; }
    if (ARM_RIGHT.position(deg) > ARM_STOP - 150 && arm_up) { ARM_MAX = true; }

    // subtract 150 to account for stopping distance
    if (ARM_LEFT.position(deg) < ARM_LOW + 150 && arm_down) { ARM_MIN = true; }
    if (ARM_RIGHT.position(deg) < ARM_LOW + 150 && arm_down) { ARM_MIN = true; }

    if (Controller1.ButtonL1.pressing()) {
      INTAKE_IN = true;
      INTAKE_LEFT.spin(directionType::fwd, INTAKE_SPEED, pct);
      INTAKE_RIGHT.spin(directionType::fwd, INTAKE_SPEED, pct);
    }

    if (Controller1.ButtonL2.pressing() && !INTAKE_IN) {
      INTAKE_IN = true;
      INTAKE_LEFT.spin(directionType::rev, INTAKE_SPEED, pct);
      INTAKE_RIGHT.spin(directionType::rev, INTAKE_SPEED, pct);
    }

    userInterface();
    wait(20, msec);
  }
}

void haltArm() {
  // arm stopping will be false if motion has been requested
  // between controller btn release and callback set
  if (ARM_STOPPING) {
    ARM_LEFT.stop();
    ARM_RIGHT.stop();
    ARM_LEFT.setBrake(hold);
    ARM_RIGHT.setBrake(hold);
  }
}

void haltIntake() {
  // only halt intake if grabbing
  if (INTAKE_IN) {
    INTAKE_IN = false;
    INTAKE_LEFT.stop();
    INTAKE_RIGHT.stop();
  }
}

void calibrate() {
  ARM_LEFT.setBrake(hold);
  ARM_RIGHT.setBrake(hold);

  ARM_LEFT.resetPosition();
  ARM_RIGHT.resetPosition();

}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Controller1.ButtonX.pressed(calibrate);
  Controller1.ButtonL2.released(haltIntake);
  Controller1.ButtonUp.pressed(trimUp);
  Controller1.ButtonDown.pressed(trimDown);

  calibrate();

  vexcodeInit();

  while (true) { wait(100, msec); }
}
