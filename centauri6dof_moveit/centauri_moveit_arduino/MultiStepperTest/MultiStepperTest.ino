// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

// Joint 1
#define JOINT1_STEP_PIN        48 //CLK+
#define JOINT1_DIR_PIN         46 //CW+

#define JOINT2_STEP_PIN_M1     44
#define JOINT2_DIR_PIN_M1      42

#define JOINT2_STEP_PIN_M2     40
#define JOINT2_DIR_PIN_M2      38

#define JOINT3_STEP_PIN        36
#define JOINT3_DIR_PIN         34

#define JOINT4_STEP_PIN        32
#define JOINT4_DIR_PIN         30

#define JOINT5_STEP_PIN        28
#define JOINT5_DIR_PIN         26

#define JOINT6_STEP_PIN        24
#define JOINT6_DIR_PIN         22


// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper joint1   (1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2_m1(1, JOINT2_STEP_PIN_M1, JOINT2_DIR_PIN_M1);
AccelStepper joint2_m2(1, JOINT2_STEP_PIN_M2, JOINT2_DIR_PIN_M2);
AccelStepper joint3   (1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4   (1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
AccelStepper joint5   (1, JOINT5_STEP_PIN, JOINT5_DIR_PIN);
AccelStepper joint6   (1, JOINT6_STEP_PIN, JOINT6_DIR_PIN);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;


void setup() {

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2_m1.setMaxSpeed(750);
  joint2_m2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);
  joint6.setMaxSpeed(250);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2_m1);
  steppers.addStepper(joint2_m2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);
}

void loop() {
  long positions[7]; // Array of desired stepper positions

  // Back of the envelope calculation for microsteps/revolution, where positions[i] is the number of steps (or microsteps).
    positions[0] = 0; //8000 = 90°
    positions[1] = 0; //4100 = 90°
    positions[2] = 0;  //-4100 = 90°
    positions[3] = 0;     //18000 = 90°
    positions[4] = 0;     //800 = 90°
    positions[5] = 0;     //3600 = 90°
    positions[6] = 0;   //750 = 90°
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1);
}
