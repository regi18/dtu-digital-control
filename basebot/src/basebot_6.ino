/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
* 
* The MIT License (MIT)  https://mit-license.org/
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */


#include <stdbool.h>
#include <stdio.h>
#include <math.h>
// Support functions
#include "lib/udisplay.h"
#include "lib/urobot.h"
#include "lib/uusb.h"
#include "lib/umotor.h"
#include "lib/uimu2.h"
#include "lib/uencoder.h"
#include "utils/pi_controller.h"
#include "utils/state_machine.h"
#include "utils/parameters.h"
#include "utils/smoother.h"
#include "utils/pose_tracker.h"
#include "utils/logger.h"
// #include "only_controller.h"


// ////////////////////////////////////////


void setup()
{ 
  Serial.begin(12000000);

  // Initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  imu2.setup(); // gyro and accelerometer - include tilt angle

  // only_controller_initialize();
}


// --------------------------------------------------


double getTurnrateComand(
  PIController *headingController, 
  PIController *turnrateController, 
  Smoother *s, 
  PoseTracker *pose, 
  StateMachine *sm, 
  Logger *logger
)
{
  if (sm->getIsUsingHeadingController()) {
    double headingComand = headingController->compute(sm->getHeadingReference(), pose->getHeading());
    sm->setTurnrateReference(headingComand);
  }

  // Turnrate limiter
  double turnrateRef = s->compute(sm->getTurnrateReference());
  return turnrateController->compute(turnrateRef, pose->getTurnrate());
}

/**
 * Make the control for the two motors
 */
void motorControlUpdate(
  StateMachine *sm, 
  PIController *pi0, 
  PIController *pi1, 
  double velocityReferenceLeft, 
  double velocityReferenceRight
)
{ 
  // running a sequence, so do control
  float ref_left = velocityReferenceLeft * GEAR/WHEEL_RADIUS;
  float ref_right = velocityReferenceRight * GEAR/WHEEL_RADIUS;

  // Left motor
  motor.motorVoltage[0] = -pi0->compute(ref_left, -encoder.motorVelocity[0]);

  // Right motor
  motor.motorVoltage[1] = pi1->compute(ref_right, encoder.motorVelocity[1]);
}


// --------------------------------------------------

PoseTracker poseTracker;
StateMachine sm(&poseTracker);
Logger logger(&sm, &poseTracker);

void printlog() {
  logger.printLog();
}



/**
* Main loop
*/
void loop(void)
{ 
  // init sample time
  uint32_t nextSample = 0;

  PIController pi0(KP, KI, true);
  PIController pi1(KP, KI, true);
  PIController headingController(KP_HEADING, KI_HEADING, false);
  PIController turnrateController(KP_TURNRATE, KI_TURNRATE, false);
  
  Smoother velSmoother(LIN_ACC_LIMIT);
  Smoother turnSmoother(ANG_ACC_LIMIT);

  poseTracker.reset();

  while (true)
  { 
    // main loop
    // get time since start in microseconds
    uint32_t us = micros();

    // loop until time for next sample
    if (us > nextSample) // start of new control cycle
    { 
      // advance time for next sample
      nextSample += SAMPLE_TIME_US;

      // give value to actuators
      motor.tick();

      // support functions
      robot.tick(); // measure battery voltage etc.
      display.tick(); // update O-LED display

      // Handle state machine
      bool res = sm.tick();
      // Exit to main loop if test sequence is done (resets controllers)
      if (res) break;

      if (sm.getState() > 0) {
        // read sensors
        imu2.tick();
        encoder.tick();

        poseTracker.update();

        // Control turnrate, heading and set acceleration limits
        double turnrateComand = getTurnrateComand(
          &headingController,
          &turnrateController,
          &turnSmoother,
          &poseTracker,
          &sm,
          &logger
        );

        double velRef = velSmoother.compute(sm.getVelocityReference());
        double leftRef = velRef - turnrateComand;
        double rightRef = velRef + turnrateComand;

        // make control actions
        motorControlUpdate(&sm, &pi0, &pi1, leftRef, rightRef);

        logger.log();
      }
    }

    usb.tick(); // listen to incoming from USB
  }
}


/////////////////////////////////////////////////////////////////

