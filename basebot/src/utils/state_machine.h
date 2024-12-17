#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


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
#include "utils/parameters.h"
#include "pose_tracker.h"


// meters, velocity, useHeadingController, heading/turnrate

// Speed mode (NOT stopping for turns)
#define VEL_REF_SPEED 0.5
const double path[][4] = {
    {0.6, VEL_REF_SPEED, 1, 0},
    {0.3, VEL_REF_SPEED, 1, -30},
    {1.6, VEL_REF_SPEED, 0, VEL_REF_SPEED / 0.45},  // omega = v/R
    {0.5, VEL_REF_SPEED, 0, -VEL_REF_SPEED / 1},
    {1.4, VEL_REF_SPEED, 0, 0},
};

// Precise mode (stopping for turns)
// const double path[][4] = {
//     {0.8, VEL_REF, 1, 0},
//     {1.4, 0, 1, -40},
//     {3.87, VEL_REF, 0, VEL_REF / 0.5},  // omega = v/R
//     {4.8, 0, 1, 193},
//     {6.2, VEL_REF, 0, 0},
// };



class StateMachine {

private:
    PoseTracker *pose;
    int state = 0;                 // state machine current state
    uint32_t startTime = 0;        // time, when 'start' was pressed (ms)
    double desiredVelocity = 0;    // desired (reference) value send to controller (m/s)
    double desiredHeading = 0;     // desired (reference) value send to controller (radians)
    double desiredTurnrate = 0;    // desired (reference) value send to controller (radians/s)
    bool isUsingHeadingController = true;

    /**
     * Commands the robot based on the desired turnrate
     */
    void chooseTurnrate(double turnrate) {
        desiredTurnrate = turnrate;
        isUsingHeadingController = false;
    }
    /**
     * Commands the robot based on the desired heading (degrees)
     */
    void chooseHeading(double heading) {
        desiredHeading = heading * M_PI/180;
        isUsingHeadingController = true;
    }


    /**
     * Execute the defined path sequence
     */
    void pathSequence() {
        double pathTotDist = 0;
        for (size_t i = 0; i < sizeof(path) / sizeof(path[0]); ++i) {

            pathTotDist += path[i][0];
            if (pose->getTraveledDistance() < pathTotDist) {
                desiredVelocity = path[i][1];

                if (path[i][2] == 1) {
                    chooseHeading(path[i][3]);
                } 
                else {
                    chooseTurnrate(path[i][3]);
                }
                return;
            }
        }

        // End of path sequence
        state = 99;
    }


public:
    StateMachine(PoseTracker *pose): pose(pose) {}

    /**
     * Returns true if sequence is finished, false otherwise
     */
    bool tick()
    { 
        // This function is called at every sample time and should never wait in a loop.
        // Update variables as needed and return.
        bool button = false;

        // This is a state machine. State 0: wait for start button press. Other states are part of a sequence
        switch (state)
        { 
            // ------------------------------------------------------------
            // Start mission, initial value
            case 0:
                button = digitalReadFast(PIN_START_BUTTON);

                if (button or robot.missionStart)
                { 
                    // starting a sequence
                    desiredVelocity = 0;
                    chooseHeading(0);
                    startTime = millis();

                    // Update of display takes too long time for fast sampling so, disable during sequence.
                    display.useDisplay = false;

                    // go to next state
                    state = 2;
                }
                break;


            // ------------------------------------------------------------
            // Handles path
            case 2:
                if (millis() - startTime > START_REF_TIME)
                { 
                    desiredVelocity = VEL_REF;
                    pathSequence();
                }
                break;


            // ------------------------------------------------------------
            // Mission finished, stop and go back to initial state
            default:
                // back to start
                state = 0;
                desiredVelocity = 0;
                desiredHeading = 0;
                desiredTurnrate = 0;
                isUsingHeadingController = false;
                motor.motorVoltage[0] = 0; // left motor
                motor.motorVoltage[1] = 0;  // right motor
                display.useDisplay = true;

                // Print some results
                Serial.print("% Set Sample time ");
                Serial.print(SAMPLE_TIME_US);
                Serial.print(" usec. Measured Sample time ");
                Serial.print(encoder.sampleTime_us);
                Serial.print(" usec.");
                Serial.println();

                return true;
                break;
        }
    
        return false;
    }


    // Getters
    int getState() { return state; }
    uint32_t getStartTime() { return startTime; }
    double getVelocityReference() { return desiredVelocity; }
    double getHeadingReference() { return desiredHeading; }
    double getTurnrateReference() { return desiredTurnrate; }
    bool getIsUsingHeadingController() { return isUsingHeadingController; }

    // Setters
    void setTurnrateReference(double turnrate) { desiredTurnrate = turnrate; }
};

#endif // STATE_MACHINE_H