#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <iostream>
#include "parameters.h"


class PIController {
private:
    double Kp;     // Proportional gain
    double Ki;     // Integral gain
    double integral; // Accumulated integral term
    double prevError; // Previous error for integration
    bool isSaturation;

public:
    // Constructor
    PIController(double Kp, double Ki, bool saturate) : Kp(Kp), Ki(Ki), integral(0.0), prevError(0.0), isSaturation(saturate) {}

    // Method to calculate control output
    double compute(double reference, double measurement) {
        // Calculate error
        double error = reference - measurement;

        // Update integral term (trapezoidal rule)
        integral += 0.5 * SAMPLE_TIME * (error + prevError);

        // Calculate control output
        double output = Kp * error + Ki * integral;

        // Update previous error
        prevError = error;

        // Saturation
        if (isSaturation) {
            if (output > MAX_VOLTAGE) {
                output = MAX_VOLTAGE;
            }
            else if (output < -MAX_VOLTAGE) {
                output = -MAX_VOLTAGE;
            }
        }

        return output;
    }

    // Method to reset the controller
    void reset() {
        integral = 0.0;
        prevError = 0.0;
    }
};


#endif // PI_CONTROLLER_H
