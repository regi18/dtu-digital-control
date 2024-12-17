#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <iostream>
#include "parameters.h"


/**
 * Implements G(s) = 100 / (s + 100) with a saturator
 */
class Smoother {
private:
    float const POLE = 100;
    float limit;
    float integral;
    float prevIntegralInput;
    float prevOutput;

public:
    // Constructor
    Smoother(float limit): limit(limit), integral(0), prevIntegralInput(0), prevOutput(0) {}

    float integrate(float input) {
        integral += 0.5 * SAMPLE_TIME * (input + prevIntegralInput);
        prevIntegralInput = input;
        return integral;
    }

    // Method to calculate control output
    float compute(float input) {
        float error = (input - prevOutput) * POLE;

        // Saturate the output
        if (error > limit) {
            error = limit;
        }
        else if (error < -limit) {
            error = -limit;
        }

        float output = integrate(error);

        // Update the previous output for the next iteration
        prevOutput = output;

        return output;
    }
};


#endif // SMOOTHER_H
