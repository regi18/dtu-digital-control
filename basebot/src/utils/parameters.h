#ifndef PARAMETERS_H
#define PARAMETERS_H


#define SAMPLE_TIME_US 4000 // sample time for the whole system
#define SAMPLE_TIME (SAMPLE_TIME_US / 1e6)

#define GEAR 9.6
#define WHEEL_DISTANCE 0.147
#define WHEEL_RADIUS 0.03

#define MAX_VOLTAGE 9.0
#define LIN_ACC_LIMIT 4.0
#define ANG_ACC_LIMIT 3.0

// PI Controller parameters for the motors
#define KP 0.0095439
#define KI 0.1276496625

#define KP_HEADING 2.542915352424283
#define KI_HEADING 1.633920868751176

#define KP_TURNRATE 0.142516073155422
#define KI_TURNRATE 1.221810656830948

// #define KP_TURNRATE 0.01
// #define KI_TURNRATE 100

#define VEL_REF 0.5         // m/s
#define START_REF_TIME 50   // ms


#endif // PARAMETERS_H