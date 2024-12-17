#ifndef POSE_TRACKER_H
#define POSE_TRACKER_H

#include <iostream>
#include "parameters.h"
#include "../lib/uencoder.h"
#include "../lib/uimu2.h"
#include "smoother.h"


class PoseTracker {
private:
    double x;
    double y;
    double heading;  // radians
    double traveledDistance;
    double distancePerTick;
    int32_t currentEncoder[2];
    int32_t previousEncoder[2];

public:
    // Constructors
    PoseTracker(double x0, double y0, double theta0) : x(x0), y(y0), heading(0), traveledDistance(0) {
        previousEncoder[0] = 0;
        previousEncoder[1] = 0;
        currentEncoder[0] = 0;
        currentEncoder[1] = 0;

        // 12 magnets, 2 sensors, each 2 flanks
        distancePerTick = WHEEL_RADIUS / GEAR / 12.0 / 4.0*2.0*M_PI;
    }
    PoseTracker() : PoseTracker(0, 0, 0) {}

    void reset(double x0, double y0, double theta0) {
        previousEncoder[0] = 0;
        previousEncoder[1] = 0;
        currentEncoder[0] = 0;
        currentEncoder[1] = 0;

        x = x0;
        y = y0;
        heading = theta0;
        traveledDistance = 0;
    }
    void reset() { 
        reset(0, 0, 0);
    }

    // Update values
    void update() {
        currentEncoder[0] = encoder.encoder[0];
        currentEncoder[1] = encoder.encoder[1];

        double leftDistance = -double(currentEncoder[0] - previousEncoder[0]) * distancePerTick;
        double rightDistance = double(currentEncoder[1] - previousEncoder[1]) * distancePerTick;
        traveledDistance += (leftDistance + rightDistance) / 2.0;

        heading += (rightDistance - leftDistance) / WHEEL_DISTANCE;

        double vel = (leftDistance + rightDistance) / 2.0;
        x += vel * cos(heading);
        y += vel * sin(heading);

        previousEncoder[0] = currentEncoder[0];
        previousEncoder[1] = currentEncoder[1];
    }

    double getTurnrate() {
        double leftVel = -encoder.motorVelocity[0];
        double rightVel = encoder.motorVelocity[1];

        return (rightVel - leftVel) * (WHEEL_RADIUS/GEAR) / WHEEL_DISTANCE;

        // return imu2.gyro[2] * M_PI / 180.0;
    }

    double getX() { return x; }
    double getY() { return y; }
    double getHeading() { return heading; }
    double getTraveledDistance() { return traveledDistance; }
    double getVelocity() { return (-encoder.motorVelocity[0] + encoder.motorVelocity[1]) * (WHEEL_RADIUS/GEAR)/2; }
};


#endif // POSE_TRACKER_H