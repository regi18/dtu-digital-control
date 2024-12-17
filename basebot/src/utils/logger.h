#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include "lib/udisplay.h"
#include "lib/urobot.h"
#include "lib/uusb.h"
#include "state_machine.h"
#include "pose_tracker.h"

struct LogEntry
{
    float time;
    int state;
    float leftVel;
    float rightVel;
    int leftCurrent;
    int rightCurrent;
    float leftVoltage;
    float rightVoltage;
    float distance;
    float heading;
    float turnrate;
    float x;
    float y;
    float velocity;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
    float velRef;
    float headingRef;
    float turnrateRef;
    bool isUsingHeadingController;
};

class Logger
{
private:
    StateMachine *sm;
    PoseTracker *pose;
    LogEntry *logArray; // Array to hold 100 log entries
    int logIndex = 0;       // Keeps track of the current index
    int maxLogEntries;

public:
    Logger(StateMachine *sm, PoseTracker *pose, int maxEntries = 5000): sm(sm), pose(pose), logIndex(0), maxLogEntries(maxEntries)
    {
        logArray = new LogEntry[maxLogEntries]; // Allocate the array on the heap
    }

    ~Logger()
    {
        delete[] logArray; // Clean up the dynamically allocated array
    }

    void log()
    {
        if (logIndex >= maxLogEntries) return; // Prevents overflow of the array

        LogEntry entry;
        entry.time = float(micros() - sm->getStartTime() * 1000) / 1000.0;
        entry.state = sm->getState();
        entry.leftVel = -encoder.motorVelocity[0] * WHEEL_RADIUS / GEAR;
        entry.rightVel = encoder.motorVelocity[1] * WHEEL_RADIUS / GEAR;
        entry.leftCurrent = analogRead(A1);
        entry.rightCurrent = analogRead(A0);
        entry.leftVoltage = motor.motorVoltage[0];
        entry.rightVoltage = motor.motorVoltage[1];
        entry.distance = pose->getTraveledDistance();
        entry.heading = pose->getHeading();
        entry.turnrate = pose->getTurnrate();
        entry.x = pose->getX();
        entry.y = pose->getY();
        entry.velocity = pose->getVelocity();
        entry.gyroX = imu2.gyro[0];
        entry.gyroY = imu2.gyro[1];
        entry.gyroZ = imu2.gyro[2];
        entry.accX = imu2.acc[0];
        entry.accY = imu2.acc[1];
        entry.accZ = imu2.acc[2];
        entry.velRef = sm->getVelocityReference();
        entry.headingRef = sm->getHeadingReference();
        entry.turnrateRef = sm->getTurnrateReference();
        entry.isUsingHeadingController = sm->getIsUsingHeadingController();

        // Store the entry into the array
        logArray[logIndex++] = entry;
    }

    void
    printLog()
    {
        // Print the header
        Serial.println("time, state, left_vel, right_vel, left_current, right_current, left_voltage, right_voltage, distance, heading, turnrate, x, y, velocity, gyroX, gyroY, gyroZ, accX, accY, accZ, vel_ref, heading_ref, turnrate_ref, is_using_heading_controller");

        // Print each log entry
        for (int i = 0; i < logIndex; i++)
        {
            Serial.print(logArray[i].time);
            Serial.print(",");
            Serial.print(logArray[i].state);
            Serial.print(",");
            Serial.print(logArray[i].leftVel);
            Serial.print(",");
            Serial.print(logArray[i].rightVel);
            Serial.print(",");
            Serial.print(logArray[i].leftCurrent);
            Serial.print(",");
            Serial.print(logArray[i].rightCurrent);
            Serial.print(",");
            Serial.print(logArray[i].leftVoltage);
            Serial.print(",");
            Serial.print(logArray[i].rightVoltage);
            Serial.print(",");
            Serial.print(logArray[i].distance);
            Serial.print(",");
            Serial.print(logArray[i].heading);
            Serial.print(",");
            Serial.print(logArray[i].turnrate);
            Serial.print(",");
            Serial.print(logArray[i].x);
            Serial.print(",");
            Serial.print(logArray[i].y);
            Serial.print(",");
            Serial.print(logArray[i].velocity);
            Serial.print(",");
            Serial.print(logArray[i].gyroX);
            Serial.print(",");
            Serial.print(logArray[i].gyroY);
            Serial.print(",");
            Serial.print(logArray[i].gyroZ);
            Serial.print(",");
            Serial.print(logArray[i].accX);
            Serial.print(",");
            Serial.print(logArray[i].accY);
            Serial.print(",");
            Serial.print(logArray[i].accZ);
            Serial.print(",");
            Serial.print(logArray[i].velRef);
            Serial.print(",");
            Serial.print(logArray[i].headingRef);
            Serial.print(",");
            Serial.print(logArray[i].turnrateRef);
            Serial.print(",");
            Serial.println(logArray[i].isUsingHeadingController);
        }
    }
};

#endif // LOGGER_H