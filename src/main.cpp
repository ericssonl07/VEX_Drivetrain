#include <vex.h>
#include "xdrive.hpp"

const int leftMotor1Port  = vex::PORT1;
const int leftMotor2Port  = vex::PORT2;
const int rightMotor1Port = vex::PORT3;
const int rightMotor2Port = vex::PORT4;
const int gpsPort         = vex::PORT5;

const bool leftMotor1Reverse = false;
const bool leftMotor2Reverse = false;
const bool rightMotor1Reverse = false;
const bool rightMotor2Reverse = false;

const vex::gearSetting gearSetting = vex::gearSetting::ratio6_1;

vex::motor leftMotor1(leftMotor1Port, gearSetting, leftMotor1Reverse);
vex::motor leftMotor2(leftMotor2Port, gearSetting, leftMotor2Reverse);
vex::motor rightMotor1(rightMotor1Port, gearSetting, rightMotor1Reverse);
vex::motor rightMotor2(rightMotor2Port, gearSetting, rightMotor2Reverse);
vex::gps gps(gpsPort, 0.0, vex::turnType::left);

drivetrain bot(&leftMotor1, &leftMotor2, &rightMotor1, &rightMotor2, &gps);

void autonomousSequence(void) {
    bot.move(0, 0, 0);
}

void controlSequence(void) {
    bot.control();
}

int main() {
    vex::competition competition;
    competition.autonomous(autonomousSequence);
    competition.drivercontrol(controlSequence);
}