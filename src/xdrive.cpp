#include "xdrive.hpp"
#include <math.h>
#include <vex.h>

drivetrain::drivetrain(vex::motor * leftMotor1, vex::motor * leftMotor2, vex::motor * rightMotor1, vex::motor * rightMotor2, vex::gps * gps) {
    this -> leftMotor1 = leftMotor1;
    this -> leftMotor2 = leftMotor2;
    this -> rightMotor1 = rightMotor1;
    this -> rightMotor2 = rightMotor2;
    this -> gps = gps;
}

void drivetrain::move(double x, double y, double a) {
    const double d_t = distance_tolerance, a_t = angle_tolerance;
    integralAxis1 = integralAxis2 = integralAngle = 0;
    currentX = gps -> xPosition();
    currentY = gps -> yPosition();
    currentAngle = gps -> heading();
    relativeX = x - currentX;
    relativeY = y - currentY;
    relativeAngle = a - currentAngle;
    theta = currentAngle + 45;
    sine = sin(theta / 180.0 * M_PI);
    cosine = cos(theta / 180.0 * M_PI);
    lastAxis1 = cosine * currentX + sine * currentY;
    lastAxis2 = cosine * currentY - sine * currentX;
    angleError = relativeAngle;
    lastAngle = angleError;
    while (fabs(relativeX) > d_t or fabs(relativeY) > d_t or fabs(relativeAngle) > a_t) {
        currentX = gps -> xPosition();
        currentY = gps -> yPosition();
        currentAngle = gps -> heading();
        relativeX = x - currentX;
        relativeY = y - currentY;
        relativeAngle = a - currentAngle;
        theta = currentAngle + 45;
        sine = sin(theta / 180.0 * M_PI);
        cosine = cos(theta / 180.0 * M_PI);
        axis1Error = cosine * currentX + sine * currentY;
        axis2Error = cosine * currentY - sine * currentX;
        angleError = relativeAngle;

        proportionalAxis1 = axis1Error * kp;
        proportionalAxis2 = axis2Error * kp;
        proportionalAngle = angleError * kpa;

        integralAxis1 += axis1Error * ki;
        integralAxis2 += axis2Error * ki;
        integralAngle += angleError * kia;

        derivativeAxis1 = (axis1Error - lastAxis1) * kd;
        derivativeAxis2 = (axis2Error - lastAxis2) * kd;
        derivativeAngle = (angleError - lastAngle) * kda;

        lastAxis1 = axis1Error;
        lastAxis2 = axis2Error;
        lastAngle = angleError;

        powerAxis1 = proportionalAxis1 + integralAxis1 + derivativeAxis1;
        powerAxis2 = proportionalAxis2 + integralAxis2 + derivativeAxis2;
        powerAngle = proportionalAngle + integralAngle + derivativeAngle;

        leftMotor1Speed = powerAxis1 - powerAngle;
        leftMotor2Speed = powerAxis2 - powerAngle;
        rightMotor1Speed = powerAxis2 + powerAngle;
        rightMotor2Speed = powerAxis1 + powerAngle;

        leftMotor1 -> spin(vex::directionType::fwd, leftMotor1Speed, vex::percentUnits::pct);
        leftMotor2 -> spin(vex::directionType::fwd, leftMotor2Speed, vex::percentUnits::pct);
        rightMotor1 -> spin(vex::directionType::fwd, rightMotor1Speed, vex::percentUnits::pct);
        rightMotor2 -> spin(vex::directionType::fwd, rightMotor2Speed, vex::percentUnits::pct);
    }
}

void drivetrain::turn_degrees(double angle) {
    move(gps -> xPosition(), gps -> yPosition(), gps -> heading() + angle);
}

void drivetrain::turn_heading(double heading) {
    move(gps -> xPosition(), gps -> yPosition(), heading);
}