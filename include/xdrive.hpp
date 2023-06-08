#ifndef DRIVETRAIN
#define DRIVETRAIN

#include <vex.h>

class drivetrain {

    private:

        vex::motor * leftMotor1;
        vex::motor * leftMotor2;
        vex::motor * rightMotor1;
        vex::motor * rightMotor2;
        vex::gps * gps;

        double leftMotor1Speed, leftMotor2Speed;
        double rightMotor1Speed, rightMotor2Speed;

        const double distance_tolerance = 5.0;
        const double angle_tolerance = 1.0;

        const double kp = 0, ki = 0, kd = 0;
        const double kpa = 0, kia = 0, kda = 0;
        double theta;
        double sine, cosine;
        double currentX, currentY, currentAngle; // Cartesian coordinates
        double relativeX, relativeY, relativeAngle; // Cartesian error
        double axis1Error, axis2Error, angleError; // Basis changed error
        double proportionalAxis1, proportionalAxis2, proportionalAngle; // proportional
        double integralAxis1, integralAxis2, integralAngle; // integral
        double derivativeAxis1, derivativeAxis2, derivativeAngle; // derivative
        double lastAxis1, lastAxis2, lastAngle; // last
        double powerAxis1, powerAxis2, powerAngle; // sum

    public:

        drivetrain(vex::motor * lm1, vex::motor * lm2, vex::motor * rm1, vex::motor * rm2, vex::gps * gps);
        void move(double x, double y, double angle);
        void turn_degrees(double angle);
        void turn_heading(double heading);

};

#endif // #ifndef DRIVETRAIN