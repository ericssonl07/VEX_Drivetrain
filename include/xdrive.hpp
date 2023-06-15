#ifndef DRIVETRAIN
#define DRIVETRAIN

#include <vex.h>
#include <functional>

class drivetrain {

    private:

        vex::motor * leftMotor1;
        vex::motor * leftMotor2;
        vex::motor * rightMotor1;
        vex::motor * rightMotor2;
        vex::gps * gps;

        double leftMotor1Speed;
        double leftMotor2Speed;
        double rightMotor1Speed;
        double rightMotor2Speed;

        const double distance_tolerance = 5.0;
        const double angle_tolerance = 1.0;

        const double kp = 0;
        const double ki = 0;
        const double kd = 0;
        const double kpa = 0;
        const double kia = 0;
        const double kda = 0;
        double theta;
        double sine;
        double cosine;
        double currentX;
        double currentY;
        double currentAngle;
        double relativeX;
        double relativeY;
        double relativeAngle;
        double axis1Error;
        double axis2Error;
        double angleError;
        double proportionalAxis1;
        double proportionalAxis2;
        double proportionalAngle;
        double integralAxis1;
        double integralAxis2;
        double integralAngle;
        double derivativeAxis1;
        double derivativeAxis2;
        double derivativeAngle;
        double lastAxis1;
        double lastAxis2;
        double lastAngle;
        double powerAxis1;
        double powerAxis2;
        double powerAngle;

        double controllerAxis1;
        double controllerAxis2;
        double controllerAxis3;
        double controllerAxis4;

        bool toggle(vex::controller::button button, bool & pastState, bool & toggleState);

    public:

        drivetrain(vex::motor * lm1, vex::motor * lm2, vex::motor * rm1, vex::motor * rm2, vex::gps * gps);
        void move(double x, double y, double angle);
        void turn_degrees(double angle);
        void turn_heading(double heading);

        void control();

};

#endif // #ifndef DRIVETRAIN