#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep

const double circ = 7.739; // find this by pushing the chassis forward 60 inches 5 times and average all motor revolution counts.
// the value of circ will be (60*motor_rpm) / (average_rev_counts*wheel_rpm)+7.742

const double calc = 60/(circ*0.75);
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-1,-11,-12},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10,19,20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(-3);
pros::Motor bottom(9);
pros::Motor top(8);

pros::adi::Pneumatics park(7, false);
pros::adi::Pneumatics match(6, false);
// Inertial Sensor on port 7
pros::Imu imu(7);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-4);
// distance sensor, right side on port 12
pros::Distance rightdist(6);
pros::Distance leftdist(5);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2, -.78);
// use distance sensor in the drivetrain
lemlib::DistanceSensor right(&rightdist, 9.25);
lemlib::DistanceSensor left(&leftdist, 9.25);
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14.25, // 14 inch track width
                              (circ / M_PI), // found using empirical testing
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. Since we had traction wheels, it is 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &right, // right side distance
                            &left, // left side distance
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "LeftW: %f", (leftMotors.get_power()+leftMotors.get_power(1)+leftMotors.get_power(2))/3); 
            pros::lcd::print(4, "RightW: %f", (rightMotors.get_power()+rightMotors.get_power(1)+rightMotors.get_power(2))/3); 
            pros::lcd::print(5, "Lticks: %f", (leftMotors.get_position()+leftMotors.get_position(1)+leftMotors.get_position(2))/3); 
            pros::lcd::print(6, "Rticks: %f", (rightMotors.get_position()+rightMotors.get_position(1)+rightMotors.get_position(2))/3); 

            //controller.print(0, 0, "D: %s", rightdist.get());
            // log position telemetry
            float lidarAngle = fmod(chassis.getPose().theta, 360.0f);     // Wrap within [-360, 360)
            if (lidarAngle < 0) lidarAngle += 360.0f;
            
            //printf("%.4f,%.4f,%.4f,%d,%d\n", chassis.getPose().x, chassis.getPose().y, imu.get_heading(),rightdist.get(), leftdist.get());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
constexpr float degToRad(float deg) { return deg * M_PI / 180; }

void autonomous() {
    chassis.setPose(0,-65.35,0);
    for(int i = 0; i < 6; i ++){
        chassis.moveToPoint(24,-24,1500,{.maxSpeed=50});
        chassis.moveToPoint(48,-48,1500,{.maxSpeed=50});
        chassis.moveToPoint(12,-52,1500,{.maxSpeed=50});

        chassis.moveToPoint(-12,-52,1100,{.maxSpeed=50});
        chassis.moveToPoint(-48,-48,1100,{.maxSpeed=50});
        chassis.moveToPoint(-24,-24,1500,{.maxSpeed=50});
    }
    

}
/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            intake.move_voltage(-8000);
            //top.move_voltage(-8000);
            bottom.move_voltage(-8000);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            intake.move_voltage(13000);
            bottom.move_voltage(13000);
            //top.move_voltage(8000);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move_velocity(200);
            bottom.move_velocity(-200);
            top.move_voltage(13000);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move_voltage(13000);
            bottom.move_voltage(-13000);
            top.move_voltage(-13000);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            park.toggle();
        }else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            match.toggle();
        }else{
            intake.move_voltage(0);
            bottom.move_voltage(0);
            top.move_voltage(0);
        }
        // delay to save resources
        pros::delay(10);
    }
}

