#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"

const double circ = 7.861; // find this by pushing the chassis forward 60 inches 5 times and average all motor revolution counts.
// the value of circ will be (60*motor_rpm) / (average_rev_counts*wheel_rpm)+7.742
//2846.8
//2824.4
const double calc = 60/(circ*0.75);
int starthue = 0;
bool side = true; //false is blue
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2,-19,-12},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10,20,13}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(-1);
pros::Motor top(-14);
//pros::Motor top(8);

pros::adi::Pneumatics hood(1, false);
pros::adi::Pneumatics match(2, false);
// Inertial Sensor on port 11
pros::Imu imu(11);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(3);
// distance sensor, right side on port 12
pros::Distance rightdist(9);
pros::Distance leftdist(5);

pros::Optical colorsens(4);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -5.75);
// vertical tracking wheel. 2" diameter, .5" offset, right of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2.1, .5);
// use distance sensor in the drivetrain
lemlib::DistanceSensor right(&rightdist, 9.25);
lemlib::DistanceSensor left(&leftdist, 5);
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14.25, // 14 inch track width
                              (circ / M_PI), // found using empirical testing
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. Since we had traction wheels, it is 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(4, // proportional gain (kP)
                                            0.25, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            7 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.7, // proportional gain (kP)
                                             0.5, // integral gain (kI)
                                             5, // derivative gain (kD)
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


void run_intake(){
    intake.move_voltage(13000);
    top.move_voltage(0);
}
void load_up(){
    intake.move_voltage(13000);
    if(side){//red
        if(colorsens.get_hue()<starthue-20){
        top.move_voltage(-3000);
        }else if (colorsens.get_hue()>starthue+20){
            top.move_voltage(10000);
        }else{
            top.move_voltage(13000);
        }
    }else{
        if(colorsens.get_hue()<starthue-20){
        top.move_voltage(10000);
        }else if (colorsens.get_hue()>starthue+20){
            top.move_voltage(-3000);
        }else{
            top.move_voltage(13000);
        }
    }
    
}
void outtake(){
    intake.move_voltage(-8000);
    top.move_voltage(-8000);
}
void scoretop(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    hood.retract();
}
void scorebottom(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    hood.extend();  
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensor
    rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    colorsens.set_led_pwm(100);
    colorsens.set_integration_time(3);

    

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread to for brain screen and position logging
    //printf("time,x,y,heading,rightDist,leftDist\n");

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // pros::lcd::print(3, "LeftW: %f", (leftMotors.get_power()+leftMotors.get_power(1)+leftMotors.get_power(2))/3); 
            // pros::lcd::print(4, "RightW: %f", (rightMotors.get_power()+rightMotors.get_power(1)+rightMotors.get_power(2))/3); 
            // pros::lcd::print(5, "Lticks: %f", (leftMotors.get_position()+leftMotors.get_position(1)+leftMotors.get_position(2))/3); 
            // pros::lcd::print(6, "Rticks: %f", (rightMotors.get_position()+rightMotors.get_position(1)+rightMotors.get_position(2))/3); 
            //controller.print(0, 0, "D: %s", rightdist.get());
            // // log position telemetry
            // float lidarAngle = fmod(chassis.getPose().theta, 360.0f);     // Wrap within [-360, 360)
            // if (lidarAngle < 0) lidarAngle += 360.0f;


            // printf("%.4f,%.4f,%.4f,%d,%d\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta,rightdist.get(), leftdist.get());
            
            // delay to save resources
            pros::delay(50);
        }
    starthue = colorsens.get_hue();
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
//constexpr float degToRad(float deg) { return deg * M_PI / 180; }

void skills() {
    pros::Task([&](){
        if((intake.get_voltage()/13000)*0.75 > intake.get_actual_velocity()/127){
            intake.move_voltage(-1000);
            pros::delay(250);
            intake.move_voltage(13000);
        }
        if((top.get_voltage()/13000)*0.75 > top.get_actual_velocity()/127){
            top.move_voltage(-1000);
            pros::delay(250);
            top.move_voltage(13000);
        }
        pros::delay(50);
    });
    //chassis.setPose(0,0,0);
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+4.5),14.25,0); 
    pros::delay(50);
    chassis.moveToPose(chassis.getPose().x, 48,-90,3000,{.earlyExitRange=2});
    run_intake();
    match.toggle();
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-61.5,chassis.getPose().y,1000,{.minSpeed=80});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(-61.5,chassis.getPose().y,2000,{.maxSpeed=30});

    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-61.5,chassis.getPose().y,2000,{.maxSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-62,chassis.getPose().y,2000,{.maxSpeed=30});
    
    chassis.moveToPoint(-29,48,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    scoretop();
    pros::delay(1500);
    outtake();
    pros::delay(200);
    scoretop();
    pros::delay(1500);
    chassis.moveToPose(-40,-24,180,1000);
    chassis.moveToPoint(-40,-48,3000);
    chassis.turnToHeading(180,1000);
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+3),chassis.getPose().y,chassis.getPose().theta);
    pros::delay(50);
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, -70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().theta);
    run_intake();
    chassis.waitUntilDone();
    chassis.moveToPose(-50,-48,-90,2000);
    chassis.moveToPoint(-61.5,-48,1000,{.minSpeed=70});
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveToPoint(-61.5,chassis.getPose().y,2000,{.maxSpeed=30});

    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-61.5,chassis.getPose().y,2000,{.maxSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-62,chassis.getPose().y,2000,{.maxSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-61.5,chassis.getPose().y,2000,{.maxSpeed=30});

    chassis.moveToPoint(-29,-48,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    scoretop();
    pros::delay(1500);
    outtake();
    pros::delay(200);
    scoretop();
    pros::delay(1500);
    chassis.moveToPoint(-64,-36,2000);
    match.toggle();
    chassis.turnToHeading(0,1000);
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-62,-24,0,2000);
    chassis.moveToPoint(-68, 0, 2000, {.minSpeed=100});
    chassis.moveToPoint(-68, -5, 2000, {.forwards=false,.minSpeed=70});

}
void sawp(){

}
void qteam(){

}
void l_elim(){

}
void r_elim(){

}
// Create robodash selector
rd::Selector selector({
    // {"Q SAWP", &sawp, "", 0},
    // {"Q TEAM", &qteam, "", 220},
    // {"L Elim", &l_elim, "", 100},
    // {"R Elim", &r_elim, "", 100},
    {"Skills", &skills, "", 100},
});

// Create robodash console
rd::Console console;
/**
 * Runs in driver control
 */
void autonomous(){
    skills();
}
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
            outtake();
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            run_intake();    
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            scoretop();
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            scorebottom();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            hood.toggle();
        }else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            match.toggle();
        }else{
            intake.move_voltage(0);
            //bottom.move_voltage(0);
            top.move_voltage(0);
        }
        // delay to save resources
        pros::delay(10);
    }
}

