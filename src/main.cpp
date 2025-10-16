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
double starthue = 0;
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
    intake.move_voltage(-4000);
    top.move_voltage(-8000);
}
void outtakefast(){
    intake.move_voltage(-6000);
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
void redloader(){
    chassis.moveToPoint(-58,chassis.getPose().y,1000,{.minSpeed=80});
    chassis.moveToPoint(-56,chassis.getPose().y,2000,{.forwards=false,.maxSpeed=30});
    chassis.moveToPoint(-62,chassis.getPose().y,2000,{.maxSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-62,chassis.getPose().y,2000,{.maxSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,500,{.forwards=false,.maxSpeed=20});
    chassis.moveToPoint(-62,chassis.getPose().y,2000,{.maxSpeed=30});
}
void redloaderquick(){
    chassis.moveToPoint(-59,chassis.getPose().y,500,{.minSpeed=70});
    chassis.moveToPoint(-57,chassis.getPose().y,700,{.maxSpeed=30});
    chassis.moveToPoint(-63,chassis.getPose().y,800,{.maxSpeed=40});
}
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
    redloader();

    
    chassis.moveToPoint(-29,48,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    scoretop();
    pros::delay(1500);
    outtake();
    pros::delay(200);
    scoretop();
    pros::delay(3500);
    chassis.moveToPoint(-33,48,3000);

    chassis.moveToPose(-40,-24,180,1000);
    chassis.moveToPoint(-40,-36,3000);
    chassis.turnToHeading(180,1000);
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+3),chassis.getPose().y,chassis.getPose().theta);
    pros::delay(50);
    chassis.moveToPoint(-40,-48,3000);
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, -70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().theta);
    run_intake();
    chassis.waitUntilDone();
    chassis.moveToPose(-50,-48,-90,2000);
    redloader();
    chassis.moveToPoint(-29,-48,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    scoretop();
    pros::delay(1500);
    outtake();
    pros::delay(200);
    scoretop();
    pros::delay(3500);
    chassis.moveToPoint(-64,-36,2000);
    match.toggle();
    chassis.turnToHeading(0,1000);
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-62,-24,0,2000);
    chassis.moveToPoint(-68, 0, 2000, {.minSpeed=100});
    chassis.moveToPoint(-68, -5, 2000, {.forwards=false,.minSpeed=70});

}
void q_team(){
    if(colorsens.get_hue()<20){
        side = true;
    }else{
        side = false;
    }
    
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+4.5),13.75,0); 
    pros::delay(50);
    chassis.moveToPose(chassis.getPose().x, 48,-90,2000,{.minSpeed=30,.earlyExitRange=4});
    run_intake();
    match.toggle();
    chassis.waitUntilDone();
    redloaderquick();

    chassis.moveToPoint(-29,48,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, 70.5-(rightdist.get_distance()/25.4+2),chassis.getPose().theta);
    scoretop();
    pros::delay(1200);

    match.toggle();
    run_intake();
    chassis.moveToPoint(-33,48,500);
    chassis.moveToPoint(-28,28,1000);
    chassis.moveToPoint(-22,22,3000,{.maxSpeed=30});
    chassis.turnToHeading(-45,1000);

    chassis.moveToPoint(-11,13,3000,{.forwards=false,.maxSpeed=50});
    chassis.turnToHeading(-45,1000);
    scorebottom();
}
void elim(){
    if(colorsens.get_hue()<20){
        side = true;
    }else{
        side = false;
    }
    
    chassis.setPose(-49,11,90);
    chassis.moveToPoint(-28,20,1000,{.minSpeed=10});
    run_intake();
    chassis.moveToPoint(-16,30,2000,{.maxSpeed=35,.minSpeed=10});
    chassis.turnToHeading(-50,600);
    chassis.moveToPoint(-50,49.5,1000,{.minSpeed=10});
    match.toggle();
    chassis.turnToHeading(-90,1000);
    redloaderquick();
    chassis.moveToPoint(-27,49.5,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    scoretop();
    
}
void sawp(){
    chassis.setPose(-49,-11,90);
    chassis.moveToPoint(-30,-20,1000,{.minSpeed=10});
    run_intake();
    chassis.moveToPoint(-20,-26,1000,{.maxSpeed=35,.minSpeed=10});
    chassis.turnToHeading(45,500);
    chassis.moveToPoint(-9,-10,1000,{.minSpeed=10});
    outtakefast();
    chassis.waitUntilDone();
    pros::delay(300);
    run_intake();
    chassis.moveToPoint(-20,-20,1000,{.forwards=false,.minSpeed=10});
    chassis.turnToHeading(0,500);
    chassis.moveToPoint(-20,15,1000,{.minSpeed=10});
    chassis.moveToPoint(-23,26,2000,{.maxSpeed=35,.minSpeed=10});
    chassis.turnToHeading(-45,500);
    chassis.moveToPoint(-10,13,3000,{.forwards=false,.maxSpeed=50});
    chassis.turnToHeading(-45,500);
    outtake();
    pros::delay(100);
    scorebottom();
    pros::delay(800);
    run_intake();
    chassis.moveToPoint(-50,49,1000,{.minSpeed=10});
    match.toggle();
    chassis.turnToHeading(-90,500);
    redloaderquick();
    chassis.moveToPoint(-29,chassis.getPose().y,1000,{.forwards=false});
    hood.retract();
    chassis.turnToHeading(-90,500);
    scoretop();
}

// Create robodash selector
rd::Selector selector({
    {"SAWP", &sawp, "", 0},
    {"Q TEAM", &q_team, "", 220},
    {"Elim", &elim, "", 100},
    {"Skills", &skills, "", 100},
});

// Create robodash console
rd::Console console;
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensor
    rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    // colorsens.set_led_pwm(100);
    // starthue = colorsens.get_hue();

    // pros::Task screenTask([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
    //         pros::delay(50);
    //     }
    
    // });
    // pros::Task([&](){
    //     if(side){//red
    //         if (colorsens.get_hue()>starthue+20){
    //             top.move_voltage(13000);
    //             pros::delay(500);
    //             top.move_voltage(0);
    //         }
    //     }else{
    //         if(colorsens.get_hue()<starthue-20){
    //             top.move_voltage(13000);
    //             pros::delay(500);
    //             top.move_voltage(0);
    //         }
    //     }
    //     pros::delay(50);
    // });
    selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
		if (routine == std::nullopt) {
			std::cout << "No routine selected" << std::endl;
		} else {
			std::cout << "Selected Routine: " << routine.value().name << std::endl;
        
         controller.print(0,0,"%s", routine.value().name);
         controller.rumble("- -");
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
void competition_initialize() {
    selector.focus();
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(arc_txt); // '.' replaced with "_" to make c++ happy
ASSET(under_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs in driver control
 */
void autonomous(){
    selector.run_auton();
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
            load_up();
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

