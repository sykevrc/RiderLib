#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


// Create robodash selector
rd::Selector selector({
    {"SAWP", &sawp, "", 50},//done
    {"Q L", &q_l, "", 50},//done
    {"ElimL", &elim_l, "", 50},//done
    {"Q R", &q_r, "", 150},
    {"ElimR", &elim_r, "", 150},
    {"Skills", &skillsv2, "", 250},
});

// Create robodash console
rd::Console console;
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensor
    rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

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
    //skillsv2();
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
        if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            stop();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            outtakefast();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            run_intake();    
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            scoretop();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            scorebottom();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            match.toggle();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            scoot();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            scorebottomslow();
        }
        chassis.lateralSettings.kP=3;
        // delay to save resources
        pros::delay(10);
    }
}

