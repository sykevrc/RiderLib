#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.h"
#include "pros/rtos.hpp"

void run_intake(){
    intake.move_voltage(13000);
    top.move_voltage(0);
    hood.retract();
    lift.retract();
}

void outtake(){
    intake.move_voltage(-6000);
    top.move_voltage(-8000);
    lift.extend();
}
void outtakefast(){
    intake.move_voltage(-8000);
    top.move_voltage(-8000);
}
void scoretop(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(13000);
    hood.extend();
}
void scorebottom(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(-10000);

    //hood.extend();  
}
void scorebottomslow(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(-2000);
}
void scoot(){
    intake.move_voltage(13000);
    top.move_voltage(10000);
    out.move_voltage(10000);
}
void stop(){
    intake.move_voltage(0);
    top.move_voltage(0);
    out.move_voltage(0);

}


void redloaderquick(){
    chassis.moveToPoint(-70,chassis.getPose().y,200,{.minSpeed=50});
    chassis.moveToPoint(-70,chassis.getPose().y,1000,{.maxSpeed=14,.minSpeed=10});
    //chassis.moveToPoint(chassis.getPose().x+.5,chassis.getPose().y,300,{.forwards=false,.minSpeed=30});
    //chassis.moveToPoint(chassis.getPose().x-15,chassis.getPose().y,1000,{.minSpeed=30});

}
void redloaderskillsclose(){
    chassis.moveToPoint(-70,chassis.getPose().y,200);
    chassis.moveToPoint(-70,chassis.getPose().y,1000,{.maxSpeed=14,.minSpeed=10});
    chassis.moveToPoint(chassis.getPose().x+1,chassis.getPose().y,300,{.forwards=false,.maxSpeed=35,.minSpeed=30});

    chassis.moveToPoint(-70,chassis.getPose().y,1600,{.maxSpeed=16,.minSpeed=10});

}
void redloaderskillsfar(){
    chassis.moveToPoint(70,chassis.getPose().y,200);
    chassis.moveToPoint(70,chassis.getPose().y,1000,{.maxSpeed=14,.minSpeed=10});
    chassis.moveToPoint(chassis.getPose().x-1,chassis.getPose().y,300,{.forwards=false,.maxSpeed=35,.minSpeed=30});

    chassis.moveToPoint(70,chassis.getPose().y,1600,{.maxSpeed=16,.minSpeed=10});
}
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
//constexpr float degToRad(float deg) { return deg * M_PI / 180; }

void skillsv2() {
   
    //left red
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+4.5),14,0); 
    chassis.moveToPose(chassis.getPose().x, 48,-90,3000,{.earlyExitRange=2});
    hood.extend();
    run_intake();
    match.toggle();
    chassis.waitUntilDone();
    redloaderskillsclose();

    
    // chassis.moveToPoint(-24,48,1000,{.forwards=false,.maxSpeed=50});
    // chassis.waitUntilDone();
    // chassis.setPose(-28.75,48,chassis.getPose().theta);
    // scoretop();
    // pros::delay(1000);
    // outtake();
    // pros::delay(200);
    // scoretop();
    // pros::delay(2000);
    // match.retract();
   
    chassis.moveToPoint(-44,48,700,{.forwards=false});
    top.move_relative(600,600);
    //cross
    chassis.turnToHeading(-45,400);
    match.retract();
    run_intake();
    chassis.moveToPoint(-24,30,1000,{.forwards=false,.minSpeed=20,.earlyExitRange=4});//switch to far
    chassis.turnToHeading(-90,400);

    chassis.moveToPoint(30,30,1000,{.forwards=false,.minSpeed=20,.earlyExitRange=4});
    chassis.moveToPoint(40,48,1200,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(22,49,700,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(28.75,48,chassis.getPose().theta);
    scoretop();
    pros::delay(1000);
    outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);
    chassis.moveToPoint(55,47,1000);
    run_intake();
    match.extend();
    chassis.turnToHeading(90,300);
    chassis.waitUntilDone();
    redloaderskillsfar();   
    chassis.moveToPoint(20,48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(28.75,48,chassis.getPose().theta);

    scoretop();
    pros::delay(1000);
    outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);

    chassis.moveToPoint(33,48,500);
    match.retract();
    
    chassis.moveToPose(43,-24,180,3000,{.minSpeed=20,.earlyExitRange=4});
    run_intake();
    //far right 
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+4),chassis.getPose().y,chassis.getPose().theta);
    pros::delay(50);
    chassis.moveToPoint(37,-48,700);
    chassis.turnToHeading(90,1000,{.maxSpeed=40});
    match.extend();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, -70.5+(rightdist.get_distance()/25.4+4),chassis.getPose().theta);
    chassis.waitUntilDone();
    chassis.moveToPoint(54,-48,700);
    chassis.turnToHeading(90,500);
    chassis.waitUntilDone();
    redloaderskillsfar();

    
    

    chassis.moveToPoint(44,-48,800,{.forwards=false});
    top.move_relative(600,600);
    match.retract();
    //right
    chassis.turnToHeading(135,800);
    run_intake();
    chassis.moveToPoint(24,-30,1000,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(90,400);
    chassis.moveToPoint(-30,-30,1000,{.forwards=false,.minSpeed=20});
    chassis.moveToPoint(-40,-49,1200,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(-90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-22,-48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    scoretop();
    pros::delay(1000);
    outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);
    chassis.moveToPoint(-55,-46.5,1000);
    run_intake();
    match.extend();
    chassis.turnToHeading(-90,400);
    chassis.waitUntilDone();
    redloaderskillsclose();   
    chassis.moveToPoint(-20,-48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);

    scoretop();
    pros::delay(1000);
    outtake();
    pros::delay(200);
    scoretop();
    pros::delay(2000);

    chassis.moveToPoint(-37,-48,800);
    // chassis.turnToHeading(-45,600);

    // chassis.moveToPoint(-62,-24,1000);
     match.toggle();
    // chassis.turnToHeading(0,1000);
    // chassis.waitUntilDone();
    // chassis.setPose(-70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-62,-20,0,1000);
    chassis.moveToPoint(-68, 0, 2000, {.minSpeed=100});
    chassis.moveToPoint(-68, -1, 1000, {.forwards=false,.minSpeed=70});

}
void q_l(){
    chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-16,24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(10.5);
    match.extend();
    chassis.turnToHeading(180,500);
    chassis.moveToPoint(-15,15,1200);
    chassis.turnToHeading(135,500);
    chassis.moveToPoint(-6,6,600);
    hood.extend();
    chassis.turnToHeading(135,200);
    chassis.waitUntilDone();
    outtake();
    pros::delay(500);
    scorebottom();
    pros::delay(2000);
    chassis.moveToPoint(-48,48,1500,{.forwards=false,.minSpeed=10});
    run_intake();
    chassis.turnToHeading(-90,900);
    match.extend();
    
    redloaderquick();

    chassis.moveToPoint(-20,48,1000,{.forwards=false});
    chassis.waitUntil(14);
    scoretop();
}
void q_r(){
    
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+4.5),-13.75,180); 
    pros::delay(50);
    chassis.moveToPose(chassis.getPose().x+2, -49,-90,2000,{.minSpeed=30,.earlyExitRange=4});
    run_intake();
    match.toggle();
    chassis.waitUntilDone();
    redloaderquick();

    chassis.moveToPoint(-29,-49,1000,{.forwards=false});
    chassis.turnToHeading(-90,1000);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, -70.5+(leftdist.get_distance()/25.4+4),chassis.getPose().theta);
    scoretop();
    pros::delay(1200);

    match.toggle();
    run_intake();
    chassis.moveToPoint(-33,-48,500);
    chassis.moveToPoint(-27,-27,1000);
    chassis.moveToPoint(-22,-22,3000,{.maxSpeed=30});
    match.extend();
    chassis.turnToHeading(45,1000);

    chassis.moveToPoint(-11,-11,1000,{.maxSpeed=50});
    match.retract();
    chassis.turnToHeading(45,1000);
    outtakefast();
}
void elim_l(){
    chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-16,24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(10.5);
    match.extend();
    chassis.turnToPoint(-7,30,400);
    match.retract();
    chassis.moveToPoint(-7,30,1000,{.maxSpeed=45});
    chassis.turnToHeading(0,200);
    chassis.moveToPoint(-5.25,40,700);
    chassis.waitUntil(8);
    match.extend();
    pros::delay(500);
    chassis.moveToPoint(-7,30,600,{.forwards=false,.minSpeed=50});
    chassis.turnToHeading(-90,400);

    
    chassis.moveToPoint(-48,50,1600,{.minSpeed=10});
    top.move_relative(400,600);
    chassis.turnToHeading(-90,700);
    
    redloaderquick();
    top.move_relative(300,600);
    chassis.moveToPoint(-20,51,1000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntil(13);
    outtake();
    chassis.waitUntil(17);
    scoretop();
    pros::delay(2000);
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    chassis.moveToPoint(-42,chassis.getPose().y,600);
    chassis.turnToHeading(-45,500);
    chassis.moveToPoint(-25,38,600,{.forwards=false});
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(-9  ,38,600,{.forwards=false});
    // chassis.turnToPoint(-8,48,400);
    // chassis.moveToPoint(-8,48,1000,{.maxSpeed=45});
    // match.retract();
    // chassis.waitUntil(10);
    // match.extend();
    // pros::delay(500);
    // chassis.moveToPoint(-24,26,800,{.forwards=false,.minSpeed=40});
    // chassis.turnToHeading(-90,400);
    
    // chassis.moveToPoint(-46,51,1500,{.minSpeed=10});
    // top.move_relative(600,600);
    // chassis.turnToHeading(-90,900,{.maxSpeed=50});
    
    // redloaderquick();

    // chassis.moveToPoint(-20,52,1000,{.forwards=false});
    // chassis.waitUntil(14);
    // scoretop();
    
}
void elim_r(){
    chassis.setPose(-49,-12,90);
    //chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-16,-26,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.turnToPoint(-7,-30,400);
    match.retract();
    chassis.moveToPoint(-7,-30,1000,{.maxSpeed=45});
    chassis.turnToHeading(180,200);
    chassis.moveToPoint(-5.2,-40,700);
    chassis.waitUntil(8);
    match.extend();
    pros::delay(500);
    chassis.moveToPoint(-7,-35,600,{.forwards=false});
    chassis.turnToHeading(-90,400);

    
    chassis.moveToPoint(-46,-52,2000,{.minSpeed=10});
    top.move_relative(600,600);
    chassis.turnToHeading(-90,700,{.maxSpeed=50});
    
    redloaderquick();
    top.move_relative(300,600);
    chassis.moveToPoint(-20,-52,1000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntil(16);
    scoretop();
    pros::delay(2000);
    chassis.moveToPoint(-42,chassis.getPose().y,600);
    chassis.turnToHeading(-45,500);
    chassis.moveToPoint(-25,-62,600,{.forwards=false});
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(-9  ,-62,600,{.forwards=false});

}
void sawp(){
    chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-12,24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(11);
    match.extend();
    pros::delay(200);
    chassis.moveToPoint(-48,50,1500,{.forwards=false,.minSpeed=10});
    top.move_relative(600,600);
    chassis.turnToHeading(-90,900,{.maxSpeed=50});
    
    redloaderquick();

    chassis.moveToPoint(-20,50,1000,{.forwards=false});
    chassis.waitUntil(10);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    chassis.moveToPoint(-40,48,500);
    match.retract();
    run_intake();
    chassis.moveToPoint(-20,-5,1300);
    chassis.turnToHeading(180,300);
    chassis.moveToPoint(-20,-28,1000);
    chassis.waitUntil(11);
    match.extend();
    chassis.turnToHeading(45,500);
    chassis.moveToPoint(-6,-9,800);
    match.retract();
    outtakefast();
    
    pros::delay(300);
    chassis.moveToPoint(-44,-52,1500,{.forwards=false});
    run_intake();
    match.extend();
    top.move_relative(200,600);
    chassis.turnToHeading(-90,900,{.maxSpeed=50});
    
    redloaderquick();
    //chassis.setPose(chassis.getPose().x,-70.5+(leftdist.get_distance()/25.4+4.5),chassis.getPose().theta);
    chassis.moveToPoint(-20,-52,1000,{.forwards=false,.minSpeed=60});
    chassis.waitUntil(15);
    scoretop();
}
