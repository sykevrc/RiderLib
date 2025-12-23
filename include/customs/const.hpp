#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/distanceSensor.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

using namespace pros;
using namespace lemlib;


extern Controller controller;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Motor intake;
extern Motor top;
extern Motor out;

extern Imu imu;


extern adi::Pneumatics hood;
extern adi::Pneumatics match;
extern adi::Pneumatics lift;

//extern Rotation verticalEnc;
//extern Rotation horizontalEnc;

extern Drivetrain drivetrain;

extern DistanceSensor right;
extern DistanceSensor left;

extern Chassis chassis;
extern Optical colorsens;
extern Distance rightdist;
extern Distance leftdist;