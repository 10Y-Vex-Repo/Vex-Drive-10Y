#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
pros::Motor intake(-4, pros::MotorGearset::blue);
pros::Motor toptake(8, pros::MotorGearset::blue);
pros::Motor intake2(-5, pros::MotorGearset::blue);

// pnuematics
pros::adi::Pneumatics descore('F', false);
pros::adi::Pneumatics matchLoader('B', false);
pros::adi::Pneumatics hood('D', false);


// motor groups
pros::MotorGroup leftMotors({15, 13, 14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-17, -18, -16}, pros::MotorGearset::blue);
pros::MotorGroup aleftMotors({-15, -13, -14}, pros::MotorGearset::blue);
pros::MotorGroup arightMotors({17, 18, 16}, pros::MotorGearset::blue);

// sensor
pros::Imu imu(7);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(19);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -1);

// drivetrain settings
lemlib::Drivetrain drivetrain(&aleftMotors, // left motor group
                              &arightMotors, // right motor group
                              13, // 11 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              150, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              250, // large error range timeout, in milliseconds
                                              8 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              6 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
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
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

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
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
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

void intakesf(int intakeS, int intake2S) {
    intake.move(intakeS);
    intake2.move(intake2S);
}

void intakes(int intakelow, int intakemid, int intakehigh) {
    intake.move(intakelow);
    intake2.move(intakemid);
    toptake.move(intakehigh);
}

void toptakes(int topTake, int midtake) {
    // toptake.move(topTake);
    toptake.move(midtake);
}
// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void parkClear(int n) {
    for (int i = 0; i < n; i++) {
        chassis.tank(-80, 80);
        pros::delay(200);
        chassis.tank(0, 0);
        chassis.tank(80, -80);
        pros::delay(200);
    }
    chassis.tank(0, 0);
}

void skillsAuton2() {
    //experimenting with early exit range to motion change so faster
    // add resets
    chassis.setPose(0, 0, 180);
    intakes(127, 127, 127);
    chassis.moveToPoint(0, -2.3, 2000, {.minSpeed = 120});
    pros::delay(500);
    chassis.moveToPoint(0, -8, 1500, {.minSpeed = 120});
    parkClear(3);
    chassis.turnToHeading(180, 1000);
    chassis.setPose(0, -10, chassis.getPose().theta);
    // park zone balls

    chassis.moveToPoint(0, 2, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
    chassis.turnToHeading(60, 1000, {.maxSpeed = 70, .minSpeed = 5, .earlyExitRange = 3});
    chassis.moveToPoint(22, 20, 2000, {.maxSpeed = 70, .minSpeed = 5, .earlyExitRange = 2});
    // // right mid goal balls

    chassis.turnToHeading(-37, 1000, {.maxSpeed = 60, .minSpeed = 4});
    chassis.moveToPoint(12, 30, 1500, {.maxSpeed = 70, .minSpeed = 10});
    pros::delay(500);
    intakes(-80, -80, -80);
    pros::delay(3000);
    // low mid goal scoring

    // chassis.moveToPoint(18.5, 38.5, 1000, {.earlyExitRange = 2});
    // intakes(127, 127, 127);
    // chassis.turnToHeading(-110, 1000, {.earlyExitRange = 5});
    // chassis.moveToPoint(-26.5, 31, 1500, {.minSpeed = 80, .earlyExitRange = 3});
    // // left mid goal balls
    
    // chassis.turnToHeading(-135, 1000, {.earlyExitRange = 4});
    // matchLoader.extend();
    // chassis.moveToPoint(-45.5, 2, 1500, {.earlyExitRange = 1});
    // chassis.turnToHeading(-180, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(-46, -10, 1000);
    // pros::delay(2000);
    // // first matchload

    // chassis.moveToPoint(-60, 31, 1500, {.forwards = false, .earlyExitRange = 2});
    // chassis.turnToHeading(-180, 1000, {.earlyExitRange = 4});
    // chassis.moveToPoint(-59.5, 82.5, 2000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 3});
    // matchLoader.retract();
    // // arrive at other side
    
    // chassis.moveToPoint(-47, 104, 1500, {.forwards = false, .earlyExitRange = 1});  
    // chassis.turnToHeading(0, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(-46.5, 84.5, 1500, {.forwards = false});
    // hood.extend();
    // pros::delay(2000);
    // matchLoader.extend();
    // // 1st long goal scoring

    // chassis.moveToPoint(-46, 122, 1500);
    // hood.retract();
    // pros::delay(2000);
    // // 2nd matchloading

    // chassis.moveToPoint(-46.5, 84.5, 1500, {.forwards = false});
    // hood.extend();
    // pros::delay(2000);
    // // 2nd long goal scoring

    // chassis.turnToHeading(90, 1000, {.earlyExitRange = 5});
    // matchLoader.retract();
    // chassis.moveToPoint(1.5, 101.5, 1500, {.earlyExitRange = 1});
    // hood.retract();
    // chassis.turnToHeading(0, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(1.5, 119, 2000);
    // chassis.moveToPoint(1.5, 122.5, 1000);
    // pros::delay(1000);
    // // second park balls

    // chassis.moveToPoint(1.5, 101.5, 1500, {.forwards = false, .earlyExitRange = 2});
    // chassis.turnToHeading(135, 1000, {.earlyExitRange = 5});
    // chassis.moveToPoint(27.5, 78, 1500, {.earlyExitRange = 2});
    // matchLoader.extend();
    // chassis.moveToPoint(24, 81, 1000, {.earlyExitRange = 2});
    // // second mid balls
    
    // chassis.turnToHeading(45, 1000, {.earlyExitRange = 5});
    // chassis.moveToPoint(11.5, 69, 1500);
    // intakes(80, 80, -60);
    // pros::delay(3000);
    // intakes(127, 127, 127);
    // // top mid goal scoring

    // chassis.moveToPoint(48, 107, 1500, {.minSpeed = 80, .earlyExitRange = 2});
    // chassis.turnToHeading(0, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(48, 122, 1500);
    // pros::delay(2000);
    // // third matchloading

    // chassis.moveToPoint(62, 81, 2000, {.forwards = false, .earlyExitRange = 2});
    // chassis.turnToHeading(0, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(61.5, 30, 2000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2});
    // matchLoader.retract();
    // // arrival at other side

    // chassis.moveToPoint(48, 11, 1500, {.forwards = false, .earlyExitRange = 1});
    // chassis.turnToHeading(180, 1000, {.earlyExitRange = 3});
    // chassis.moveToPoint(48.5, 29.5, 1000, {.forwards = false});
    // hood.extend();
    // pros::delay(2000);
    // matchLoader.extend();
    // // 3rd long goal scoring

    // chassis.moveToPoint(48, -7.5, 1500);
    // hood.retract();
    // pros::delay(2000);
    // // 4th matchloading

    // chassis.moveToPoint(48.5, 29.5, 2000, {.forwards = false});
    // hood.extend();
    // pros::delay(2000);
    // // 4th long goal scoring

    // chassis.moveToPoint(29.5, 11.5, 2000, {.earlyExitRange = 3});
    // chassis.moveToPoint(17.5, -5.5, 1500, {.earlyExitRange = 3});
    // chassis.turnToHeading(-90, 1000, {.earlyExitRange = 5});
    // chassis.moveToPoint(0.5, -6.5, 1500);
    // // park
}

void skillsAuton() {
    // 1010G route btw
    // start on edge of parking zone
    // chassis.setPose(0, 0, 0);
    // intake.move(-127);
    // toptakes(-100, 0);
    // chassis.moveToPoint(0, 20, 4000, {.maxSpeed = 80});
    // chassis.moveToPoint(0, 0, 2000);
    // chassis.moveToPoint(0, 20, 2000, {.maxSpeed = 50});
    // // end of parking balls

    // chassis.setPose(0, 0, 180);
    // intake.move(0);
    // toptakes(0, 0);
    // chassis.moveToPoint(-2, 21, 2000, {.forwards = false});
    // chassis.turnToHeading(-100, 1000, {.maxSpeed = 60});
    // toptakes(-100, 0);
    // intake.move(-127);
    // // chassis.moveToPoint(-11.5, 22, 2000, {.maxSpeed = 80});
    // chassis.moveToPoint(-15, 20, 2000, {.maxSpeed = 80});
    // pros::delay(1000);
    // intake.move(0);
    // toptakes(0, 0);
    // // end of mid balls

    // chassis.turnToHeading(-135, 1000, {.maxSpeed = 60});
    // chassis.moveToPoint(-9.5, 28, 2000, {.forwards = false});
    // pros::delay(300);
    // toptakes(-127, -70);
    // intake.move(-100);
    // pros::delay(1500);
    // toptakes(0, 0);
    // intake.move(0);
    // //end of mid goal scoring hope for 7 balls

    // chassis.moveToPoint(-35, 0, 2000, {.maxSpeed = 100});
    // matchLoader.extend();
    // chassis.turnToHeading(-175, 1000);
    // toptakes(-100, 0);
    // intake.move(-127);
    // chassis.moveToPoint(-35.5, -7, 1000, {.maxSpeed = 40});
    // chassis.moveToPoint(-35.5, -11.5, 2000, {.maxSpeed = 20, .minSpeed = 20}); 
    // // matchLoad(3);
    // pros::delay(2400);  
    // // matchloading end

    // chassis.moveToPoint(-35, -3, 1000, {.forwards = false});
    // chassis.turnToHeading(150, 1000);
    // chassis.moveToPoint(-45.5, 10, 2000, {.forwards = false});
    // chassis.turnToHeading(-185, 1000);
    // intake.move(0);
    // toptakes(0, 0);
    // matchLoader.retract();
    // chassis.moveToPoint(-45.5, 69, 4000, {.forwards = false, .maxSpeed = 100});
    // // arriving at other side

    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(-34.6, 70, 1500);
    // chassis.turnToHeading(0, 1000);
    // chassis.moveToPoint(-34.3, 60, 1500, {.forwards = false, .maxSpeed = 80});
    // intake.move(-127);
    // toptakes(127, 114);
    // matchLoader.extend();
    // pros::delay(3000);
    // toptakes(0, 0);
    // intake.move(0);
    // // end of first long goal score (at other side)

    // chassis.moveToPoint(-33.8, 83, 2000, {.maxSpeed = 50});
    // toptakes(-100, 0);
    // intake.move(-127);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-33.8, 88.5, 4000, {.maxSpeed = 20, .minSpeed = 20});
    // pros::delay(2400);
    // chassis.cancelMotion();
    // intake.move(0);
    // toptakes(0, 0);
    // // end of second matchloading

    // chassis.moveToPoint(-34.5, 70, 1000, {.forwards = false, .minSpeed = 80});
    // chassis.moveToPoint(-34.5, 60, 1000, {.forwards = false, .maxSpeed = 60});
    // matchLoader.retract();
    // pros::delay(200);
    // // slowing down so no accidental descores
    // intake.move(-127);
    // toptakes(127, 114);
    // chassis.turnToHeading(0, 1000);
    // pros::delay(3000);
    // intake.move(0);
    // toptakes(0, 0);
    // // end of second long goal scoring

    // // moving to right side
    // chassis.moveToPoint(-34, 70, 1500);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(37.8, 70, 4000);
    // // arrival at top right side
    
    // chassis.turnToHeading(0, 1000);
    // matchLoader.extend();
    // intake.move(-127);
    // toptakes(-100, 0);
    // chassis.moveToPoint(37.8, 85, 2000, {.maxSpeed = 50});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(38, 90, 4000, {.maxSpeed = 20, .minSpeed = 20});
    // pros::delay(2400);
    // chassis.cancelMotion();
    // intake.move(0);
    // toptakes(0, 0);
    // // end of 3rd matchloading
    
    // chassis.moveToPoint(38, 81, 1000, {.forwards = false});
    // chassis.turnToHeading(-40, 1000);
    // chassis.moveToPoint(47, 70, 2000, {.forwards = false, .maxSpeed = 80});
    // matchLoader.retract();
    // chassis.turnToHeading(-10, 1000);
    // chassis.moveToPoint(49.5, 8, 4000, {.forwards = false, .maxSpeed = 100});
    // // arrival at bottom right side

    // chassis.turnToHeading(-90, 1000);
    // chassis.moveToPoint(38.5, 8, 1500);
    // chassis.turnToHeading(-180, 1000);
    // chassis.moveToPoint(38.5, 14, 1500, {.forwards = false});
    // pros::delay(200);
    // intake.move(-127);
    // toptakes(127, 114);
    // matchLoader.extend();
    // pros::delay(3000);
    // intake.move(0);
    // toptakes(0, 0);
    // // end of 3rd scoring at long goal bottom right

    // chassis.moveToPoint(38.5, -5, 1500, {.maxSpeed = 50});
    // intake.move(-127);
    // toptakes(-100, 0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(38.5, -12, 4000, {.maxSpeed = 20, .minSpeed = 20});
    // pros::delay(2000);
    // chassis.cancelMotion();
    // intake.move(0);
    // toptakes(0, 0);
    // // end of 4th matchloading bottom right

    // chassis.moveToPoint(38.2, 8, 1500, {.forwards = false, .minSpeed = 80});
    // chassis.moveToPoint(38.2, 16, 1000, {.forwards = false, .maxSpeed = 50});
    // pros::delay(200);
    // intake.move(-127);
    // toptakes(127, 114);
    // pros::delay(3000);
    // intake.move(0);
    // toptakes(0, 0);
    // // end of 4th scoring bottom right

    // chassis.moveToPoint(38.5, 8, 1000);
    // matchLoader.retract();
    // chassis.turnToHeading(-90, 1000, {.maxSpeed = 60});
    // chassis.moveToPoint(3, 8, 2000);
    // intake.move(-127);
    // chassis.turnToHeading(-180, 1000, {.maxSpeed = 60});
    // chassis.moveToPoint(3, -50, 5000, {.minSpeed = 127});

    // parked
    // around 56-60 seconds if it goes well
    // around 80 points hopefully
}

void soloAWP() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 22.5, 1500);
    chassis.turnToHeading(90, 800);
    intake.move(-127);
    toptakes(-60, 0);
    matchLoader.extend();
    chassis.moveToPoint(7.5, 23, 1000);
    pros::delay(1000);
    // matchload

    chassis.moveToPoint(-16, 23, 1500, {.forwards = false});
    pros::delay(500);
    toptakes(127, 114);
    matchLoader.retract();
    pros::delay(2000);
    toptakes(0, 0);
    // first long goal score

    chassis.moveToPoint(-9, 23, 1000);
    chassis.turnToHeading(220, 1000);
    // go to mid

    intake.move(-127);
    toptakes(-60, 0);
    chassis.moveToPoint(-22, 3, 1500);
    pros::delay(700);
    matchLoader.extend();
    chassis.turnToHeading(-175, 700);
    matchLoader.retract();
    intake.move(0);
    // mid ball 1

    chassis.moveToPoint(-21, -31.5, 1500);
    intake.move(-127);
    toptakes(-60, 0);
    pros::delay(800);
    matchLoader.extend();
    chassis.turnToHeading(-220, 800);
    // mid ball 2

    chassis.moveToPoint(-28, -23.5, 1000, {.forwards = false});
    toptakes(-127, -114);
    pros::delay(1500);
    toptakes(0, 0);
    // mid goal score

    chassis.moveToPoint(-4, -50, 1500);
    matchLoader.retract();
    intake.move(0);
    toptakes(0, 0);
}

void leftAuton() {
    chassis.setPose(0, 0, 0);
    intake.move(-127);
    toptakes(-127, 0);
    chassis.moveToPoint(-8.5, 23, 2000, {.maxSpeed = 40});
    pros::delay(1400);
    matchLoader.extend();
    chassis.moveToPoint(-6.5, 19, 800);
    chassis.turnToHeading(-135, 1000);
    matchLoader.retract();
    intake.move(0);
    chassis.moveToPoint(2, 24.5, 1500, {.forwards = false});
    pros::delay(500);
    intake.move(-80);
    toptakes(-127, -114);
    pros::delay(2000);
    intake.move(0);
    toptakes(0, 0);
    chassis.moveToPoint(-26.5, 0, 1500);
    pros::delay(100);
    chassis.turnToPoint(-26.5, -15, 1000);
    matchLoader.extend();
    intake.move(-127);
    toptakes(-60, 0);
    chassis.moveToPoint(-26.5, -8, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(-26.5 , -15.5, 4000, {.maxSpeed = 20, .minSpeed = 20});
    pros::delay(1700);
    chassis.moveToPoint(-27.5, 11, 2000, {.forwards = false, .maxSpeed = 100});
    toptakes(0, 0);
    pros::delay(500);
    toptakes(127,144);
    pros::delay(1600);
    matchLoader.retract();
    intake.move(0);
    toptakes(0,0);
    chassis.moveToPoint(-27.5, 5, 800, {.minSpeed = 127});
    pros::delay(500);
    chassis.moveToPoint(-27.5, 10, 1000, {.forwards = false, .minSpeed = 127});
}

void rightAuton() {
    chassis.setPose(0, 0, 0);
    intake.move(-127);
    chassis.moveToPose(6.5, 19, 20, 2000, {.maxSpeed = 100});
    pros::delay(800);
    matchLoader.extend();
    chassis.moveToPoint(5.5, 17, 1000, {.forwards = false});
    intake.move(0);
    matchLoader.retract();
    chassis.turnToHeading(-40, 1000);
    chassis.moveToPoint(-3, 25.5, 2000);
    pros::delay(1000);
    intake.move(80);
    pros::delay(800);
    intake.move(0);
    chassis.moveToPoint(26, 1, 2000, {.forwards = false});
    chassis.turnToPoint(26, -15, 1000);
    pros::delay(1000);
    matchLoader.extend();
    pros::delay(500);
    intake.move(-127);
    chassis.moveToPoint(26, -15, 1500, {.maxSpeed = 100});
    pros::delay(1700);
    chassis.moveToPoint(26, 17, 2000, {.forwards = false, .maxSpeed = 100});
    pros::delay(800);
    toptakes(127, 114);
    pros::delay(2000);
    toptakes(0, 0);
    intake.move(0);
    matchLoader.retract();
    chassis.moveToPoint(26, 10, 800, {.minSpeed = 127});
    pros::delay(500);
    chassis.moveToPoint(26, 17, 1000, {.forwards = false, .minSpeed = 127});
}

/**
 * Runs during auto
 */
void autonomous() {
    //robot starts with the back touching the front left corner of the parking space
    //robot starts facing forward, not at an angle
    // leftAuton();
    // rightAuton();
    // soloAWP();
    skillsAuton2();
    // aleftMotors.move(50);
    // arightMotors.move(50);
    // pros::delay(100);
    // aleftMotors.move(0);
    // arightMotors.move(0);
}
/**
 * Runs in driver control
 */

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // // move the chassis with curvature drive
	    // aleftMotors.move(leftY);
        // arightMotors.move(rightY);
        int power = controller.get_analog( pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog( pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int Left = power + turn;
        int Right = power - turn;

        aleftMotors.move(Left);
        arightMotors.move(Right);

        // Intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakesf(127, 127);
            if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                toptake.move(127);
            }
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakesf(-127, -127);
            if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                toptake.move(-127);
            }
        } 
        else {
            intakesf(0, 0);
            if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                toptake.move(0);
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (!hood.is_extended()) {
                hood.extend();
            }
            intakes(127, 127, 127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (hood.is_extended()) {
                hood.retract();
            }
            intakes(127, 127, -63);
        } 
        else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (hood.is_extended()) {
                hood.retract();
            }
            intakes(0, 0, 0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if (matchLoader.is_extended() == true) {
                matchLoader.retract();
                pros::delay(500);
            } else {
                matchLoader.extend();
                pros::delay(500);
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (descore.is_extended() == true) {
                descore.retract();
                pros::delay(200);
            } else {
                descore.extend();
                pros::delay(200);
            }
        }

        // delay to save resources
        pros::delay(10);
    }
        
        //Arcade Controls:

        
        //int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		//int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		//leftMotors.move(dir - turn);                      // Sets left motor voltage
		//rightMotors.move(dir + turn);                     // Sets right motor voltage
		//pros::delay(20);
}