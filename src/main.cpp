#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
pros::Motor intake(14);
pros::Motor toptake(1);

// pnuematics
pros::adi::Pneumatics matchLoader('A', false);

// motor groups
pros::MotorGroup leftMotors({-18, -19, -20}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({11, 12, 13}, pros::MotorGearset::blue);

pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(1);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
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

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void skillsAuton() {
    chassis.turnToPoint(10, 20, 1000);
    intake.move(127);
    chassis.moveToPoint(10, 20, 5000);
    intake.move(0);
    chassis.turnToPoint(20, 10, 1000);
    // put down ml
    chassis.moveToPoint(20, 10, 5000);
    chassis.turnToPoint(20, 0, 1000);
    intake.move(127);
    chassis.moveToPoint(20, 0, 5000);
    pros::delay(1000);
    chassis.moveToPoint(20, 5, 1000, {.forwards = false});
    chassis.moveToPoint(20, 0, 1000);
    pros::delay(1000);
    chassis.moveToPoint(20, 15, 5000, {.forwards = false});
    toptake.move(127);
    pros::delay(1000);
    toptake.move(0);
    intake.move(0);
    chassis.moveToPoint(20, 10, 1000);
    // put up match loader
    chassis.moveToPoint(30, 10, 1000);
    chassis.turnToPoint(30, 50, 1000);
    // put down match loader
    chassis.moveToPoint(30, 50, 5000);
    chassis.moveToPoint(20, 60, 5000);
    chassis.turnToPoint(20, 70, 1000);
    intake.move(127);
    chassis.moveToPoint(20, 70, 5000);
    pros::delay(1000);
    chassis.moveToPoint(20, 50, 5000, {.forwards = false});
    toptake.move(127);
    pros::delay(1000);
    // put up match loader
    toptake.move(0);
    intake.move(0);
    chassis.moveToPoint(20, 60, 5000);
    chassis.moveToPoint(-50, 60, 5000);
    chassis.turnToPoint(-50, 70, 1000);
    // put down match loader
    intake.move(127);
    chassis.moveToPoint(-50, 70, 5000);
    pros::delay(1000);
    chassis.moveToPoint(-50, 65, 1000, {.forwards = false});
    chassis.moveToPoint(-50, 70, 1000);
    pros::delay(1000);
    chassis.moveToPoint(-50, 50, 5000, {.forwards = false});
    //put up match loader
    toptake.move(127);
    pros::delay(1000);
    toptake.move(0);
    intake.move(0);
    chassis.moveToPoint(-50, 60, 5000);
    chassis.turnToPoint(-40, 50, 1000);
    intake.move(127);
    chassis.moveToPoint(-48, 58, 5000, {.maxSpeed = 30});
    chassis.moveToPoint(-48, 20, 5000);
    chassis.turnToPoint(-40, 15, 1000);
    // put down match loader
    chassis.moveToPoint(-40, 15, 5000);
    chassis.moveToPoint(-40, 10, 5000);
    pros::delay(1000);
    chassis.moveToPoint(-40, 12, 1000, {.forwards = false});
    chassis.moveToPoint(-40, 10, 1000);
    chassis.moveToPoint(-40, 20, 5000, {.forwards = false});
    // put up match loader
    toptake.move(127);
    pros::delay(1000);
    toptake.move(0);
    intake.move(0);
    chassis.moveToPoint(-20, 0, 5000);
    chassis.moveToPoint(-10, 0, 5000);
}

/**
 * Runs during auto
 */
void autonomous() {
    leftMotors.move(65);
    rightMotors.move(65);
    pros::delay(1150);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(10);
    leftMotors.move(-45);
    rightMotors.move(45);
    pros::delay(100);
    leftMotors.move(0);
    rightMotors.move(0);
    intake.move(-127);
    pros::delay(300);
    intake.move(0);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        //int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        //int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
	    //leftMotors.move(leftY);
        //rightMotors.move(rightY);
        int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		leftMotors.move(dir - turn);                      // Sets left motor voltage
		rightMotors.move(dir + turn);                     // Sets right motor voltage
        // Intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
        } 
        else {
            intake.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            toptake.move(113);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            toptake.move(-113);
        } 
        else {
            toptake.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            if (matchLoader.is_extended() == true) {
                matchLoader.retract();
            } else {
                matchLoader.extend();
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