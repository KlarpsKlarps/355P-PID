#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp"
pros::MotorGroup left({-1,-2,-3});
pros::MotorGroup right({13,12,18});
pros::Motor babybrown(5);
lemlib::Drivetrain drivetrain(&left, // left motor group
                              &right, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
pros::Rotation rotleft(8);
pros::Rotation rotright(-6);
pros::Rotation ladybrown(15);
lemlib::TrackingWheel vert1(&rotleft, lemlib::Omniwheel::NEW_275, -7.5);
// vertical tracking wheel
lemlib::TrackingWheel vert2(&rotright, lemlib::Omniwheel::NEW_275, 7.5);
lemlib::OdomSensors sensors(&vert1, // vertical tracking wheel 1, set to null
                            &vert2, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,-18.7,0,3000,{.forwards=false});
    chassis.turnToHeading(-90, 3000);
    chassis.waitUntilDone();
    chassis.moveToPose(-8.25, -16, -90, 3000,{.forwards=true});
    chassis.turnToHeading(-90, 3000);
	pros::delay(1000);
	babybrown.move(-70);
	pros::delay(400);
	babybrown.brake();
	pros::delay(700);
	babybrown.move(70);
	pros::delay(400);
	babybrown.brake();
	pros::delay(1000);
    


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
        // print measurements from the adi encoder
        pros::lcd::print(1, "rotlepht: %i", rotleft.get_position());
        // print measurements from the rotation sensor
        pros::lcd::print(5, "rotright: %i", rotright.get_position());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    

	}
}