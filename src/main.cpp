#include "main.h"
#include "pros/motors.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::set_text(1, "Inactive");
}

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
void autonomous() {}

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
	pros::lcd::set_text(4, "teleop started");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({19});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-18});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::MotorGroup flywheel_intake_mg({-7});
	pros::MotorGroup flywheel_outtake_mg({9});
	pros::Motor flywheel_out (9);
	// Set up ADI Line follower with 3 wire ports G and H
	pros::adi::Port line_follower_left('H');
	pros::adi::Port line_follower_right('G');

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick

		pros::lcd::print(1, "dir: %d", dir);     // Prints the forward/backward value to the LCD
		pros::lcd::print(2, "turn: %d", turn);   // Prints the turn value to the LCD

		int last_line_value = 0; // value to store the last line sensor value: 0 = left, 1 = right
		int WHITE_THRESHOLD = 2000; // threshold val to see if the sensor is on white or black

		// flywheel_out.move_voltage(127); // always run the outtake flywheel at full speed
		// left_mg.move(dir - turn);    // Sets left motor voltage
		// right_mg.move(dir + turn);
		flywheel_intake_mg.move(master.get_analog(ANALOG_RIGHT_Y));  // intake on the amt the down stick
		flywheel_outtake_mg.move(master.get_analog(ANALOG_LEFT_X));   // run outtake based on how negative the right stick y is
		pros::lcd::print(3, "running either intake? : %d", master.get_digital(DIGITAL_DOWN) || master.get_digital(DIGITAL_UP));
		/* // Autonomous Teleop:
		// Sensors have a gap barely the width of the line, so the robot will be off track when neither line sensor detects the line
		// Use the left and right line sensors to follow the line
		// If only one line follower detects, log which one
		// Give the robot 30 seconds to get to the intake place
		pros::lcd::print(5, "LF Left: %d", line_follower_left.get_value());
		pros::lcd::print(6, "LF Right: %d", line_follower_right.get_value());
		// the above should be between 0 and 4095

		while (pros::millis() < 30000) {
			// follow the line for 30 seconds
			if (line_follower_left.get_value() < WHITE_THRESHOLD && line_follower_right.get_value() < WHITE_THRESHOLD) {
				// both online normal forward
				left_mg.move(100);
				right_mg.move(100);
			} else if (line_follower_left.get_value() < WHITE_THRESHOLD) {
				// only left on line then slight turn left		
				left_mg.move(50);
				right_mg.move(100);
				last_line_value = 0;
			} else if (line_follower_right.get_value() < WHITE_THRESHOLD) {
				// only right on line then slight turn right
				left_mg.move(100);
				right_mg.move(50);
				last_line_value = 1;
			} else {
				// both off line then turn left/right (depending on which sensor was last on the line) until at least one sensor on line
				if (last_line_value == 0) {
					// last on left so turn left
					left_mg.move(-50);
					right_mg.move(50);
				} else {
					// last on right so turn right
					left_mg.move(50);
					right_mg.move(-50);
				}
			}
		}

		// now try to send balls in box to goal:
		// drive forward while spinning up flywheel_intake (for 5 seconds)
		// then drive backwards for 10 seconds with both flywheel motor groups off
		// pray its in a good position and just run flywheel_outtake_mg for 5 seconds and hope
		left_mg.move(100);
		right_mg.move(100);
		flywheel_intake_mg.move(127);
		pros::delay(5000);
		left_mg.move(-100);
		right_mg.move(-100);
		flywheel_intake_mg.move(0);
		flywheel_outtake_mg.move(0);
		pros::delay(10000);
		left_mg.move(0);
		right_mg.move(0);
		flywheel_outtake_mg.move(127);
		pros::delay(5000); */

		// Super scuffed auton:
		// Drive forward for 10 seconds with flywheel_intake_mg on max for the last 5 seconds
		// then drive backwards for 10 seconds with both flywheel motor groups off
		// pray its in a good position and just run flywheel_outtake_mg for 5 seconds
		
		// wait for digital left button to be pressed to start the auton
		
		if (master.get_digital(DIGITAL_LEFT)) {
			while (true) {
				pros::delay(20);
				left_mg.move(80);
				right_mg.move(127);
				pros::delay(5000);
				flywheel_intake_mg.move(127);
				pros::delay(5000);
				left_mg.move(-80);
				right_mg.move(-127);
				flywheel_intake_mg.move(0);
				flywheel_outtake_mg.move(0);
				pros::delay(10000);
				left_mg.move(0);
				right_mg.move(0);
				flywheel_intake_mg.move(127);
				flywheel_outtake_mg.move(127);
				pros::delay(5000);
			}
		}
	}
}