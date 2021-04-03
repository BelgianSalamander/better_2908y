#include "main.h"
using namespace okapi;

Motor left_mtr_front(16, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_mtr_back(17, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_front(12, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_back(11, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor lift(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor input_right(9, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor input_left(19, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

auto chassis = ChassisControllerBuilder()
															.withMotors({left_mtr_front, left_mtr_back}, {right_mtr_front, right_mtr_back})
//															.withMotors({left_mtr_front, left_mtr_back}, {right_mtr_front, right_mtr_back})
															.withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
															.withOdometry()
															.build();

Controller controller(ControllerId::master);
ControllerButton lift_up(ControllerDigital::up);
ControllerButton lift_down(ControllerDigital::down);
ControllerButton out(ControllerDigital::R1);
ControllerButton in(ControllerDigital::R2);
ControllerButton intake_out(ControllerDigital::L1);
ControllerButton intake_in(ControllerDigital::L2);

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
	while (true) {
		chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::leftX));
		int lift_thing = (lift_up.isPressed() - lift_down.isPressed()) * 12000;
		lift.moveVoltage(lift_thing);


		int intake_thing = (intake_in.isPressed() - intake_out.isPressed()) * 12000;
		input_right.moveVoltage(intake_thing);
		input_left.moveVoltage(intake_thing);
	}
}
