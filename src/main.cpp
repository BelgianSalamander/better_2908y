#include "main.h"

Motor left_mtr_front(1, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_mtr_back(6, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_front(11, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_back(16, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor lift_bottom(15, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor lift_top(3, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor intake_left(20, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor intake_right(10, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

auto chassis = ChassisControllerBuilder()
															.withMotors(left_mtr_front, right_mtr_front, right_mtr_back, left_mtr_back)
//															.withMotors({left_mtr_front, left_mtr_back}, {right_mtr_front, right_mtr_back})
															.withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
															.withOdometry()
															.build();

Controller controller(ControllerId::master);
ControllerButton lift_up(ControllerDigital::R1);
ControllerButton lift_down(ControllerDigital::R2);
ControllerButton poop(ControllerDigital::A);

ControllerButton intake_in(ControllerDigital::L1);
ControllerButton intake_out(ControllerDigital::L2);

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
	printf("initialized screen");
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	//pros::lcd::register_btn2_rb(on_right_button);
	//pros::lcd::register_btn3_lb(on_left_button);
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
void competition_initialize() {

}

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


	lift_top.moveVoltage(12000);
	lift_bottom.moveVoltage(-12000);
	pros::delay(2000);
	lift_top.moveVoltage(0);
	lift_bottom.moveVoltage(0);
	//chassis->moveDistance(-2_ft);
	//chassis->turnAngle(-250_deg);
}

void opcontrol() {
	pros::lcd::set_text(2, "Pogchamp");
	int start = pros::millis();

	int rumble[6] = {95000, 100000, 101000, 102000, 103000, 104000};
	int index = 0;

	auto xModel = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
											 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
											 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		xModel->xArcade(
			controller.getAnalog(ControllerAnalog::leftX),
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::rightX)
		);

		int lift_pow = 12000 * (lift_up.isPressed() - lift_down.isPressed());
		int poop_sign = lift_up.isPressed() - lift_down.isPressed();
		if(poop.isPressed()){
			lift_top.moveVoltage(-1 * poop_sign * 12000);
			lift_bottom.moveVoltage(poop_sign * 12000);
		}else{
			lift_top.moveVoltage(lift_pow);
			lift_bottom.moveVoltage(lift_pow);
		}
		if(poop_sign == 0){
			lift_top.moveVoltage(0);
			lift_bottom.moveVoltage(0);
		}

		int intake_thing = 12000 * (intake_in.isPressed() - intake_out.isPressed());
		intake_left.moveVoltage(intake_thing);
		intake_right.moveVoltage(intake_thing);
	  int time_ =  pros::millis();
		if(time_ - start >= rumble[index]){
			index++;
			controller.rumble("-");
		}
	}

}
