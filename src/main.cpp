#include "main.h"
#include <vector>
#include <string>

Motor left_mtr_front(18, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_mtr_back(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_front(17, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_mtr_back(8, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor lift_bottom(6, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor lift_top(19, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor intake_left(7, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor intake_right(9, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

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

bool auton_left = false;
bool auton_mid = true;
bool auton_right = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void update_auton_description(){
	std::vector<std::string> text;
	if(auton_left){
		text.push_back("left");
	}
	if(auton_mid){
		text.push_back("middle");
	}
	if(auton_right){
		text.push_back("right");
	}

	std::string msg = "Auton will score in ";

	switch(text.size()){
		case(3):
			msg = msg + "the left, middle and right goals!";
			break;
		case(2):
			msg = msg + "the " + text[0] + " and  " + text[1] + " goals!";
			break;
		case(1):
			msg = msg + "the " + text[0] + " goal!";
			break;
		case(0):
			msg = msg + "no goals! :(";
	}

	pros::lcd::set_text(2, msg);
}

void on_left_button(){
	auton_left = !auton_left;
	update_auton_description();
}

void on_center_button() {
	auton_mid = !auton_mid;
	update_auton_description();
}

void on_right_button(){
	auton_right = !auton_right;
	update_auton_description();
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
	update_auton_description();

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
	pros::lcd::register_btn0_cb(on_left_button);
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
	//pros::lcd::set_text(2, "Pogchamp");
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
