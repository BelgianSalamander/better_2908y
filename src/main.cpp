#include "main.h"
#include <vector>
#include <string>
#include <sstream>

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

ControllerButton left(ControllerDigital::left);
ControllerButton right(ControllerDigital::right);
ControllerButton A(ControllerDigital::A);

bool auton[3] = {false, true, false};
bool not_started = true;
short toggle_index = 1;

void moveDistance(okapi::QLength distance, int timeout) {
     chassis->moveDistanceAsync(distance); // move to the target asynchronously (without waiting)
     long endTime = pros::millis() + timeout; // determine when to stop if it hasnt settled
     while(!chassis->isSettled()) { // loop if not settled
          if(pros::millis() >= endTime) { // if not settled and time has ran out
             chassis->stop();
               break; // break the loop and continue with autonomous
          }
     }
}

void turnAngle(okapi::QAngle angle, int timeout = 2000) {
     chassis->turnAngle(angle); // move to the target asynchronously (without waiting)
     long endTime = pros::millis() + timeout; // determine when to stop if it hasnt settled
     while(!chassis->isSettled()) { // loop if not settled
          if(pros::millis() >= endTime) { // if not settled and time has ran out
             chassis->stop();
               break; // break the loop and continue with autonomous
          }
     }
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


void update_auton_description(){
	std::vector<std::string> text;
	if(auton[0]){
		text.push_back("left");
	}
	if(auton[1]){
		text.push_back("middle");
	}
	if(auton[2]){
		text.push_back("right");
	}

	std::string msg1 = "Auton will score in ";
	std::string msg2 = "";

	switch(text.size()){
		case(3):
			msg1 = msg1 + "the left,";
			msg2 = "middle and right goals!";
			break;
		case(2):
			msg1 = msg1 + "the " + text[0];
			msg2 = "and  " + text[1] + " goals!";
			break;
		case(1):
			msg1 = msg1 + "the " + text[0];
			msg2 = "goal!";
			break;
		case(0):
			msg1 = msg1 + "no goals! :(";
	}

	pros::lcd::set_text(0, msg1);
	pros::lcd::set_text(1, msg2);

	controller.clearLine(0);
	pros::delay(50);
	controller.clearLine(1);
	pros::delay(50);
	std::ostringstream ss;
	ss << auton[0] << "   " << auton[1] << "   " << auton[2];
	controller.setText(0, 0, ss.str());
	std::string arrow_ting = "         ";
	arrow_ting[toggle_index * 4] = '^';
	pros::delay(50);
	std::ostringstream ss2;
	controller.setText(1, 0, arrow_ting);
}

void left_right_press_ting(){
	while(not_started){
		if(left.changedToPressed()){
			printf("Left");
			toggle_index = (toggle_index - 1) % 3;
			update_auton_description();
		}else if(right.changedToPressed()){
			toggle_index = (toggle_index + 1) % 3;
			update_auton_description();
		}else if(A.changedToPressed()){
			auton[toggle_index] = !auton[toggle_index];
			update_auton_description();
		}
		pros::delay(50);
	}
}

void on_left_button(){
	auton[0] = !auton[0];
	update_auton_description();
}

void on_center_button() {
	auton[1] = !auton[1];
	update_auton_description();
}

void on_right_button(){
	auton[2] = !auton[2];
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

	//pros::lcd::set_text(0, "Poggers");
	printf("%d",pros::lcd::is_initialized());
	update_auton_description();

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
	pros::lcd::register_btn0_cb(on_left_button);

	pros::Task button_detect(left_right_press_ting);

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
	switch(auton[0] + auton[1] * 2 + auton[2] * 4){
		case 0:   //Nothing selected
			break;
		case 1:   //Left goal selected
			break;
		case 2:   //Middle goal selected
			break;
		case 3:   //Left and middle goal selected
			break;
		case 4:   //Right goal selected
			break;
		case 5:   //Left and right goal selected
			break;
		case 6:   //Middle and right goals selected
			break;
		case 7:   //All goals selected
			break;
	}
}

void opcontrol() {
	not_started = false;
	int start = pros::millis();

	int rumble[6] = {95000, 100000, 101000, 102000, 103000, 104000};
	int index = 0;

	auto xModel = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
	while (true) {
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
		pros::delay(20);
	}



}
