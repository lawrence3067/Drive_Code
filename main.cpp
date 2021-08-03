#include "main.h"

using namespace okapi;
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

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({1, 2}, {-6, -8})
  .withDimensions(AbstractMotor::gearset::green, {{4.125_in, 10_in}, imev5GreenTPR})
  .build();


okapi::Controller controller;
okapi::Motor four_bar_lift(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::rotations);
okapi::Motor chain_bar(9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::rotations);

struct PID
{
	float kP;
	float kI;
	float kD;
	float integral;
	float derivative;
	float error;
	float prev_error;
	float speed;
	float target;
	float sensor_value;
};

typedef struct PID pid;

pid C_B;


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
void opcontrol()
{
	float setpoint = chain_bar.tarePosition();
	while (1)
	{
		//float leftJoyValue = controller.getAnalog(ControllerAnalog::leftY);
		//float rightJoyValue = controller.getAnalog(ControllerAnalog::rightY);
		drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY),
																controller.getAnalog(ControllerAnalog::rightY), 5);

		//Four Bar Buttons
		if (controller.getDigital(ControllerDigital::L1) == 1)
		{
			four_bar_lift.moveVoltage(8000);
		}
		if (controller.getDigital(ControllerDigital::L1) == 0 and controller.getDigital(ControllerDigital::L2) == 0)
		{
			four_bar_lift.moveVoltage(0);
		}
		if (controller.getDigital(ControllerDigital::L2) == 1)
		{
			four_bar_lift.moveVoltage(-8000);
		}


		//Chain Bar Buttons
		if (controller.getDigital(ControllerDigital::R1) == 1)
		{
			chain_bar.moveVoltage(8000);
			setpoint = chain_bar.getPosition();
		}
		else if (controller.getDigital(ControllerDigital::R2) == 1)
		{
			chain_bar.moveVoltage(-8000);
			setpoint = chain_bar.getPosition();
		}
		else if (controller.getDigital(ControllerDigital::R2) == 0 and (controller.getDigital(ControllerDigital::R1) == 0))
		{
			C_B.kP = 0;
			C_B.kI = 0;
			C_B.kD = 0;
			C_B.target = setpoint;
			C_B.error = setpoint - chain_bar.getPosition();
		}
		pros::delay(20);
	}
}
