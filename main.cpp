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


//Drivetrain Motor initialization
Motor rightBack(8, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightFront(6, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftFront(2, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftBack(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

//Controller Initialization
Controller controller;

//Misc Motor Initialization
Motor four_bar_lift(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor chain_bar(9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

//Variable Initialization
int A_buttonPress = 0;
int lift_macro_level = 0;
double chain_bar_speed = 0;
double chain_bar_PID_speed = 0;
double four_bar_speed = 0;
double four_bar_PID_speed = 0;

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

pid CB; 								//Chain bar PID
pid FB; 								//Four bar PID
pid left_drive_PID; 		//PID for left side of drivetrain
pid right_drive_PID;		//PID for right side of drivetrain


//Drivetrain Controller Initialization
std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftBack, leftFront}, {rightFront, rightBack}) 												//MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})		  //Gearset(rpm) and wheel dimensions
  .build();


double chain_bar_PID(double chain_bar_set) 							//PID for chain bar
{
	CB.kP = 0.3;
	CB.kI = 0.0003;
	CB.kD = 0.01;
	CB.target = chain_bar_set;
	pros::lcd::set_text(4, std::to_string(chain_bar.getPosition()));
	CB.error = CB.target - chain_bar.getPosition();
	CB.derivative = CB.error - CB.prev_error;
	CB.integral += CB.error;
	CB.prev_error = CB.error;
	CB.speed = CB.error * CB.kP + CB.integral * CB.kI + CB.derivative * CB.kD;
	pros::lcd::set_text(2, std::to_string(CB.speed));
	return CB.speed;
	pros::delay(10);
}

double four_bar_PID(double four_bar_setpoint) 					//PID for four bar
{
	FB.kP = 0; //need tuning
	FB.kI = 0; //need tuning
	FB.kD = 0; //need tuning
	FB.target = four_bar_setpoint;
	FB.error = FB.target - four_bar_lift.getPosition();
	FB.derivative = FB.error - FB.prev_error;
	FB.integral += FB.error;
	FB.prev_error = FB.error;
	FB.speed = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;
	//pros::lcd::set_text(3, std::to_string(FB.speed));
	return FB.speed;
	pros::delay(10);
}

void movement_PID(double left_distance, double right_distance)
{
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	double left_target = left_distance * (360 / (2 * 3.1415 * (4 / 2)));				// calculates left side motors target distance in wheel degrees
	double right_target = right_distance * (360 / (2 * 3.1415 * (4 / 2)));			// calculates right side motors target distance in wheel degrees

	//PID constants
	left_drive_PID.kP = 0.001575;
	left_drive_PID.kI = 0.0005;
	left_drive_PID.kD = 0.00015;

	right_drive_PID.kP = 0.001575;
	right_drive_PID.kI = 0.0005;
	right_drive_PID.kD = 0.00015;

	//initializes left and right side drivetrain PID controllers
	auto left_pid_controller = IterativeControllerFactory::posPID(left_drive_PID.kP,
																																left_drive_PID.kI,
																																left_drive_PID.kD);

	auto right_pid_controller = IterativeControllerFactory::posPID(right_drive_PID.kP,
																																 right_drive_PID.kI,
																																 right_drive_PID.kD);

	drive -> getModel() -> resetSensors(); 																								//reset sensor values before use

	while (true)
	{
		//pros::lcd::set_text(3, std::to_string(drive -> getModel() -> getSensorVals()[0]));
		left_drive_PID.error = left_target - drive -> getModel() -> getSensorVals()[0];
		left_drive_PID.speed = left_pid_controller.step(left_drive_PID.error); 							//returns speed for left side
		pros::lcd::set_text(5, std::to_string(left_drive_PID.error));

		right_drive_PID.error = right_target - drive -> getModel() -> getSensorVals()[1];
		right_drive_PID.speed = right_pid_controller.step(right_drive_PID.error); 					//returns speed for right side
		pros::lcd::set_text(6, std::to_string(right_drive_PID.error));

		pros::lcd::set_text(1, std::to_string(leftBack.getPosition()));
		pros::lcd::set_text(2, std::to_string(leftFront.getPosition()));
		pros::lcd::set_text(3, std::to_string(rightBack.getPosition()));
		pros::lcd::set_text(4, std::to_string(rightFront.getPosition()));

		drive -> getModel() -> tank(-left_drive_PID.speed, -right_drive_PID.speed);

		pros::delay(20);
		if (abs(left_drive_PID.error) < 0.1 and abs(right_drive_PID.error) < 0.1)
		{
			break;
		}
	}

	drive -> getModel() -> tank(0, 0); 								//brakes drivetrain right after PId movement

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
void autonomous()
{
	//movement_PID(70, 70);
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
void opcontrol()
{
	//movement_PID(40, 40);
	//movement_PID(24.5, -24.5);
	//movement_PID(40, 40);
	double chain_bar_setpoint = chain_bar.getPosition();    //Marks position of chain bar
	double four_bar_setpoint = four_bar_lift.getPosition(); //Marks position of four bar

	while (true)
	{
		//Main drivetrain code
		drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY),
																controller.getAnalog(ControllerAnalog::rightY));


		if (controller.getDigital(ControllerDigital::A) == 1)
		{
			pros::delay(1000);
			if (A_buttonPress > 1) 														 //resets count
			{
				A_buttonPress = 1;
			}
			else
			{
				A_buttonPress = A_buttonPress + 1;
			}
			pros::lcd::set_text(2, std::to_string(A_buttonPress));
			pros::delay(1000); 																//delay so won't keep adding to count
		}

		if (A_buttonPress == 1) 														//Macros for lift
		{
			if (controller.getDigital(ControllerDigital::R1) == 1) //sets 4 different macros
			{

				if (lift_macro_level > 30)
				{
					lift_macro_level = 10;
				}
				else
				{
					lift_macro_level += 10;
				}
				//pros::lcd::set_text(5, std::to_string(R1_buttonPress));
				pros::delay(500);
			}

			if (controller.getDigital(ControllerDigital::R2) == 1)
			{
				if (lift_macro_level < 20)
				{
					lift_macro_level = 40;
				}
				else
				{
					lift_macro_level -= 10;
				}
				pros::delay(500);
			}

			if (lift_macro_level == 10) 												// first macro (first height)
			{
				chain_bar_PID_speed = chain_bar_PID(-200);
				chain_bar.moveVelocity(chain_bar_PID_speed);

				four_bar_PID_speed = four_bar_PID(0);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 20)										//second macro (second height)
			{
				chain_bar_PID_speed = chain_bar_PID(-500);
				chain_bar.moveVelocity(chain_bar_PID_speed);

				four_bar_PID_speed = four_bar_PID(0);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 30)										//thrid macro (third height)
			{
				chain_bar_PID_speed = chain_bar_PID(-800);
				chain_bar.moveVelocity(chain_bar_PID_speed);

				four_bar_PID_speed = four_bar_PID(0);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 40)										//fourth macro (sets lift back at starting position)
			{
				chain_bar_PID_speed = chain_bar_PID(0);
				chain_bar.moveVelocity(chain_bar_PID_speed);

				four_bar_PID_speed = four_bar_PID(0);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
		}

		else if (A_buttonPress == 2)												//user control lift
		{
			if (controller.getDigital(ControllerDigital::L1) == 1)
			{
				four_bar_lift.moveVelocity(100);								//Moves four bar up when L1 is pressed
				four_bar_setpoint = four_bar_lift.getPosition();
			}
			else if (controller.getDigital(ControllerDigital::L2) == 1)
			{
				four_bar_lift.moveVelocity(-100);								//Moves four bar down when L2 is pressed
				four_bar_setpoint = four_bar_lift.getPosition();
			}
			else if((controller.getDigital(ControllerDigital::L1) == 0) and controller.getDigital(ControllerDigital::L2) == 0)
			{
				four_bar_speed = four_bar_PID(four_bar_setpoint);
				four_bar_lift.moveVelocity(four_bar_speed);
			}


			//Chain Bar Buttons
			if (controller.getDigital(ControllerDigital::R2) == 1)
			{
				if (chain_bar.getPosition() > 0)								//Stops chain bar from going under starting position
				{
					chain_bar.moveVelocity(0);
					chain_bar_setpoint = chain_bar.getPosition();
				}
				else
				{
					chain_bar.moveVelocity(75);										//Moves chain bar down when R2 is pressed
					chain_bar_setpoint = chain_bar.getPosition();
				}
			}

			else if (controller.getDigital(ControllerDigital::R1) == 1)
			{
				if (chain_bar.getPosition() < -1360)						//Stops chain bar from going over maximum position
				{
					chain_bar.moveVelocity(0);
					chain_bar_setpoint = chain_bar.getPosition();
				}
				else
				{
					chain_bar.moveVelocity(-75);									//Moves chain bar up when R1 is pressed
					chain_bar_setpoint = chain_bar.getPosition();
				}
			}
			else if (controller.getDigital(ControllerDigital::R2) == 0 and (controller.getDigital(ControllerDigital::R1) == 0))
			{
				chain_bar_speed = chain_bar_PID(chain_bar_setpoint);
				chain_bar.moveVelocity(chain_bar_speed);				//Keeps chain bar in the air when nothing is pressed
			}
			//pros::lcd::set_text(4, std::to_string(chain_bar.getPosition()));
		}

		pros::delay(20);

	}
}
//https://github.com/ananthgoyal/TippingPoint/tree/master/src
