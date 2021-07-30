#include "main.h"

using namespace okapi;

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
    .withMotors(leftMotorGroup, rightMotorGroup)
    .withDimensions(AbstractMotor::gearset::green, {{4.125, 10}, imev5GreenTPR})
    .build();


void user_drive()
{
  while (true)
  {
    int rightJoystickValue = controller.getAnalog(okapi::ControllerAnalog::rightY);
    int leftJoystickValue = controller.getAnalog(okapi::ControllerAnalog::leftY);
    drive -> getModel() -> tank(leftJoystickValue, rightJoystickValue);
    pros::delay(10);
  }
}
