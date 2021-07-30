#include "main.h"

using namespace okapi;

//Motor Groups
okapi::MotorGroup leftMotorGroup({1, 3});
okapi::MotorGroup rightMotorGroup({-2, -4});

//Functions
void user_drive();

//Controller
extern Controller controller;
