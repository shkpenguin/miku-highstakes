#include "main.h"
#include "robot-config.h"
#include "api.h"
#include "macros.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "pros/adi.hpp"
#include "utils/pose.h"
#include "chassis/mcl.h"
#include "selector.h"
#include "routes.h"
#include "debug.h"
#include "system.h"

void initialize()
{
	if(pros::competition::is_field_control()) initSelector();
	else selectedAuton = Auton(RED, "Test", test, Pose(48, -48, 0));

	initSystem();

	pros::Task intakeTask(intakeControl);
	pros::Task lbTask(lbControl);

	pros::Task debugTask(controllerDisplay);
}

void disabled()
{
}

void competition_initialize()
{
}

void autonomous()
{
	selectedAuton.func();
}

void opcontrol()
{
}