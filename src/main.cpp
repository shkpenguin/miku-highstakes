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

pros::Task* intakeTask = nullptr;
pros::Task* lbTask = nullptr;
pros::Task* debugTask = nullptr;

void initialize()
{
	if(pros::competition::is_field_control()) initSelector();
	else selectedAuton = Auton(RED, "Test", test, Pose(48, -48, 0));

	initSystem();

	intakeTask = new pros::Task(intakeControl);
	lbTask = new pros::Task(lbControl);
	// debugTask = new pros::Task(controllerDisplay);
}

void disabled()
{
}

void competition_initialize()
{
}

void autonomous()
{
	initAuto(selectedAuton.start);
	pros::Task autoTask(autoSystem);
	selectedAuton.func();
}

void opcontrol()
{
	if(intakeTask->get_state() != pros::E_TASK_STATE_RUNNING) intakeTask = new pros::Task(intakeControl);
	if(lbTask->get_state() != pros::E_TASK_STATE_RUNNING) lbTask = new pros::Task(lbControl);
	pros::Task driveTask(driveControl);
}