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

int display = 0;
int options = 9;

void on_center_button()
{
}

void initialize()
{

	if(pros::competition::is_field_control()) initSelector();
	else selectedAuton = Auton(RED, "Test", test, Pose(48, -48, 0));

	display = 0;

	vertical.reset_position();
	hori.reset_position();

	intake.tare_position();
	lbRot.reset_position();
	// lbRot.set_position(READY); //fix later

	optical.set_integration_time(5);

	imu.reset();
	while (imu.is_calibrating())
	{
		pros::delay(10);
	}

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
	sortColor = static_cast<Color>(-1 * (selectedAuton.color));
	initOdom(selectedAuton.start);
	initParticles();
	selectedAuton.func();
}

void opcontrol()
{
	miku.disableCorrection();
	pros::Task userTask(driveControl);
}