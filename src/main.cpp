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

std::string error_msg = "";
int display = 0;
int options = 9;

void on_center_button()
{
}

void initialize()
{

	// initSelector();
	selectedAuton = Auton(BLUE, "Blue 6 Ring", blueRing, Pose(11, -52.5, 213));

	display = 0;

	vertical.reset_position();
	hori.reset_position();

	intake.tare_position();
	lbRot.reset_position();
	lbRot.set_position(READY);//fix later

	optical.set_integration_time(5);

	imu.reset();
	while (imu.is_calibrating())
	{
		pros::delay(10);
	}

	pros::Task intakeTask(intakeControl);
	pros::Task lbTask(lbControl);

	pros::Task controllerDisplay = pros::Task([]
											  {
		while (true) {
			std::vector<float> dist = estimateDistance(Point(getPose().x, getPose().y));
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				display = (display + 1) % options;
			}
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				display = (display - 1 + options) % options;
			}
			if(error_msg == "") {
				switch (display) {
					case 0:
						master.set_text(0, 0, _1f(getPose().x)
						+ " " + _1f(getPose().y)
						+ " " + _1f(getPose().theta));
						pros::delay(100);
						break;
					case 1: 
						master.set_text(0, 0, _2f(getPose().x) 
						+ " " + _2f(getPose().y)
						+ " " + _2f(getPose(true).theta));
						pros::delay(100);
						break;
					case 2: 
						master.set_text(0, 0, _2f(getSpeed().x) 
						+ " " + _2f(getSpeed().y)
						+ " " + _2f(getSpeed(true).theta));
						pros::delay(100);
						break;
					case 3:
						master.set_text(0, 0, "LB: " + std::to_string(lbRot.get_position()));
						pros::delay(100);
						break;
					case 4:
						master.set_text(0, 0, _2f(intake.get_position()) + " " + _2f(optical.get_proximity()));
						pros::delay(100);
						break;
					case 5:
						master.set_text(0, 0, "Hori: " + _2f(hori.get_position()));
						pros::delay(100);
						break;
					case 6:
						master.set_text(0, 0, "Vert: " + _2f(vertical.get_position()));
						pros::delay(100);
						break;
					case 7:
						master.set_text(0, 0, "Dist: " + _2f(leftDist.get()) + " " + _2f(rightDist.get()));
						pros::delay(100);
						break;
					case 8:
						master.set_text(0, 0, "Est: " + _2f(dist[0]) + " " + _2f(dist[1]));
						pros::delay(100);
						break;
				}
			} else {
				master.set_text(0, 0, error_msg);
				pros::delay(100);
			}
		} });
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
	pros::Task userTask(driveControl);
}