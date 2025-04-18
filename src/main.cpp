#include "main.h"
#include "robot-config.h"
#include "api.h"
#include "macros.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "pros/adi.hpp"

std::string error_msg = "";

void on_center_button() {
	
}

void initialize() {

	fieldState = INITIALIZE;

	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

	left_dt.tare_position_all();
	right_dt.tare_position_all();
	intake.tare_position();
	lb.tare_position();

	pros::Task intakeTask(intakeControl);
	pros::Task lbTask(lbControl);
	pros::Task controllerDisplay = pros::Task([] {
		while (true) {
			master.set_text(0, 0, std::to_string(static_cast<int>(getPose().x * 10) / 10.0) + 
				" " + std::to_string(static_cast<int>(getPose().y * 10) / 10.0) + 
				" " + std::to_string(static_cast<int>(getPose().theta * 10) / 10.0));
			master.set_text(1, 0, error_msg);
			pros::delay(50);
		}
	});

}

void disabled() {

	fieldState = DISABLED;

}

void competition_initialize() {

	fieldState = COMPETITION;

}

void autonomous() {

	fieldState = AUTONOMOUS;

}

void opcontrol() {

	fieldState = OPCONTROL;

	while (true) {

		int dir = master.get_analog(ANALOG_LEFT_Y);   
		int turn = master.get_analog(ANALOG_RIGHT_X);  
		arcade(dir, turn);
		pros::delay(10); 
	}
}