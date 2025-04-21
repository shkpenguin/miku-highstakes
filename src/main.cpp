#include "main.h"
#include "robot-config.h"
#include "api.h"
#include "macros.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "pros/adi.hpp"
#include "utils/pose.h"

std::string error_msg = "";

void on_center_button() {
	
}

void initialize() {

	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

	pros::Task trackingTask = pros::Task{[=] {
		while (true) {
			update();
			pros::delay(10);
		}
	}};

	vertical.reset_position();
	hori.reset_position();
	intake.tare_position();
	lbRot.reset_position();

	pros::Task intakeTask(intakeControl);
	pros::Task lbTask(lbControl);
	pros::Task controllerDisplay = pros::Task([] {
		while (true) {
			if(error_msg == "") {
				master.set_text(0, 0, _2f(odomPose.x) 
				+ " " + _2f(odomPose.y)
				+ " " + _2f(odomPose.theta));
			} else {
				master.set_text(0, 0, error_msg);
			}
			pros::delay(50);
		}
	});

}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {

}

bool clampEnable = true;
bool pisEnable = true;

void opcontrol() {

	while (true) {

		int dir = master.get_analog(ANALOG_LEFT_Y);   
		int turn = master.get_analog(ANALOG_RIGHT_X);  
		miku.arcade(dir, turn);

		// clamp control here because why not
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            clamp.set_value(clampEnable);
            clampEnable = !clampEnable;
        }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        //     asmacro();
        // }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        //     pistake.set_value(pisEnable);
        //     pisEnable = !pisEnable;
        // }
		
		pros::delay(10);

	}
}