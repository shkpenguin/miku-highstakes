#include "main.h"
#include "robot-config.h"
#include "api.h"
#include "macros.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "pros/adi.hpp"
#include "utils/pose.h"
#include "chassis/mcl.h"

std::string error_msg = "";
int display = 0;
int options = 7;

void on_center_button() {
	
}

void initialize() {

	display = 0;

	vertical.reset_position();
	hori.reset_position();
	
	intake.tare_position();
	lbRot.reset_position();

	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

	initOdom();
	
	pros::Task intakeTask(intakeControl);
	pros::Task lbTask(lbControl);
	pros::Task controllerDisplay = pros::Task([] {
		while (true) {
			std::vector<float> dist = estimateDistance(getPose(true));
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				display = (display + 1) % options;
			}
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				display = (display - 1 + options) % options;
			}
			if(error_msg == "") {
				switch (display) {
					case 0:
						master.set_text(0, 0, std::to_string(dist[0]) 
						+ " " + std::to_string(dist[1]));
						pros::delay(100);
						break;
					case 1: 
						master.set_text(0, 0, _2f(getPose().x) 
						+ " " + _2f(getPose().y)
						+ " " + _2f(getPose(true).theta));
						pros::delay(100);
						break;
					case 2: 
						master.set_text(0, 0, _2f(leftDist.get_distance() / 25.4) 
						+ " " + _2f(rightDist.get_distance() / 25.4));
						pros::delay(100);
						break;
					case 3:
						master.set_text(0, 0, "LB: " + _2f(lbRot.get_position()));
						pros::delay(100);
						break;
					case 4:
						master.set_text(0, 0, "Intake: " + _2f(intake.get_position()));
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
				}
					
			} else {
				master.set_text(0, 0, error_msg);
				pros::delay(100);
			}
		}
	});

}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {

	initOdom();
	initParticles();

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