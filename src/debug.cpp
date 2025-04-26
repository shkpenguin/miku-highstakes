#include "debug.h"
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
#include "utils/timer.h"

std::string error_msg = "";

void controllerDisplay() {
    int display = 0;
    int options = 10;
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
                case 9:
                    master.set_text(0, 0, "Vel: " + _2f(miku.get_velocity(&left_dt)) + " " + _2f(miku.get_velocity(&right_dt)));
                    pros::delay(100);
            }
        } else {
            master.set_text(0, 0, error_msg);
            pros::delay(100);
        }
    } 
}

void displayError(std::string msg, int time) {
    Timer timer(time);
    error_msg = msg;
    while(!timer.isDone()) {
        pros::delay(10);
    }
    error_msg = "";
}