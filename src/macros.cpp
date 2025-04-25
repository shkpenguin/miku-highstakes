#include "macros.h"
#include "robot-config.h"
#include "utils/utils.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "utils/timer.h"
#include "chassis/odom.h"

double target = DOWN;
double previous_error = lbRot.get_position();

LBState currState = DOWN; 

void lbControl() {

    while(true) {

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if(currState == DESCORE) target += 1200;
            else {
                currState = RAISED;
            }
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            if(currState == READY) {
                currState = DOWN;
                target = DOWN;
            }
            else {
                currState = READY;
                target = READY;
            }
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            currState = DESCORE;
            target = DESCORE;
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            if(currState == HANG) t3();
            else {
                currState = HANG;
                target = HANG;
                pros::delay(500);
                hang.set_value(true);
            }
        }

        if(currState == RAISED) { 
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                lb.move_voltage(12000);
            } else {
                lb.move_voltage(0);
            }
            pros::delay(10);
            continue;
        }
 
        double error = target - lbRot.get_position();
        double derivative = error - previous_error;
        lb.move_voltage(error * 2 + derivative * 15);
        previous_error = error;

        pros::delay(10);

    }

}

int intakeVoltage = 0;

// allows toggling colorsort/antijam
bool enableColorSort = true;
bool enableAntiJam = true;

// colorsort management
RingState currRing = OUT;
double hue;
double proximity;
double error = 0.9;
double start = 0;
Color sortColor = BLUE;
Color currentColor = NONE;

// antijam management
int jamTimer = 0;

void intakeControl() {

    while(true) {

        if(pros::competition::is_field_control() || !pros::competition::is_competition_switch()) { 
            // L1 up intake
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intakeVoltage = 12000;
            // L2 intake reverse
            else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intakeVoltage = -12000;
            else intakeVoltage = 0;
        }
        
        // prevent led from burning out 
        if(intakeVoltage > 0) optical.set_led_pwm(100);
        else optical.set_led_pwm(0);

        if(enableColorSort && currState != READY) { 
        
            // update sensor values
            hue = optical.get_hue();
            proximity = optical.get_proximity();

            if(currRing == OUT && proximity > 50) { // something is in the intake, colorsort is not actively running
                if(hue < 5 || hue > 330) { 
                    currentColor = RED;
                } else if(hue > 210 && hue < 250) {
                    currentColor = BLUE;
                } else currentColor = NONE;
                if(currentColor == sortColor) { // if the color we just detected matches 
                    currRing = IN; // stop checking for color, start colorsort 
                    start = intake.get_position(); 
                } else currRing = OUT;
            }
            else if(currRing == IN) {
                if(intake.get_position() > start + error) { // if the position has passed far enough
                    currRing = OUT; // start checking for rings again
                    currentColor = NONE; // reset color
                    intake.move_voltage(-12000); // spin back 0.1s
                    pros::delay(50);
                    intake.move_voltage(0);
                } 
            }
        }

        // anti-jam: if it is jamming, we increase the timer by 10(since the loop runs each 10ms), otherwise we reset to 0
        // conditions for jamming:
        // 1. the intake is spinning at at least 83% of its max voltage(always true for driver control)
        // 2. velocity is less than 100(normally 600)
        // 3. the lb is not raised
        // 4. antijam is enabled in the first place(always true for driver control)
        if(intakeVoltage > 10000 && intake.get_actual_velocity() < 100
        && currState != RAISED && enableAntiJam) jamTimer += 10;
        else jamTimer = 0;

        if(jamTimer > 500) { // if it has jammed for more than 0.4 seconds
            intake.move_voltage(-12000); // spin the intake back 0.2s
            jamTimer = 0; // reset timer
            pros::delay(200);
        }

        intake.move_voltage(intakeVoltage);

        pros::delay(10);

    }

}

void setup() {
    initOdom(Pose(0, 0, 0));
    miku.turnToHeading(33, 10000);
}

bool hanging = false;
bool clampEnable = true;
bool pisEnable = true;

void driveControl() {
    while (true) {
        if (hanging) break;
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  
        miku.arcade(dir, turn);

        // clamp control here because why not
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            clamp.set_value(clampEnable);
            clampEnable = !clampEnable;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            setup();
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

void t3() {

    pto.set_value(true);
    pros::delay(200);

    hanging = true;
    currState = RAISED;

    /*
    * pass requirements:
    * 1. lb has reached 5 degrees
    * 2. motor efficiency is less than 5%(needs to be small)
    * 3. 3-4 seconds have passed
    */
    // drive backwards while pushing down lb motor slowly(torquemaxxing)
    Timer timer(5000);

    while(sanitizeAngle(lb.get_position()) > -1500) {
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) || timer.isDone()) break;
        left_dt.move_voltage(-12000);
        right_dt.move_voltage(-12000);
        lb.move_voltage(-2000);
        pros::delay(10);
    }
    left_dt.move_voltage(0);
    right_dt.move_voltage(0);
    lb.move_voltage(0);
    master.rumble(".");
    hang.set_value(false);

    // spin forwards, push lb motor up
    float error = 6000 - lbRot.get_position();
    ExitCondition pass(500, 250);

    while(!pass.getExit()) {
        left_dt.move_voltage(error * 1);
        right_dt.move_voltage(error * 1);
    }

    // engage hang piston, push lb back a little

}