#include "macros.h"
#include "robot-config.h"
#include "utils/utils.h"

double load = 0.2;
double descore = 0.8;
double target = load;
double previous_error = lb.get_position();

FieldState fieldState = INITIALIZE;
LBState currState = DOWN; 

void lbControl() {

    while(true) {

        if(currState == DOWN) { // if the lb is down
            // move the lb up if R1 is pressed
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                currState = RAISED;
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            // set the state to ready if R2 is pressed
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                currState = READY; 
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            }
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                currState = DESCORE;
                target = descore;
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            // otherwise, keep the lb down
            else {
                double error = target - lb.get_position();
                lb.move_voltage(error * 20000);
                previous_error = error;
            }
        }

        else if(currState == READY) { // use PD loop to raise lb
            // move the lb up if R1 is pressed
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                load += 0.1;
                target = load;
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                load -= 0.1;
                target = load;
            }
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                currState = RAISED;
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            // put the lb back down if R2 is pressed 
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                target = 0;
                currState = DOWN;
            }
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                currState = DESCORE;
                target = descore;
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            // otherwise, keep the lb ready
            else {
                double error = target - lb.get_position();
                lb.move_voltage(error * 20000);
                previous_error = error;
            }
        }

        else if(currState == RAISED) { // move the lb forwards
            // if R1 is pressed, keep raising the lb
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) lb.move_voltage(12000);
            // if R2 is pressed, put the lb back to ready position
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                currState = READY;
            }
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                currState = DESCORE;
                target = descore;
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            // otherwise, hold the lb
            else {
                lb.move_voltage(0);
            }
        }

        else if(currState == DESCORE) { // move the lb back down
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) target += 0.2;
            else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                target = load;
                currState = READY;
            }
            else {
                double error = target - lb.get_position();
                lb.move_voltage(error * 20000);
                previous_error = error;
            }
        }

        pros::delay(10);

    }

}

int intakeVoltage = 0;

// allows toggling colorsort/antijam
// bool enableColorSort = true;
bool enableAntiJam = true;

// colorsort management
// ringState currRing = OUT;
// double hue;
// double proximity;
// double error = 0.4;
// double start = 0;
// Color sortColor;
// Color currentColor = NONE;

// antijam management
int jamTimer = 0;

// pistonized functions
bool clampEnable = true;
bool pisEnable = true;

void intakeControl() {

    while(true) {

        if(fieldState == OPCONTROL) { // only take controller input in driver control
            // L1 up intake
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intakeVoltage = 12000;
            // L2 intake reverse
            else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intakeVoltage = -12000;
            else intakeVoltage = 0;

            // color sort toggle
            // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { 
            //     enableColorSort = !enableColorSort;
            //     controller.rumble("-");
            // }
        }
        
        // prevent led from burning out 
        // if(intakeVoltage > 0) optical.set_led_pwm(100);
        // else optical.set_led_pwm(0);

        /*
        if(enableColorSort && currState != READY) { // dont run if lb is currently in ready pos
        
            // update sensor values
            hue = optical.get_hue();
            proximity = optical.get_proximity();

            if(currRing == OUT && proximity > 150) { // something is in the intake, colorsort is not actively running
                if(hue < 5 || hue > 330) { 
                    currentColor = RED;
                } else if(hue > 220 && hue < 240) {
                    currentColor = BLUE;
                } else currentColor = NONE;
                if(currentColor == sortColor) { // if the color we just detected matches 
                    currRing = COLOR; // stop checking for color, start colorsort 
                } else currRing = OUT;
            }
            else if(currRing == COLOR) { 
                if(dist.get_distance() < 10) {
                    currRing = DIST;
                    controller.rumble(".");
                }
                start = intake.get_position();
            }
            else if(currRing == DIST) {
                if(intake.get_position() > start + error) { // if the position has passed far enough
                    currRing = OUT; // start checking for rings again
                    currentColor = NONE; // reset color
                    intake.move_voltage(-12000); // spin back 0.1s
                    pros::delay(100);
                    intake.move_voltage(0);
                } 
            }
        }
        */

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

        intake.move_voltage(intakeVoltage);

        pros::delay(10);

    }

}

void arcade(int throttle, int turn) {

    int leftPower = throttle + turn;
    int rightPower = throttle - turn;

    // move drive
    left_dt.move(leftPower);
    right_dt.move(rightPower);

}