#include "robot-config.h"
#include "selector.h"
#include "routes.h"
#include "macros.h"

std::vector<Auton> redAutons = {
    Auton(RED, "Test 1", blueRing, Pose(0, 0, 0)),
};

std::vector<Auton> blueAutons = {

};

void redRush() {}

void blueRush() {}

void redRing() {}

void blueRing() {
    target = 21000;
    pros::delay(700);
    miku.boomerang(24, -24, 180, 1600, {.forwards = false, .maxSpeed = 80});
    pros::delay(200);
    target = 0;
    miku.waitUntilDone();
    clamp.set_value(true);
    pros::delay(100);
    miku.turnToHeading(45, 700);
    miku.waitUntilDone();
    intakeVoltage = 12000;
    miku.boomerang(57, -13, 90, 1800, {.maxSpeed = 70});
    miku.waitUntilDone();
    miku.swingToHeading(180, DriveSide::RIGHT, 400);
    miku.turnToPoint(48, -26, 400);
    miku.waitUntilDone();
    miku.movePoint(48, -26, 500, {.minSpeed = 50});
    miku.boomerang(64, -64, 135, 2000);
    miku.waitUntilDone();
    miku.moveTime(500, 100);
    miku.waitUntilDone();
    miku.moveDistance(-15, 400);
    miku.waitUntilDone();
    miku.moveDistance(8, 300);
}

void redSawp() {}

void blueSawp() {}