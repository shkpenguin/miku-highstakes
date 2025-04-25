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

void redRing() {
    // miku.boomerang(24, -24, 180, 1600, {.forwards = false, .maxSpeed = 80});
    // pros::delay(200);
    miku.moveDistance(18, 600); // scores alliance stake
    target = 21000;
    miku.waitUntilDone();
    miku.moveDistance(-34, 1000); // grabs goal
    pros::delay(500);
    target = 0;
    miku.waitUntilDone();
    clamp.set_value(true);
    pros::delay(100);
    miku.turnToHeading(45, 700);
    miku.waitUntilDone();
    intakeVoltage = 12000;
    miku.boomerang(57, -13, 90, 1800, {.maxSpeed = 70}); // center rings
    miku.waitUntilDone();
    miku.swingToHeading(180, DriveSide::RIGHT, 400);
    miku.turnToPoint(48, -26, 400); // silly turn to ring
    miku.waitUntilDone();
    miku.movePoint(48, -26, 500, {.minSpeed = 50}); // ring
    miku.boomerang(64, -64, 135, 1500); // corner
    miku.waitUntilDone();
    miku.moveTime(500, 127); // initial backshot
    miku.waitUntilDone();
    miku.moveTime(500, -127);
    miku.waitUntilDone();
    miku.moveTime(300, 127); // small backshot to pick up third ring
    miku.waitUntilDone();
    miku.boomerang(54, -36, 180, 1200, {.forwards = false}); // pull out
    miku.boomerang(12, -48, -90, 1200); // move to behind ring stack
    miku.moveTime(1000, 60); // slowly drive forwards to pick up both rings
    miku.turnToPoint(-64, -64, 500, {.forwards = false}); // move to corner
    miku.movePoint(-64, -64, 800, {.minSpeed = 50});
}

void blueRing() {
    // miku.boomerang(24, -24, 180, 1600, {.forwards = false, .maxSpeed = 80});
    // pros::delay(200);
    miku.moveDistance(18, 600); // scores alliance stake
    target = 21000;
    miku.waitUntilDone();
    miku.moveDistance(-34, 1000); // grabs goal
    pros::delay(500);
    target = 0;
    miku.waitUntilDone();
    clamp.set_value(true);
    pros::delay(100);
    miku.turnToHeading(45, 700);
    miku.waitUntilDone();
    intakeVoltage = 12000;
    miku.boomerang(57, -13, 90, 1800, {.maxSpeed = 70}); // center rings
    miku.waitUntilDone();
    miku.swingToHeading(180, DriveSide::RIGHT, 400);
    miku.turnToPoint(48, -26, 400); // silly turn to ring
    miku.waitUntilDone();
    miku.movePoint(48, -26, 500, {.minSpeed = 50}); // ring
    miku.boomerang(64, -64, 135, 1500); // corner
    miku.waitUntilDone();
    miku.moveTime(500, 127); // initial backshot
    miku.waitUntilDone();
    miku.moveTime(500, -127);
    miku.waitUntilDone();
    miku.moveTime(300, 127); // small backshot to pick up third ring
    miku.waitUntilDone();
    miku.boomerang(54, -36, 180, 1200, {.forwards = false}); // pull out
    miku.boomerang(12, -48, -90, 1200); // move to behind ring stack
    miku.moveTime(1000, 60); // slowly drive forwards to pick up both rings
    miku.turnToPoint(-64, -64, 500, {.forwards = false}); // move to corner
    miku.movePoint(-64, -64, 800, {.minSpeed = 50});
}

void redSawp() {}

void blueSawp() {}