#pragma once   

#include "utils/math.h"
#include "utils/utils.h"

enum LBState {
    DOWN = 0,
    READY = 3700,
    RAISED = -1,
    DESCORE = 16500,
    HANG = 10000
};

enum RingState {
    IN = 1,
    OUT = 2
};

extern LBState currState;

void intakeControl();
void lbControl();

extern double target;

extern int intakeVoltage;

// allows toggling colorsort/antijam
extern bool enableColorSort;
extern bool enableAntiJam;

// colorsort management
extern RingState currRing;
extern double hue;
extern double proximity;
extern double error;
extern double start;
extern Color sortColor;
extern Color currentColor;

// antijam management
extern int jamTimer;

void intakeControl();
void driveControl();

extern bool hanging;
void t3();