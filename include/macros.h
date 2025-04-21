#pragma once   

#include "utils/math.h"
#include "utils/utils.h"

extern LBState currState;

extern pros::Task lbTask;

void lbControl();

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

void t3();