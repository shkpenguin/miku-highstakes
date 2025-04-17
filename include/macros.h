#pragma once   

#include "math.h"
#include "utils.h"

extern LBState currState;
extern FieldState fieldState;

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

void arcade(int throttle, int turn);