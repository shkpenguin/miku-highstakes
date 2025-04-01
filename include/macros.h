#pragma once   

enum lbState {
    DOWN = 0,
    READY = 1,
    RAISED = 2,
    DESCORE = 3
};

enum ringState {
    COLOR = 0,
    DIST = 1,
    OUT = 2
};

enum Color {
    RED = 1,
    BLUE = -1,
    NONE = 0
};

extern lbState currState;

void lbControl();

extern bool autoEnable;
extern int intakeVoltage;

// allows toggling colorsort/antijam
extern bool enableColorSort;
extern bool enableAntiJam;

// colorsort management
extern ringState currRing;
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