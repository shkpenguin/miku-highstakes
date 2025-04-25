#pragma once

#include "utils/pose.h"
#include "utils/utils.h"

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 240
#define HALF_WIDTH SCREEN_WIDTH / 2
#define HALF_HEIGHT SCREEN_HEIGHT / 2

struct Auton {
    public:
        Auton() = default;
        Auton(Color color, std::string name, std::function<void()> func, Pose start) : 
        color(color), name(name), func(func), start(start) {}
        std::string name;
        std::function<void()> func;
        Pose start = Pose(0, 0, 0);
        Color color;
};

extern Auton selectedAuton;

void initSelector();