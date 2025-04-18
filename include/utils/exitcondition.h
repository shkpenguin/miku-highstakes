#pragma once

class ExitCondition {
    public:

        ExitCondition(const float range, const int time);

        bool getExit();

        bool update(const float input);

        void reset();
    protected:
        const float range;
        const int time;
        int startTime = -1;
        bool done = false;
};