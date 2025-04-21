#pragma once

class ExitCondition {
    public:

        ExitCondition(const float range, const int time) :
            range(range),
            time(time) {}

        bool getExit();

        bool update(const float input);

        void reset();
    protected:
        const float range;
        const int time;
        int startTime = -1;
        bool done = false;
};