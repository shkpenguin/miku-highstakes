#pragma once

#include <cstdint>

class Timer {
    public:

        Timer(uint32_t time);

        uint32_t getTimeSet();
        
        uint32_t getTimeLeft();

        uint32_t getTimePassed();
        
        bool isDone();
        
        bool isPaused();
        
        void set(uint32_t time);
        
        void reset();
        
        void pause();
        
        void resume();
        
        void waitUntilDone();
    private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;
};