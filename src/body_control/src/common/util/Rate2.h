#pragma once

#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unistd.h>


class Rate
{
    std::uint64_t baseTime, sleepTime, hz, counter;
    struct timeval tvStart, tvEnd;

public:
    Rate(std::uint64_t hz) : hz(hz), counter(0)
    {
        baseTime = 1000000 / hz;
        sleepTime = baseTime;
    }

    int sleep()
    {
        if (counter < hz / 2) {
            ++counter;
            return usleep(sleepTime);
        }
        else if (counter == hz / 2) {
            gettimeofday(&tvStart, NULL);
            ++counter;
            // printf("\nsleep time1: %ld", sleepTime);
            return usleep(sleepTime);
        }
        else if (counter < hz * 2) {
            ++counter;
            // printf("\nsleep time2: %ld", sleepTime);
            return usleep(sleepTime);
        }
        else if (counter == hz * 2) {
            gettimeofday(&tvEnd, NULL);
            auto dt = (tvEnd.tv_sec - tvStart.tv_sec) * 1000000 + (tvEnd.tv_usec - tvStart.tv_usec);
            auto avgDt = dt / (hz * 3 / 2);
            sleepTime -= (avgDt - baseTime) * 95 / 100;            
            // printf("\nsleep time3: %ld", sleepTime);
            ++counter;
            return usleep(sleepTime);
        }
        else {
            // printf("\nsleep time4: %ld", sleepTime);
            return usleep(sleepTime);
        }

        return 0;
    }

};