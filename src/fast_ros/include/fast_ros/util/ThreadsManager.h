#pragma once

#include <set>
#include <unistd.h>
#include <sched.h>
#include <mutex>

namespace fast_ros {

class ThreadsManager {
public:
    enum class Stratergy : int {
        FIFO = SCHED_FIFO,
        RR = SCHED_RR,
        OTHER = SCHED_OTHER
    };
    static ThreadsManager& Instance();
    static int GetMaxPriority(Stratergy stratergy);
    static bool SetThreadStrategyAndPriority(pid_t tid, Stratergy stratergy, int priority);
    static bool SetCurrentThreadStrategyAndPriority(Stratergy stratergy, int priority);
    static bool SetThreadCpuSet(pid_t tid, std::set<int> cpus);
    static bool SetCurrentThreadCpuSet(std::set<int> cpus);

    bool Ignore(pid_t tid);
    bool SetManagerCpuSet(std::set<int> cpus);
    bool SetManagerIntervalTime(long ms);
    bool Start();
    bool Stop();

private:
    std::mutex mtxOpt;
    std::set<pid_t> threadsIgnored;
    std::set<pid_t> threadsDone;
    std::set<int> cpus;
    long interval;
    bool started;

    ThreadsManager();
    std::set<pid_t> GetAllThreadsOfProcess();
    void Run();
};

}