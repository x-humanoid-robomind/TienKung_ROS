#include "util/ThreadsManager.h"
#include <thread>
#include <functional>
#include <filesystem>
#include <iostream>

namespace fast_ros {

ThreadsManager& ThreadsManager::Instance() {
    static ThreadsManager singleton;
    return singleton;
}

int ThreadsManager::GetMaxPriority(Stratergy stratergy) {
    return sched_get_priority_max((int)stratergy);
}

bool ThreadsManager::SetThreadStrategyAndPriority(pid_t tid, Stratergy stratergy, int priority) {
    struct sched_param sched;
    auto max = sched_get_priority_max((int)stratergy);
    auto min = sched_get_priority_min((int)stratergy);
    if (priority > max || priority < min) {
        return false;
    }
    sched.sched_priority = priority;

    if (sched_setscheduler(tid, (int)stratergy, &sched) == -1) {
        return false;
    }
    return true;
}

bool ThreadsManager::SetCurrentThreadStrategyAndPriority(Stratergy stratergy, int priority) {
    return SetThreadStrategyAndPriority(gettid(), stratergy, priority);
}

bool ThreadsManager::SetThreadCpuSet(pid_t tid, std::set<int> cpus) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    auto lastCpuId = sysconf(_SC_NPROCESSORS_CONF);
    for (auto& cpu : cpus) {
        if (cpu > lastCpuId) {
            return false;
        }
        CPU_SET(cpu, &mask);
    }
    if (sched_setaffinity(tid, sizeof(mask), &mask) == -1) {
        return false;
    }
    return true;
}

bool ThreadsManager::SetCurrentThreadCpuSet(std::set<int> cpus) {
    return SetThreadCpuSet(gettid(), cpus);
}

ThreadsManager::ThreadsManager() {
    interval = 100;
    started = false;
}

bool ThreadsManager::Ignore(pid_t tid) {
    std::lock_guard<std::mutex> guard(mtxOpt);
    if (started) {
        return false;
    }
    threadsIgnored.insert(tid);
    return true;
}

bool ThreadsManager::SetManagerCpuSet(std::set<int> cpus) {
    std::lock_guard<std::mutex> guard(mtxOpt);
    if (started) {
        return false;
    }
    this->cpus = cpus;
    return true;
}

bool ThreadsManager::SetManagerIntervalTime(long ms) {
    std::lock_guard<std::mutex> guard(mtxOpt);
    if (started) {
        return false;
    }
    interval = ms;
    return true;
}

bool ThreadsManager::Start() {
    std::lock_guard<std::mutex> guard(mtxOpt);
    if (!started) {
        started = true;
        std::thread(std::bind(&ThreadsManager::Run, this)).detach();
        return true;
    }
    return false;
}

bool ThreadsManager::Stop() {
    std::lock_guard<std::mutex> guard(mtxOpt);
    if (started) {
        started = false;
        return true;
    }
    return false;
}

std::set<pid_t> ThreadsManager::GetAllThreadsOfProcess() {
    std::set<pid_t> tids;
    auto pid = std::to_string(getpid());
    auto path = "/proc/" + pid + "/task";
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (std::filesystem::is_directory(entry)) {
            auto tid = entry.path().filename();
            tids.insert(atoi(tid.c_str()));
        }
    }
    return tids;
}

void ThreadsManager::Run() {
    while (started) {
        std::lock_guard<std::mutex> guard(mtxOpt);
        auto tids = GetAllThreadsOfProcess();
        for (auto tid : tids) {
            if (threadsIgnored.find(tid) == threadsIgnored.end() && threadsDone.find(tid) == threadsDone.end()) {
                auto rlt = SetThreadCpuSet(tid, cpus);
                threadsDone.insert(tid);
                
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}

}