/*
 * Channel.h
 *
 *  Created on: Sep 27, 2021
 *      Author: ubuntu
 */

#pragma once

#include <mutex>
#include <condition_variable>
#include "LockFreeQueue.h"

template<typename DATA, typename TIMEOUT = std::chrono::milliseconds, typename SIZE=std::uint64_t>
class SingleChannel
{
public:
    SingleChannel(SIZE max = 1) : max(max), size(0), enableTimeout(false) {}

    SingleChannel(TIMEOUT& timeout, SIZE max = 1) : timeout(timeout), max(max), size(0), enableTimeout(true) {}

    // not thread-safe
    void SetTimeout(TIMEOUT timeout)
    {
        this->timeout = timeout;
        enableTimeout = true;
    }

    // not thread-safe
    void SetCapacity(SIZE max)
    {
        this->max = max;
    }

    bool operator<<(DATA data)
    {
        if (size == max)
        {
            std::unique_lock<std::mutex> lck(mtxWaitSpace, std::defer_lock);
            if (enableTimeout)
            {
                auto rlt = cvWaitSpace.wait_for(lck, timeout);
                if (rlt == std::cv_status::timeout)
                {
                    return false;
                }
            }
            else
            {
                cvWaitSpace.wait(lck);
            }
        }

        queue.push(data);
        ++size;
        cvWaitData.notify_one();
        return true;
    }

    bool operator>>(DATA& data)
    {
        if (size == 0)
        {
            std::unique_lock<std::mutex> lck(mtxWaitData, std::defer_lock);
            if (enableTimeout)
            {
                auto rlt = cvWaitData.wait_for(lck, timeout);
                if (rlt == std::cv_status::timeout)
                {
                    return false;
                }
            }
            else
            {
                cvWaitData.wait(lck);
            }
        }

        data = queue.pop();
        --size;
        cvWaitSpace.notify_one();
        return true;
    }

    // thread-not-safe, it should be called before push() and pop()
    void clear()
    {
        while (!queue.empty())
        {
            queue.pop();
        }
        size = 0;
    }
private:
    LockFreeQueue<DATA> queue;
    SIZE max;
    volatile SIZE size;
    std::condition_variable cvWaitSpace;
    std::condition_variable cvWaitData;
    std::mutex mtxWaitSpace;
    std::mutex mtxWaitData;
    TIMEOUT timeout;
    bool enableTimeout;
};

template<typename DATA>
using SChan = SingleChannel<DATA>;