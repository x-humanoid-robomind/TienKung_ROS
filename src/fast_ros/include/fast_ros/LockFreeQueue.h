#pragma once

#include <cstddef>
#include <stdexcept>

namespace fast_ros {

template <typename T>
class LockFreeQueue
{
public:
    LockFreeQueue()
    {
        head = new Node();
        tail = head;
    }

    T pop()
    {
        if (empty())
        {
            throw std::out_of_range("queue is empty");
        }

        auto e = head->next->value;
        auto preHead = head;
        head = head->next;
        delete preHead;
        return e;
    }

    bool empty()
    {
        return head == tail;
    }

    void push(T& e)
    {
        auto node = new Node();
        node->value = e;
        tail->next = node;
        tail = node;
    }

private:
    struct Node
    {
        T value;
        Node* next = nullptr;

        Node() {}
    };

    Node* head;
    Node* tail;
};

}