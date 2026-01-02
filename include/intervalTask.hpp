#ifndef INTERVAL_TASK_HPP
#define INTERVAL_TASK_HPP

#include <chrono>

class IntervalTask {
public:
    // Constructor to initialize the task interval
    IntervalTask(unsigned long long intervalMillis)
        : interval(std::chrono::milliseconds(intervalMillis)), lastExecution(std::chrono::steady_clock::now()) {}

    // Method to check if it's time to execute the task
    bool isReady() {
        auto now = std::chrono::steady_clock::now();
        if (now - lastExecution >= interval) {
            return true; // It's time to execute the task
        }
        return false;
    }

    // Method to reset the last execution time
    void reset() {
        lastExecution = std::chrono::steady_clock::now(); // Update last execution time to current
    }

private:
    std::chrono::milliseconds interval; // Interval between task executions
    std::chrono::steady_clock::time_point lastExecution; // Last execution time
};

#endif // INTERVAL_TASK_HPP
