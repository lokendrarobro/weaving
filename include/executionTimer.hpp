/**
 * @file executionTimer.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Execution Timer class to measure the time taken by a function to execute.
 * @version 1.0
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>
#include <ctime>
#include <cstdint> // For int64_t

#ifndef EXECUTION_TIMER_HPP
#define EXECUTION_TIMER_HPP

/**
 * @brief Execution Timer class to measure the time taken by a function to execute.
 */
class ExecutionTimer
{
public:
    /**
     * @brief Construct a new Execution Timer object
     *
     * @param history_size - The size of the history to store the execution times.
     */
    ExecutionTimer(size_t _history_size = 10) : history_size(_history_size) {}

    /**
     * @brief Start the timer.
     */
    void start()
    {
        start_time = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Stop the timer.
     */
    void stop()
    {
        auto stop_time = std::chrono::high_resolution_clock::now();
        execution_times.push_back(std::chrono::system_clock::now());
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time).count();
        durations.push_back(static_cast<int64_t>(duration)); // Use int64_t here
        // Since we are pushing here, we need to pop the first element if the size exceeds the history size
        if (durations.size() > history_size)
        {
            durations.erase(durations.begin());
            execution_times.erase(execution_times.begin());
        }
    }

    /**
     * @brief Get the last duration in microseconds.
     *
     * @return int64_t - The last duration in microseconds.
     */
    int64_t getLastDurationInMicroseconds() const
    {
        if (!durations.empty())
        {
            return durations.back();
        }
        return 0; // No duration recorded
    }

    /**
     * @brief Get the last duration in milliseconds.
     *
     * @return int64_t - The last duration in milliseconds.
     */
    int64_t getLastDurationInMilliseconds() const
    {
        return getLastDurationInMicroseconds() / 1000;
    }

    /**
     * @brief Get the Average Duration In Milliseconds.
     *
     * @return double - The average duration in milliseconds.
     */
    double getAverageDurationInMilliseconds() const
    {
        if (!durations.empty())
        {
            return std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size() / 1000;
        }
        return 0; // No durations to average
    }

    /**
     * @brief Print the average time after a certain number of seconds.
     *
     * @param seconds_to_print_after - The number of seconds after which to print the average time.
     */
    void printAverageTime(int seconds_to_print_after)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(time_now - last_print_time).count() > seconds_to_print_after)
        {
            std::cout << "Avg: Time: " << getAverageDurationInMilliseconds() << " ms\n";
            last_print_time = time_now;
        }
    }
    
    /**
     * @brief Reset the timer.
     */
    void reset()
    {
        durations.clear();
        execution_times.clear();
    }

private:
    /**
     * @brief The start time of the execution timer.
     */
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    
    /**
     * @brief The last time the average time was printed.
     */
    std::chrono::time_point<std::chrono::high_resolution_clock> last_print_time;
    
    /**
     * @brief Duration of the execution stored in microseconds in a vector.
     */
    std::vector<int64_t> durations;

    /**
     * @brief The time of execution stored in a vector.
     */
    std::vector<std::chrono::system_clock::time_point> execution_times;
    
    /**
     * @brief The size of the history to store the execution times.
     */
    size_t history_size;
};

#endif