/**
 * @file CutterSensor.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Dancer Sensor
 * @version 1.0
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2024
 */
#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>
#include <exception>
#include <algorithm>
#include <cmath>

#ifndef CUTTER_SENSOR_HPP
#define CUTTER_SENSOR_HPP

class CutterSensor
{
public:
    struct SensorReading // records on falling edges
    {
        std::chrono::high_resolution_clock::time_point timestamp;
        uint64_t timestamp_ms;
        uint64_t encoderValue;
    };

private:
    std::vector<SensorReading> readings;
    bool current_state{false};
    bool falling_edge{false};
    size_t max_size{10};
    uint64_t last_encoder_value{0};

public:
    explicit CutterSensor(size_t maxSize = 10) : max_size(maxSize)
    {
        SensorReading newReading;
        newReading.timestamp = std::chrono::high_resolution_clock::now();
        newReading.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(newReading.timestamp.time_since_epoch()).count();
        newReading.encoderValue = 0;
        for (int i = 0; i < 5; ++i)
        {
            readings.push_back(newReading);
        }
    }

    void updateState(bool new_state, uint64_t encoderValue)
    {
        if (!new_state && current_state)
        { // Falling edge detected
            falling_edge = true;
            current_state = false;
            if (readings.size() >= max_size)
            {
                readings.erase(readings.begin());
            }
            SensorReading newReading;
            newReading.timestamp = std::chrono::high_resolution_clock::now();
            newReading.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(newReading.timestamp.time_since_epoch()).count();
            newReading.encoderValue = encoderValue;
            readings.push_back(newReading);
        }
        else
        {
            falling_edge = false;
            current_state = new_state;
        }
        last_encoder_value = encoderValue;
    }

    bool hasFallingEdge()
    {
        return falling_edge;
    }

    bool hasNotTurnedOnSinceDistance(int x) const
    {
        if (readings.empty())
        {
            return true;
        }
        int lastReading = readings.back().encoderValue;
        return (last_encoder_value - lastReading >= x);
    }

    bool hasNotTurnedOnSinceTime(std::chrono::milliseconds x) const
    {
        if (readings.empty())
        {
            return true;
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto durationSinceLastOn = std::chrono::duration_cast<std::chrono::milliseconds>(now - readings.back().timestamp);
        return durationSinceLastOn >= x;
    }

    std::vector<SensorReading> getLastFiveActivations() const
    {
        std::vector<SensorReading> lastFive;
        auto count = std::min(size_t(4), readings.size());
        for (size_t i = 1; i <= count; i++)
        {
            lastFive.push_back(readings[readings.size() - i]);
        }
        return lastFive;
    }

    std::vector<uint64_t> getLastFiveTimestamps() const
    {
        std::vector<uint64_t> lastFive;
        auto lastFiveActivations = getLastFiveActivations();
        for (const auto &reading : lastFiveActivations)
        {
            lastFive.push_back(reading.timestamp_ms);
        }
        return lastFive;
    }

    std::vector<uint64_t> getLastFiveEncoderValues() const
    {
        std::vector<uint64_t> lastFive;
        auto lastFiveActivations = getLastFiveActivations();
        for (const auto &reading : lastFiveActivations)
        {
            lastFive.push_back(reading.encoderValue);
        }
        return lastFive;
    }

    double getAverageEncoderValueForLastFiveActions() const
    {
        auto lastFive = getLastFiveEncoderValues();
        double average = std::accumulate(lastFive.begin(), lastFive.end(), 0.0) / lastFive.size();
        return average;
    }

    void displayAllData() const
    {
        std::cout << "Cutter Sensor Readings:" << std::endl;
        for (const auto &reading : readings)
        {
            std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(reading.timestamp.time_since_epoch()).count()
                      << " ms, Encoder Value: " << reading.encoderValue << std::endl;
        }
    }
};

#endif // CUTTER_SENSOR_HPP
