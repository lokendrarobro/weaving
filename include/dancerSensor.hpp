/**
 * @file dancerSensor.hpp
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
#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>
#include <exception>
#include <algorithm>
#include <cmath>

#ifndef DANCER_SENSOR_HPP
#define DANCER_SENSOR_HPP

class DancerSensor
{
public:
    struct SensorReading
    {
        std::chrono::high_resolution_clock::time_point timestamp;
        uint64_t timestamp_ms;
        uint64_t frontEncoderValue;
        uint64_t backEncoderValue;
        int difference; // Signed difference
    };

private:
    std::vector<SensorReading> readings;  // Stores all relevant readings
    bool current_state{false};            // Current state of the sensor
    bool rising_edge{false};              // Flag to indicate a rising edge
    size_t max_size{10};                  // Maximum number of readings to store
    uint64_t last_front_encoder_value{0}; // Most recent front encoder value
    uint64_t last_back_encoder_value{0};  // Most recent back encoder value

public:
    explicit DancerSensor(size_t maxSize = 10) : max_size(maxSize)
    {
        SensorReading newReading;
        newReading.timestamp = std::chrono::high_resolution_clock::now();
        newReading.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(newReading.timestamp.time_since_epoch()).count();
        newReading.frontEncoderValue = 0;
        newReading.backEncoderValue = 0;
        newReading.difference = 0;
        for (int i = 0; i < 5; ++i)
        {
            readings.push_back(newReading);
        }
    }

    void updateState(bool new_state, int frontEncoder, int backEncoder)
    {
        last_front_encoder_value = frontEncoder;
        last_back_encoder_value = backEncoder;

        if (new_state && !current_state)
        { // Only record on rising edges
            rising_edge = true;
            current_state = true;
            if (readings.size() >= max_size)
            {
                readings.erase(readings.begin()); // Maintain a limited history
            }
            SensorReading newReading;
            newReading.timestamp = std::chrono::high_resolution_clock::now();
            newReading.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(newReading.timestamp.time_since_epoch()).count();
            newReading.frontEncoderValue = frontEncoder;
            newReading.backEncoderValue = backEncoder;
            newReading.difference = frontEncoder - backEncoder;
            readings.push_back(newReading);
        }
        else
        {
            rising_edge = false;
            current_state = new_state;
        }
    }

    bool hasRisingEdge()
    {
        return rising_edge;
    }

    bool hasNotTurnedOnSinceDistance(int x, bool isFront) const
    {
        if (readings.empty())
        {
            return true; // Assume true if no readings available
        }
        int lastReading = isFront ? last_front_encoder_value : last_back_encoder_value;
        int currentReading = isFront ? readings.back().frontEncoderValue : readings.back().backEncoderValue;
        return (currentReading - lastReading >= x);
    }

    bool hasNotTurnedOnSinceTime(std::chrono::milliseconds x) const
    {
        if (readings.empty())
        {
            return true; // Assume true if no readings available
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto durationSinceLastOn = std::chrono::duration_cast<std::chrono::milliseconds>(now - readings.back().timestamp);
        return durationSinceLastOn >= x;
    }

    std::vector<SensorReading> getLastFiveReadings() const
    {
        std::vector<SensorReading> lastFive;
        auto count = std::min(readings.size(), size_t(4));
        lastFive.insert(lastFive.end(), readings.end() - count, readings.end());
        return lastFive;
    }

    std::vector<uint64_t> getLastFiveFrontEncoderValues() const
    {
        auto lastFive = getLastFiveReadings();
        std::vector<uint64_t> frontEncoderValues;
        for (const auto &reading : lastFive)
        {
            frontEncoderValues.push_back(reading.frontEncoderValue);
        }
        return frontEncoderValues;
    }

    std::vector<uint64_t> getLastFiveBackEncoderValues() const
    {
        auto lastFive = getLastFiveReadings();
        std::vector<uint64_t> backEncoderValues;
        for (const auto &reading : lastFive)
        {
            backEncoderValues.push_back(reading.backEncoderValue);
        }
        return backEncoderValues;
    }

    std::vector<int> getLastFiveDifferences() const
    {
        auto lastFive = getLastFiveReadings();
        std::vector<int> differences;
        for (const auto &reading : lastFive)
        {
            differences.push_back(reading.difference);
        }
        return differences;
    }

    std::vector<uint64_t> getLastFiveActivationTimes() const
    {
        auto lastFive = getLastFiveReadings();
        std::vector<uint64_t> activationTimes;
        for (const auto &reading : lastFive)
        {
            activationTimes.push_back(reading.timestamp_ms);
        }
        return activationTimes;
    }

    std::string getAverageOfLastFiveDifferencesIndicator() const
    {
        auto lastFive = getLastFiveReadings();
        double average = std::accumulate(lastFive.begin(), lastFive.end(), 0.0,
                                         [](double sum, const SensorReading &reading)
                                         { return sum + reading.difference; }) /
                         lastFive.size();
        if (std::abs(average) <= 50)
        {
            return "Within 50";
        }
        else if (std::abs(average) <= 200)
        {
            return "Within 200";
        }
        else
        {
            return "Greater than 200";
        }
    }

    std::string getIndicatorColor()
    {
        if (hasNotTurnedOnSinceTime(std::chrono::minutes(2)))
        {
            return "#FFFF00"; // Yellow hex code if not turned on for 2 minutes
        }
        else if (hasNotTurnedOnSinceDistance(25000, true))
        {                     // Check distance in meters, assuming the encoder values are in meters
            return "#FF0000"; // Red hex code if not turned on for 25 meters
        }
        else
        {
            return "#00FF00"; // Green hex code otherwise
        }
    }

    void displayAllData() const
    {
        std::cout << "Sensor readings detail:" << std::endl;
        for (const auto &reading : readings)
        {
            std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(reading.timestamp.time_since_epoch()).count()
                      << " ms, Front Encoder: " << reading.frontEncoderValue
                      << ", Back Encoder: " << reading.backEncoderValue
                      << ", Difference: " << reading.difference << std::endl;
        }
    }
};

#endif // D
