/**
 * @file inductiveSensor.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Inductive Sensor class to store the state of the inductive sensor.
 * @version 1.0
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>

#ifndef INDUCTIVE_SENSOR_HPP
#define INDUCTIVE_SENSOR_HPP

/**
 * @brief Inductive Sensor class to store the state of the inductive sensor.
 */
class InductiveSensor
{
private:
    /**
     * @brief Current state of the sensor.
     */
    bool current_state{false};

    /**
     * @brief Last state of the sensor.
     */
    bool last_state{false};

    /**
     * @brief Rising edge detection flag. to indicate the rising edge of the sensor.
     */
    bool rising_edge{false};

    /**
     * @brief Falling edge detection flag. to indicate the falling edge of the sensor.
     */
    bool falling_edge{false};

    /**
     * @brief Last time the sensor was turned on.
     */
    std::chrono::high_resolution_clock::time_point last_on_time;

    /**
     * @brief Last time the sensor state was updated.
     */
    std::chrono::high_resolution_clock::time_point last_update_time;

public:
    /**
     * @brief Construct a new Inductive Sensor object
     */
    InductiveSensor()
    {
        last_on_time = std::chrono::high_resolution_clock::now();
        last_update_time = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Update the state of the sensor.
     *
     * @param new_state - The new state of the sensor.
     */
    void updateState(bool new_state)
    {
        last_state = current_state;
        current_state = new_state;
        auto now = std::chrono::high_resolution_clock::now();

        rising_edge = !last_state && current_state;
        falling_edge = last_state && !current_state;

        if (rising_edge)
        {
            last_on_time = now; // Update the last on time
        }
        last_update_time = now;
    }

    /**
     * @brief has rising edge of the sensor been detected.
     *
     * @return true - if the rising edge has been detected.
     * @return false - if the rising edge has not been detected.
     */
    bool hasRisingEdge()
    {
        return rising_edge;
    }

    /**
     * @brief has falling edge of the sensor been detected.
     *
     * @return true - if the falling edge has been detected.
     * @return false - if the falling edge has not been detected.
     */
    bool hasFallingEdge()
    {
        return falling_edge;
    }

    /**
     * @brief Reset the edge detection flags.
     */
    void resetEdges()
    {
        rising_edge = false;
        falling_edge = false;
    }

    /**
     * @brief Get the Time Since Last On object
     *
     * @return std::chrono::duration<double> - The time since the last time the sensor was turned on.
     */
    std::chrono::duration<double> getTimeSinceLastOn()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - last_on_time);
    }

    /**
     * @brief Get the Time Since Last Update object
     *
     * @return std::chrono::duration<double> - The time since the last time the sensor state was updated.
     */
    std::chrono::duration<double> getTimeSinceLastUpdate()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - last_update_time);
    }
};

#endif