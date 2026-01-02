/**
 * @file speedCalculator.hpp
 * @brief Utility to compute machine speed from encoder distance readings.
 */
#ifndef SPEED_CALCULATOR_HPP
#define SPEED_CALCULATOR_HPP

#include <chrono>
#include <cstdint>
#include <vector>
#include <numeric>

class SpeedCalculator
{
public:
    SpeedCalculator()
        : log_interval(std::chrono::seconds(10)) // fixed 10s log cadence
    {
    }

    /**
     * @brief Update the calculator with the latest absolute distance (in mm) and compute speed.
     * @param current_distance_mm Absolute encoder distance in mm.
     */
    void calculatesSpeedAndStore(uint64_t current_distance_mm)
    {
        auto now = std::chrono::steady_clock::now();

        if (!has_sample)
        {
            last_distance_mm = current_distance_mm;
            last_sample_time = now;
            last_log_time = now;
            has_sample = true;
            last_speed_m_per_min = 0.0;
        }

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample_time).count();
        if (elapsed_ms > 0)
        {
            double delta_mm = current_distance_mm > last_distance_mm ? static_cast<double>(current_distance_mm - last_distance_mm) : 0.0;
            double minutes = elapsed_ms / 60000.0;
            last_speed_m_per_min = minutes > 0.0 ? (delta_mm / 1000.0) / minutes : 0.0;
        }

        last_distance_mm = current_distance_mm;
        last_sample_time = now;
        storeSample(last_speed_m_per_min);
    }


    /**
     * @brief Indicates if it is time to log the current speed.
     */
    bool shouldLog() const
    {
        if (!has_sample)
        {
            return false;
        }
        auto now = std::chrono::steady_clock::now();
        return (now - last_log_time) >= log_interval;
    }

    /**
     * @brief Mark that we have logged, so the next log waits for the interval.
     */
    void markLogged()
    {
        last_log_time = std::chrono::steady_clock::now();
    }

    double getSpeedMPerMin() const { return last_speed_m_per_min; }
    double getAverageSpeed() const
    {
        if (speed_samples.empty())
        {
            return last_speed_m_per_min;
        }
        double sum = std::accumulate(speed_samples.begin(), speed_samples.end(), 0.0);
        return sum / static_cast<double>(speed_samples.size());
    }

private:
    void storeSample(double speed_m_per_min)
    {
        speed_samples.push_back(speed_m_per_min);
        if (speed_samples.size() > max_samples)
        {
            speed_samples.erase(speed_samples.begin());
        }
    }

    bool has_sample{false};
    uint64_t last_distance_mm{0};
    std::chrono::steady_clock::time_point last_sample_time{};
    std::chrono::steady_clock::time_point last_log_time{};
    std::chrono::steady_clock::duration log_interval;
    double last_speed_m_per_min{0.0};
    std::vector<double> speed_samples;
    size_t max_samples{20};
};

#endif // SPEED_CALCULATOR_HPP
