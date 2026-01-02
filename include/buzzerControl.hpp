/* @file buzzerControl.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Buzzer Control class.
 * @version 1.0
 * @date 2024-03-05
 * @updated at: Jun 13, 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BUZZER_CONTROL_HPP
#define BUZZER_CONTROL_HPP

#include <vector>
#include <chrono>

class BuzzerControl
{
public:
    enum class BuzzerState
    {
        BUZZER_OFF,
        BUZZER_ON,
        BEEPING,
        CUSTOM_PATTERN
    };

    BuzzerControl();

    void update();

    void turnOn(int durationMs);
    void turnOff();
    void beep(int count, int onDurationMs = 100, int offDurationMs = 100);
    void setCustomPattern(const std::vector<int> &pattern);

    bool isSignalUpdated();
    bool getSignal() const;

private:
    BuzzerState state;
    bool buzzerSignal;
    bool signalChanged;
    std::vector<int> patternTiming;
    size_t patternIndex;
    int beepCount;
    int currentBeepCount;

    std::chrono::steady_clock::time_point lastToggleTime;
    int onDurationMs;
    int offDurationMs;
    int currentDurationMs;

    void toggleBuzzer();
    void toggleBuzzerPattern();
};

#endif // BUZZER_CONTROL_HPP
