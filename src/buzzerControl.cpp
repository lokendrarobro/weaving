
/* @file buzzerControl.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Buzzer Control class.
 * @version 1.0
 * @date 2024-03-05
 * @updated at: Jun 13, 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "buzzerControl.hpp"

BuzzerControl::BuzzerControl()
    : state(BuzzerState::BUZZER_OFF),
      buzzerSignal(false),
      signalChanged(false),
      patternIndex(0),
      beepCount(0),
      currentBeepCount(0),
      onDurationMs(100),
      offDurationMs(100),
      currentDurationMs(0),
      lastToggleTime(std::chrono::steady_clock::now())
{
}

void BuzzerControl::update()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastToggleTime).count();

    signalChanged = false;

    switch (state)
    {
    case BuzzerState::BUZZER_ON:
        if (elapsedMs >= currentDurationMs)
        {
            turnOff();
        }
        break;

    case BuzzerState::BEEPING:
        if (elapsedMs >= (buzzerSignal ? onDurationMs : offDurationMs))
        {
            toggleBuzzer();
            lastToggleTime = now;

            if (!buzzerSignal)
            {
                ++currentBeepCount;
                if (currentBeepCount >= beepCount)
                {
                    state = BuzzerState::BUZZER_OFF;
                    currentBeepCount = 0;
                }
            }
        }
        break;

    case BuzzerState::CUSTOM_PATTERN:
        if (elapsedMs >= patternTiming[patternIndex])
        {
            toggleBuzzerPattern();
            lastToggleTime = now;

            if (patternIndex == 0 && !buzzerSignal)
            {
                state = BuzzerState::BUZZER_OFF;
                buzzerSignal = false;
                signalChanged = true;
            }
        }
        break;

    default:
        break;
    }
}

void BuzzerControl::turnOn(int durationMs)
{
    state = BuzzerState::BUZZER_ON;
    buzzerSignal = true;
    signalChanged = true;
    currentDurationMs = durationMs;
    lastToggleTime = std::chrono::steady_clock::now();
}

void BuzzerControl::turnOff()
{
    state = BuzzerState::BUZZER_OFF;
    buzzerSignal = false;
    signalChanged = true;
}

void BuzzerControl::beep(int count, int onDurationMs, int offDurationMs)
{
    state = BuzzerState::BEEPING;
    beepCount = count;
    currentBeepCount = 0;
    this->onDurationMs = onDurationMs;
    this->offDurationMs = offDurationMs;
    buzzerSignal = false;
    signalChanged = true;
    lastToggleTime = std::chrono::steady_clock::now();
}

void BuzzerControl::setCustomPattern(const std::vector<int> &pattern)
{
    state = BuzzerState::CUSTOM_PATTERN;
    patternTiming = pattern;
    patternIndex = 0;
    buzzerSignal = false;
    signalChanged = true;
    lastToggleTime = std::chrono::steady_clock::now();
}

bool BuzzerControl::isSignalUpdated()
{
    return signalChanged;
}

bool BuzzerControl::getSignal() const
{
    return buzzerSignal;
}

void BuzzerControl::toggleBuzzer()
{
    buzzerSignal = !buzzerSignal;
    signalChanged = true;
}

void BuzzerControl::toggleBuzzerPattern()
{
    buzzerSignal = !buzzerSignal;
    signalChanged = true;
    patternIndex = (patternIndex + 1) % patternTiming.size();

    if (patternIndex == 0 && !buzzerSignal)
    {
        state = BuzzerState::BUZZER_OFF;
        buzzerSignal = false;
        signalChanged = true;
    }
}
