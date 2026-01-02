#ifndef CYCLE_EVENT_TRACKER_HPP
#define CYCLE_EVENT_TRACKER_HPP

#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>

class CycleEventTracker
{
public:
    enum class State
    {
        Waiting,
        Planning,
        Start
    };

    struct Event
    {
        State state = State::Waiting;
        std::chrono::high_resolution_clock::time_point timestamp{};
        bool isValid = false;

        std::string getTimeString() const
        {
            if (!isValid)
                return "NA";

            using namespace std::chrono;
            auto ms = duration_cast<milliseconds>(timestamp.time_since_epoch()) % 1000;
            std::time_t t = system_clock::to_time_t(timestamp);
            std::tm now_tm = *std::localtime(&t); // not thread-safe

            std::ostringstream oss;
            oss << std::setfill('0') << std::setw(2) << now_tm.tm_min << ":"
                << std::setfill('0') << std::setw(2) << now_tm.tm_sec << "."
                << std::setfill('0') << std::setw(3) << ms.count();
            return oss.str();
        }
    };

    struct Cycle
    {
        Event planning;
        Event start;
        Event waiting;

        std::string getCycleSummary() const
        {
            std::ostringstream oss;
            oss << "Planning: " << planning.getTimeString()
                << ", Start: " << start.getTimeString()
                << ", Waiting: " << waiting.getTimeString();

            if (planning.isValid && start.isValid && waiting.isValid)
            {
                auto planningToStart = std::chrono::duration_cast<std::chrono::milliseconds>(start.timestamp - planning.timestamp).count();
                auto startToWaiting = std::chrono::duration_cast<std::chrono::milliseconds>(waiting.timestamp - start.timestamp).count();
                auto total = planningToStart + startToWaiting;

                oss << ", Duration (ms): Planning->Start=" << planningToStart
                    << ", Start->Waiting=" << startToWaiting
                    << ", Total=" << total;
            }

            return oss.str();
        }
    };

private:
    std::vector<Cycle> cycles = std::vector<Cycle>(5);
    State currentState = State::Waiting;
    Event currentEvent{};
    Cycle inProgressCycle;
    const size_t MAX_CYCLES = 5;
    bool newCycleAdded = false;
    int  newCycleCountOut = 10;

public:
    void resetCycles()
    {
        cycles = std::vector<Cycle>(5);
    }

    CycleEventTracker()
    {
        currentEvent.state = State::Waiting;
        currentEvent.timestamp = std::chrono::high_resolution_clock::now();
        currentEvent.isValid = true;
    }

    void recordPlanningEvent()
    {
        currentState = State::Planning;
        currentEvent = {State::Planning, std::chrono::high_resolution_clock::now(), true};
        inProgressCycle.planning = currentEvent;
    }

    void recordCycleStartEvent()
    {
        currentState = State::Start;
        currentEvent = {State::Start, std::chrono::high_resolution_clock::now(), true};
        inProgressCycle.start = currentEvent;
    }

    void recordAcknowledgeEvent()
    {
        currentState = State::Waiting;
        currentEvent = {State::Waiting, std::chrono::high_resolution_clock::now(), true};
        inProgressCycle.waiting = currentEvent;

        // rotate the vector to remove the oldest and push latest at the end
        cycles.erase(cycles.begin());
        cycles.push_back(inProgressCycle);
        inProgressCycle = Cycle();
        newCycleAdded = true;
    }

    std::string getCycleStatusColorHex()
    {
        if (newCycleAdded)
        {
            if (newCycleCountOut > 0)
            {
                // Decrease the count and return green
                newCycleCountOut--;
                return "#00FF00"; // Green
            }

            // Reset when cycle count reaches 0
            newCycleAdded = false;
            newCycleCountOut = 10; // Reset for next cycle
        }

        // If no new cycle added or count exhausted, return red
        return "#FF0000"; // Red
    }

    std::string getCurrentStateString() const
    {
        switch (currentState)
        {
        case State::Waiting:
            return "Waiting";
        case State::Planning:
            return "Planning";
        case State::Start:
            return "Start";
        default:
            return "Unknown";
        }
    }

    State getCurrentState() const
    {
        return currentState;
    }

    std::vector<std::string> getLastFivePlanningTimes() const
    {
        return extractLastFour([](const Cycle &c)
                               { return c.planning.getTimeString(); });
    }

    std::vector<std::string> getLastFiveCycleStartTimes() const
    {
        return extractLastFour([](const Cycle &c)
                               { return c.start.getTimeString(); });
    }

    std::vector<std::string> getLastFiveAcknowledgeTimes() const
    {
        return extractLastFour([](const Cycle &c)
                               { return c.waiting.getTimeString(); });
    }

    std::vector<std::string> getLastFiveCycleSummaries() const
    {
        return extractLastFour([](const Cycle &c)
                               { return c.getCycleSummary(); });
    }

    void printLastFourCycles() const
    {
        std::cout << "Last Cycles:\n";
        for (const auto &summary : getLastFiveCycleSummaries())
            std::cout << summary << '\n';
    }

    void printTimes(const std::vector<std::string> &times, const std::string &label) const
    {
        std::cout << label << " Times:\n";
        for (const auto &t : times)
            std::cout << " - " << t << '\n';
    }

private:
    template <typename Func>
    std::vector<std::string> extractLastFour(Func func) const
    {
        std::vector<std::string> results;
        size_t count = std::min(cycles.size(), size_t(3));
        for (size_t i = cycles.size() - count; i < cycles.size(); ++i)
            results.push_back(func(cycles[i]));
        return results;
    }
};

#endif // CYCLE_EVENT_TRACKER_HPP
