#pragma once

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <ctime>
#include <cstring>

#include <mysql/mysql.h>
#include <logMessage.hpp>

// Class for tracking system states and logging to MySQL
class componentStateTrackerLogger
{
public:
    componentStateTrackerLogger(MySQLClient *mysql_client,
                                std::chrono::seconds log_interval = std::chrono::seconds(180))
        : logger(mysql_client), logInterval(log_interval)
    {

        // Setting Default State.
        state = ComponentState::AVAILABLE_;

        last_change_time = std::chrono::steady_clock::now();
        interval_start_time = last_change_time;
        state_start_time = getCurrentDateTime("%Y-%m-%d %H:%M:%S");
        state_id = logNewStateEntry();
    }

    bool changeState(const ComponentState &new_state)
    {
        auto now = std::chrono::steady_clock::now();
        if (new_state == -1)
        {
            std::cerr << "Unknown state: " << new_state << "\n";
            return false;
        }

        if (state != new_state)
        {
            // Transition
            state = new_state;
            last_change_time = now;
            interval_start_time = now;
            state_start_time = getCurrentDateTime("%Y-%m-%d %H:%M:%S");

            // Log new state entry
            state_id = logNewStateEntry();
            return true;
        }
        return false;
    }

    std::string getStateDuration() const
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_change_time);

        int hours = static_cast<int>(duration.count() / 3600);
        int minutes = static_cast<int>((duration.count() % 3600) / 60);
        int seconds = static_cast<int>(duration.count() % 60);

        std::ostringstream oss;
        oss << std::setw(2) << std::setfill('0') << hours << ":"
            << std::setw(2) << std::setfill('0') << minutes << ":"
            << std::setw(2) << std::setfill('0') << seconds;

        return oss.str();
    }

    int64_t getCurrentStateId() const
    {
        return state_id;
    }
    std::string getStateStartTime() const
    {
        return state_start_time;
    }

    std::string getStateEndTime() const
    {
        return getCurrentDateTime("%Y-%m-%d %H:%M:%S");
    }

    bool checkLogTimeout()
    {
        auto now = std::chrono::steady_clock::now();
        if ((now - interval_start_time) >= logInterval)
        {
            interval_start_time = now;
            return true;
        }
        return false;
    }

    std::string getStateName(ComponentState state) {
        switch (state) {
            case AVAILABLE_: return "AVAILABLE";
            case CONNECTED_: return "CONNECTED";
            case RUNNING_:   return "RUNNING";
            case FAULT_:     return "FAULT";
            default:         return "UNKNOWN";
        }
    }

    int logNewStateEntry()
    {
      state_id=  logger->add_state_log(
            getAppID(),
            LogComponent::APP,
            state,
            getStateStartTime(),
            getStateEndTime(),
            getStateDuration(),
            "NA",
            "System Is " + getStateName(state));
        return state_id;
    }

    void updateStateLog(int64_t state_id)
    {
        logger->update_state_log(state_id,
                            getStateEndTime(),
                            getStateDuration());
    }
    int getLastStateId(){
        return state_id;
    }

private:
    ComponentState state;
    int64_t state_id;
    std::string state_start_time;
    MySQLClient *logger;
   
    std::chrono::steady_clock::time_point last_change_time;
    std::chrono::steady_clock::time_point interval_start_time;
    std::chrono::seconds logInterval;

    std::string getCurrentDateTime(const std::string &format) const
    {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::string s(30, '\0');
        std::strftime(&s[0], s.size(), format.c_str(), std::localtime(&now));
        s.resize(std::strlen(s.c_str()));
        return s;
    }
};
