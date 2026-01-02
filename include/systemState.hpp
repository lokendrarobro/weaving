#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <ctime>

// Structure for defining states
struct SystemState
{
    std::string name;
    int code;
};

// Class for tracking system states
class SystemStateTracker
{
public:
    // Constructor that accepts custom states and log interval
    SystemStateTracker(const std::vector<SystemState> &custom_states,
                       std::chrono::seconds log_interval = std::chrono::seconds(180)) // Default: 3 min interval
    {
        states = custom_states;
        logInterval = log_interval;

        // Setting Default State.
        SystemState default_state = {"SYSTEM_OFF", 0};
        current_state = default_state.name;
        
        last_change_time = std::chrono::steady_clock::now();
        interval_start_time = last_change_time;                     // Initialize interval timer
        state_start_time = getCurrentDateTime("%Y-%m-%d %H:%M:%S"); // Initialize start time in human-readable format
        state_id = generateNewStateId();                            // Generate the first state ID
    }

    // Changes the state and generates a new unique state_id
    bool changeState(const std::string &new_state)
    {
        auto now = std::chrono::steady_clock::now();
        int current_state_code = findStateCode(current_state);
        int new_state_code = findStateCode(new_state);

        if (new_state_code != -1)
        {
            if (current_state_code != new_state_code)
            {
                current_state = new_state;
                last_change_time = now;
                interval_start_time = now;                                  // Reset the interval timer
                state_start_time = getCurrentDateTime("%Y-%m-%d %H:%M:%S"); // Update start time in human-readable format
                state_id = generateNewStateId();                            // Generate a new state ID
                return true;
            }
            else
            {
                return false;
            }
        }
        std::cerr << "State not recognized: " << new_state << "\n";
        return false;
    }

    std::string getStateDuration() const
    {
        // Get the current time
        auto now = std::chrono::steady_clock::now();

        // Calculate the duration since the state started
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_change_time);

        // Convert the duration to hours, minutes, and seconds
        int hours = static_cast<int>(duration.count() / 3600);
        int minutes = static_cast<int>((duration.count() % 3600) / 60);
        int seconds = static_cast<int>(duration.count() % 60);

        // Format the result as HH:MM:SS
        std::ostringstream oss;
        oss << std::setw(2) << std::setfill('0') << hours << ":"
            << std::setw(2) << std::setfill('0') << minutes << ":"
            << std::setw(2) << std::setfill('0') << seconds;

        return oss.str();
    }

    // Get the current state ID (unique for each state change)
    std::string getCurrentStateId() const
    {
        return state_id;
    }

    // Get the current state code (associated with the state name)
    int getCurrentStateCode() const
    {
        return findStateCode(current_state);
    }

    // Get the start time of the current state
    std::string getStateStartTime() const
    {
        return state_start_time;
    }

    std::string getStateEndTime() const
    {
        return getCurrentDateTime("%Y-%m-%d %H:%M:%S");
    }

    // Check if log interval is over and reset the timer
    bool checkLogTimeout()
    {
        auto now = std::chrono::steady_clock::now();
        if ((now - interval_start_time) >= logInterval)
        {
            interval_start_time = now; // Reset the interval timer
            return true;               // Interval time has passed
        }
        return false; // Interval time not yet passed
    }

private:
    std::string current_state;
    std::string state_id;            // Unique state ID generated for each state change
    std::string state_start_time;    // The start time of the current state
    std::vector<SystemState> states; // List of valid states

    std::chrono::steady_clock::time_point last_change_time;
    std::chrono::steady_clock::time_point interval_start_time; // For logging interval
    std::chrono::seconds logInterval;                          // Interval for checking if logging is needed

    // Utility function to find the state code by name
    int findStateCode(const std::string &state_name) const
    {
        for (const auto &state : states)
        {
            if (state.name == state_name)
            {
                return state.code; // Return the state code if the state is found
            }
        }
        return -1; // Return -1 if the state is not found
    }

    // Generate a new unique state ID using state name and current date-time (without - or :)
    std::string generateNewStateId()
    {
        return current_state + "_" + getCurrentDateTime("%Y%m%d_%H%M%S");
    }

    // Get the current date-time in the format provided (use "%Y%m%d_%H%M%S" for state_id)
    std::string getCurrentDateTime(const std::string &format) const
    {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        std::string s(30, '\0');
        std::strftime(&s[0], s.size(), format.c_str(), std::localtime(&now));
        s.resize(std::strlen(s.c_str())); // Resize to remove any extra null characters
        return s;
    }
};
