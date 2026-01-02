/**
 * @file errorTracker.hpp
 * @author RajKumar Gupta (raj@robrosystems.com)
 * @brief Error Tracker class responsible for tracking and managing errors in the system.
 * @version 1.0
 * @date 2025-02-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef ERROR_TRACKER
#define ERROR_TRACKER

#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <set>
#include <chrono>

#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <deque>

// Enum representing different hardware components type in the system.
enum class ComponentType
{
    SENSOR,
    ENCODER,
    UNKNOWN
};

// Enum representing different hardware components model in the system.
enum class ComponentModel
{
    CUTTER_SENSOR, // Cutter sensor model
    DANCER_SENSOR, // Dancer sensor model
    FRONT_ENCODER, // Front encoder model
    BACK_ENCODER,  // Back encoder  model
    UNKNOWN        // Unknown
};

// Enum representing different error codes associated with the components.
enum class ErrorCode
{
    SHW003, // Cutter Sensor Error
    SHW004, // Front Encoder Error
    SHW009, // Dancer Sensor Error
    SHW011, // Back Encoder Error
    UNKNOWN // Default/Unknown Error
};

// Function to convert an ErrorCode to a string representation.
inline std::string errCodetoString(ErrorCode code)
{
    switch (code)
    {
    case ErrorCode::SHW003:
        return "SHW003";
    case ErrorCode::SHW004:
        return "SHW004";
    case ErrorCode::SHW009:
        return "SHW009";
    case ErrorCode::SHW011:
        return "SHW011";
    default:
        return "UNKNOWN";
    }
}

// Struct representing a Component in the system
struct Component
{
    ComponentType type = ComponentType::UNKNOWN; // Initialize properly
    ComponentModel model = ComponentModel::UNKNOWN;

    // Parameterized Constructor
    Component(ComponentType t, ComponentModel m) : type(t), model(m) {}

    // Default Constructor
    Component() = default;
};

// Struct representing an error that occurred in the system.
struct Error
{
    Component source;     // The component where the error originated
    ErrorCode code;       // The specific error code
    std::string message;  // A descriptive error message
    double recordedMeter; // Running meter at which error is detected

    std::chrono::system_clock::time_point generated_at; // Timestamp when the error was generated

    int cutter_cycle_number{0};

    // Parameterized Constructor to initialize an error instance
    Error(Component src, ErrorCode errCode, const std::string &msg, double runningMeter)
        : generated_at(std::chrono::system_clock::now()), source(src), code(errCode), message(msg), recordedMeter(runningMeter)
    {
    }

    // Default Constructor to create a default error instance
    // Default Constructor
    Error() : source(), code(ErrorCode::UNKNOWN), message("No error"), generated_at(std::chrono::system_clock::now()), recordedMeter(0) {}

    // Function to get the error code as a string
    std::string codeStr() const
    {
        return errCodetoString(code);
    }

    // Overload the less-than operator for Error
    bool operator<(const Error &other) const
    {
        // Compare based on the source.model first
        if (source.model != other.source.model)
            return source.model < other.source.model;
        return false;
    }

    /**
     * @brief Check if the error has timed out based on the given timeout duration
     */
    bool isOld(std::chrono::duration<int> timeout = std::chrono::seconds(60)) const
    {
        return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - generated_at) > timeout;
    }

    bool is_stored{false}; // Indicate that error is stored or not;
};

// Class responsible for tracking and managing errors in the system.
class ErrorTracker
{
private:
    // Struct for storing error details and tracking distance the error presist.
    struct ErrorInfo
    {
        Error err;              // The error instance
        double startMeter;      // Initial meter value when the error was first detected
        double popupStartMeter; // Meter value when popup tracking started
        double stopStartMeter;  // Meter value when stop tracking started
        double lastMeter;       // The latest meter value error is detected

        bool is_stopped_issued{false}; // Indicate stop is initiated or not

        // Function to calculate the popup distance
        double getPopupDistance() const { return lastMeter - popupStartMeter; }

        // Function to calculate the stopping distance
        double getStopDistance() const { return lastMeter - stopStartMeter; }

        // Function to calculate the error distance
        double getDistance() const { return lastMeter - startMeter; }
    };

    std::unordered_map<ErrorCode, ErrorInfo> sensorActiveErrors; // Stores currently active sensor errors
    std::deque<ErrorInfo> encodersActiveErrors;                  // Stores currently active encoder errors

    std::set<Error> sensorCurrentErrors;                       // Stores current added errors, unqiue entry
    std::unordered_map<ErrorCode, Error> encoderCurrentErrors; // Stores current added errors, unqiue entry

    double popupThreshold = 5.0;     // Default threshold for popup warnings
    double stoppingThreshold = 20.0; // Default threshold for stopping the system

    bool stopped_for_encoder_error{false};                             // Indicate system stopped for an encoder error
    ComponentModel stopped_for_encoder_model{ComponentModel::UNKNOWN}; // Indicate system stopped for which encoder component (front/back)

    int cutter_cycle_count{0};

public:
    ErrorTracker(double popupThresh = 5.0, double stoppingThresh = 20.0)
        : popupThreshold(popupThresh), stoppingThreshold(stoppingThresh) {}

    // reset Error Tracker
    void reset()
    {
        // clear any active and current errors
        sensorActiveErrors.clear();
        encodersActiveErrors.clear();
        sensorCurrentErrors.clear();
        encoderCurrentErrors.clear();
        cutter_cycle_count = 0;
    }

    // Adds a new error to the tracker
    void addError(Error &error)
    {
        switch (error.source.type)
        {
        case ComponentType::SENSOR:
        {
            error.cutter_cycle_number = cutter_cycle_count;
            sensorCurrentErrors.insert(error);
            break;
        }
        case ComponentType::ENCODER:
        {
            if (encoderCurrentErrors.find(error.code) == encoderCurrentErrors.end())
            {
                error.cutter_cycle_number = cutter_cycle_count;
                encoderCurrentErrors[error.code] = error;
            }
            else
            {
                encoderCurrentErrors[error.code].cutter_cycle_number = cutter_cycle_count;
                // Existing error entry, update timestamp generation of error
                encoderCurrentErrors[error.code].generated_at = error.generated_at;
            }
            break;
        }
        default:
            break;
        }
    }

    // Set the threshold for pop-up warnings
    void setPopupThreshold(double threshold)
    {
        popupThreshold = threshold;
    }

    // Set the threshold for stopping the system
    void setStoppingThreshold(double threshold)
    {
        stoppingThreshold = threshold;
    }

    // Updates the active error list based on the error log and current running meter, sensor on states (cutter/dancer)
    void update(double runningMeter, bool is_cutter_sensor_on = false, bool is_dancer_sensor_on = false)
    {
        // If cutter sensor is on, one cycle is completed, clear error in previous cycle and remove cutter sensor error exist
        if (is_cutter_sensor_on)
        {
            // clear encoder error
            encoderCurrentErrors.clear();

            // Increase cutter cycle count
            cutter_cycle_count += 1;
        }

        // If dancer sensor is on, reset encoder error stopping state
        if (is_dancer_sensor_on)
        {
            stopped_for_encoder_error = false;
            stopped_for_encoder_model = ComponentModel::UNKNOWN;
        }

        // Process new errors in the log
        for (const auto &err : sensorCurrentErrors)
        {
            if (sensorActiveErrors.find(err.code) == sensorActiveErrors.end())
            {
                // New error: Add it to active errors
                ErrorInfo errInfo;
                errInfo.err = err;
                errInfo.popupStartMeter = runningMeter;
                errInfo.stopStartMeter = runningMeter;
                errInfo.startMeter = runningMeter;
                errInfo.lastMeter = runningMeter;
                sensorActiveErrors[err.code] = errInfo;
            }
            else
            {
                // Existing error: Update its current meter value
                sensorActiveErrors[err.code].lastMeter = runningMeter;
            }
        }
        sensorCurrentErrors.clear();

        // Remove errors that have been resolved (not detected in the latest meter value / loop)
        for (auto it = sensorActiveErrors.begin(); it != sensorActiveErrors.end();)
        {
            switch (it->second.err.source.model)
            {
            case (ComponentModel::CUTTER_SENSOR):
            {
                // Don't do any thing in case of cutter sensor, remove only if cutter is on
                if (is_cutter_sensor_on)
                {
                    it = sensorActiveErrors.erase(it);
                }
                else
                {

                    ++it;
                }
                break;
            }
            case (ComponentModel::DANCER_SENSOR):
            {
                // Remove only if it is not observed in past stoppingThreshold or dancer sensor is on
                if (runningMeter - it->second.lastMeter >= stoppingThreshold || is_dancer_sensor_on)
                {
                    it = sensorActiveErrors.erase(it);
                }
                else
                {
                    ++it;
                }
                break;
            }
            default:
            {
                ++it;
                break;
            }
            }
        }

        // In case if encoder error is changed
        if (!encodersActiveErrors.empty())
        {
            const auto &backError = encodersActiveErrors.back();

            // If last entry is 5 cutter cycle count older remove it
            if (std::abs(cutter_cycle_count - backError.err.cutter_cycle_number) >= 5)
            {
                // remove previous entries
                std::deque<ErrorInfo> empty_queue;
                encodersActiveErrors.swap(empty_queue);
            }

            // Check if any active error 
            if (!encodersActiveErrors.empty() && !encoderCurrentErrors.empty())
            {
                const auto &backError = encodersActiveErrors.back();
                const auto &frontCurrentError = encoderCurrentErrors.begin()->second;
                // If the new encoder is from different model, remove previous active error
                if (backError.err.source.model != frontCurrentError.source.model)
                {
                    // remove previous entries
                    std::deque<ErrorInfo> empty_queue;
                    encodersActiveErrors.swap(empty_queue);
                }
            }
        }

        // Process new errors in the log
        for (auto &err_p : encoderCurrentErrors)
        {
            if (!err_p.second.is_stored)
            {
                // New error: Add it to active errors
                ErrorInfo errInfo;
                errInfo.err = err_p.second;
                errInfo.popupStartMeter = err_p.second.recordedMeter;
                errInfo.stopStartMeter = err_p.second.recordedMeter;
                errInfo.startMeter = err_p.second.recordedMeter;
                errInfo.lastMeter = err_p.second.recordedMeter;
                err_p.second.is_stored = true;
                encodersActiveErrors.push_back(errInfo);
            }
        }
    }

    // Determines if an error has exceeded the 5-meter threshold for warning (pop-up alert).
    bool shouldPop(Error &error, double runningMeter)
    {
        for (auto &actErr : sensorActiveErrors)
        {

            switch (actErr.second.err.source.model)
            {
            case ComponentModel::CUTTER_SENSOR:
            {
                return false;
                break;
            }
            case ComponentModel::DANCER_SENSOR:
            {
                if (actErr.second.getPopupDistance() >= popupThreshold)
                {
                    error = actErr.second.err;                    // Return the error instance
                    actErr.second.popupStartMeter = runningMeter; // Reset start meter
                    return true;
                }
                break;
            }
            default:
                break;
            }
        }
        // if (encodersActiveErrors.size() < 5 && encodersActiveErrors.size() >= 3)
        // {
        //     error = encodersActiveErrors.begin()->err;
        //     return true;
        // }
        return false;
    }

    // Determines if an error has exceeded the stop threshold for stopping the system.
    bool shouldStop(Error &error, double runningMeter)
    {
        // Check for any active encoder error
        if (encodersActiveErrors.size() >= 3)
        {
            error = encodersActiveErrors.begin()->err;
            stopped_for_encoder_error = true;
            stopped_for_encoder_model = error.source.model;
            encodersActiveErrors.clear();
            return true;
        }

        // Check for any active sensor error
        for (auto &actErr : sensorActiveErrors)
        {
            switch (actErr.second.err.source.model)
            {
            case ComponentModel::CUTTER_SENSOR:
            {
                if (!actErr.second.is_stopped_issued)
                {
                    actErr.second.is_stopped_issued = true;
                    error = actErr.second.err;
                    return true;
                    break;
                }
            }
            case ComponentModel::DANCER_SENSOR:
            {
                if (actErr.second.getStopDistance() >= stoppingThreshold)
                {
                    error = actErr.second.err;                   // Return the error instance
                    actErr.second.stopStartMeter = runningMeter; // Reset start meter
                    return true;
                }
                break;
            }

            default:
                break;
            }
        }
        return false;
    }

    // Determines the indicator color for a given component based on error tracking.
    std::string getComponentIndicatorColor(ComponentModel model)
    {
        // Helper function to check for active errors for the given model
        auto hasActiveError = [this](const ComponentModel &model)
        {
            return std::find_if(sensorActiveErrors.begin(), sensorActiveErrors.end(),
                                [this, model](const auto &pair)
                                {
                                    return pair.second.err.source.model == model;
                                }) != sensorActiveErrors.end();
        };

        // Helper function to get the error distance (only needed for specific components)
        auto getErrorDistance = [this](const ComponentModel &model)
        {
            auto it = std::find_if(sensorActiveErrors.begin(), sensorActiveErrors.end(),
                                   [this, model](const auto &pair)
                                   {
                                       return pair.second.err.source.model == model;
                                   });
            return it != sensorActiveErrors.end() ? it->second.getDistance() : 0.0; // Return 0 if no error
        };

        // Handle specific models
        switch (model)
        {
        case ComponentModel::CUTTER_SENSOR:
            if (hasActiveError(model))
            {
                return "#FF0000"; // Red if error exists
            }
            break;

        case ComponentModel::DANCER_SENSOR:
            if (hasActiveError(model))
            {
                // Return red if error distance exceeds the threshold, otherwise yellow
                return (getErrorDistance(model) >= stoppingThreshold) ? "#FF0000" : "#FFFF00";
            }
            break;

        case ComponentModel::FRONT_ENCODER:
        case ComponentModel::BACK_ENCODER:
        {
            if (stopped_for_encoder_error)
            {
                // Check if we stopped for any encoder model
                if (stopped_for_encoder_model == model)
                    return "#FF0000"; // Red if stopped and reset after dancer sensor on next time
            }

            // Check if the error exists for the current encoder
            if (!encodersActiveErrors.empty() && encodersActiveErrors.begin()->err.source.model == model)
            {
                if (encodersActiveErrors.size() >= 3)
                {
                    return "#FF0000"; // Red if there are 5 or more errors
                }
                else if (encodersActiveErrors.size() > 0)
                {
                    return "#FFFF00"; // Yellow if there are some errors
                }
            }
            break;
        }

        default:
            break;
        }

        return ""; // Return empty if no active error
    }
};

#endif