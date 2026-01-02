#ifndef BODY_TOWER_LIGHT_HPP
#define BODY_TOWER_LIGHT_HPP

#include <plcComm.hpp> // Include the PLCCommunication header

class BodyTowerLight {
private:
    PLCComm* plcComm;  // PLCCommunication object for controlling the lights
    bool isEnabled;    // Flag to check if the system is enabled

public:
    // Constructor that accepts a PLCCommunication object
    BodyTowerLight(PLCComm* plcComm)
        : plcComm(plcComm), isEnabled(true) {}

    // Destructor
    ~BodyTowerLight() {}

    // Function to reset all lights
    void resetLights() {
        if (!isEnabled) return;
        setGreenLightTo(false);
        setYellowLightTo(false);
        setRedLightTo(false);
    }

    // Function to set the body tower light color based on the type
    void setBodyColor(const std::string& type) {
        if (!isEnabled) return;
        resetLights();

        if (type == "P") {
            return;
        } else if (type == "S") {
            setGreenLightTo(true);
        } else if (type == "T") {
            setYellowLightTo(true);
        } else if (type == "D") {
            setRedLightTo(true);
        } else {
            ROS_ERROR("Invalid light type! Valid types are 'P', 'S', 'T', 'D'.");
        }
    }

    // Helper function to turn on/off the green light
    bool setGreenLightTo(bool state) {
        if (!isEnabled) return false;
        return plcComm->setGreenLightTo(state);
    }

    // Helper function to turn on/off the yellow light
    bool setYellowLightTo(bool state) {
        if (!isEnabled) return false;
        return plcComm->setYellowLightTo(state);
    }

    // Helper function to turn on/off the red light
    bool setRedLightTo(bool state) {
        if (!isEnabled) return false;
        return plcComm->setRedLightTo(state);
    }

    // Turn off the green light explicitly
    bool turnOffGreenLight() {
        if (!isEnabled) return false;
        return plcComm->setGreenLightTo(false);
    }

    // Turn off the yellow light explicitly
    bool turnOffYellowLight() {
        if (!isEnabled) return false;
        return plcComm->setYellowLightTo(false);
    }

    // Turn off the red light explicitly
    bool turnOffRedLight() {
        if (!isEnabled) return false;
        return plcComm->setRedLightTo(false);
    }

    // Check the current status of the lights (this now assumes PLC has status access or is just informative)
    void checkLightStatus() const {
        ROS_INFO("CheckLightStatus called. Actual light status should be queried from PLC if needed.");
    }

    // Enable the body tower light system
    void enable() {
        isEnabled = true;
        ROS_INFO("Body tower light system enabled.");
    }

    // Disable the body tower light system
    void disable() {
        isEnabled = false;
        resetLights();
        ROS_INFO("Body tower light system disabled.");
    }

    // Check if the system is enabled
    bool isSystemEnabled() const {
        return isEnabled;
    }
};

#endif // BODY_TOWER_LIGHT_HPP
