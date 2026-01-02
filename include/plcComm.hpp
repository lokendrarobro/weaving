/**
 * @file plcComm.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Blue print of Plc communication for Weaving inspection application.
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLC_COMM_HPP
#define PLC_COMM_HPP

#include <robroModbus.h>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <json.hpp>

using json = nlohmann::json;
/**
 * @brief Class to handle communication with the PLC
 *
 */
class PLCComm
{
    /**
     * @brief Map to map human readable strings to hex addresses in PLC. We can use this map
     * to read/write values from/to PLC. Which can be used to control the machine.
     */
    std::map<std::string, int> plc_addresses;

    /**
     * @brief Object to communicate with the PLC over Serial port.
     * It uses libModbus library to communicate with the PLC.
     * It provides functions to read/write bits and registers from/to PLC.
     */
    RobroModbusLib *plcComm;

    /**
     * @brief Serial port address to communicate with the PLC.
     */
    std::string port_address{""};

    /**
     * @brief Parity to communicate with the PLC.
     */
    std::string parity{"E"};

    /**
     * @brief Baud rate to communicate with the PLC.
     */
    int baud{9600};

    /**
     * @brief Data bits to communicate with the PLC.
     */
    int data_bit{8};

    /**
     * @brief Stop bits to communicate with the PLC.
     */
    int stop_bit{1};

    /**
     * @brief Slave address of the plc.
     */
    int slave_address{1};

    /**
     * @brief PLC counter increment target in mm. This is the distance in mm for which the PLC
     * counter will be incremented. This value is set in PLC.
     */
    uint16_t plc_counter_increment_target_mm;

    /**
     * @brief Simulation mode. This is used to run the application in simulation mode.
     * In simulation mode, the application will not communicate with the PLC.
     */
    bool simulation_mode{false};

    /**
     * @brief Is PLC RTU connection available. This flag is used to check if the RTU connection infomation
     * is available or not.
     */
    bool is_rtu_conn_available{false};

    /**
     * @brief Is PLC TCP connection available. This flag is used to check if the TCP connection infomation
     * is available or not.
     */
    bool is_tcp_conn_available{false};

    /**
     * @brief Store the latest command sent is successful with PLC or not in any case.
     */
    bool is_last_cmd_success{true};


    /**
     * @brief Store the last value of machine ready
     */
    bool machine_ready_last_state{false};

    /**
     * @brief Is PLC cutter connection is normally closed. This flag is used to check if the cutter
     * connection is normally closed or not. means if the cutter is off it returns true.
     * If the cutter is on it returns false.
     */
    bool is_cutter_conn_nc{false};

    /**
     * @brief when this is ON at cutter action we will reset the current body processed under plc it will not be reset by plc.
     */
    bool current_body_processed_auto_reset_mode{false};

    /**
     * @brief flag for current_body_processed reset mode will be set when the use requested and the plc program version is compatable.
     */
    bool reset_current_body_processed{false};

    /**
     * @brief PLC pulse per mm. Which is used to convert the distance from PLC
     * encoder pulses to real world.
     */
    float plc_pulse_per_mm;

    /**
     * @brief PLC stopper counter which use to mimic the count down the inside @fn requestToStopMM
     * in simulation mode separate thread also use to check is @fn isStoppingDone or not.
     */
    int stopper_counter{0};

    /**
     * @brief PLC TCP port number where the PLC is listening for the TCP connection.
     */
    int plc_tcp_port;

    /**
     * @brief PLC TCP IP address where the PLC is listening for the TCP connection.
     */
    std::string plc_tcp_ip;

public:
    /**
     * @brief Construct a new PLCComm object
     */
    PLCComm();

    /**
     * @brief Destroy the PLCComm object
     */
    ~PLCComm();

    /**
     * @brief It reads the parameters from the ros parameter server and
     * stores them in the class variables.
     *
     * @return true - if parameters are read successfully.
     * @return false - otherwise.
     */
    bool readROSParams();

    /**
     * @brief It initializes the PLC
     *
     */
    void initPLC();

    /**
     * @brief read the plc program version from the plc
     * @return value - version of the plc program
     */
    uint16_t getPLCProgramVersion();

    /**
     * @brief disabling current body processed auto reset mode
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setCurrentBodyProcessedResetModeOFF();

    /**
     * @brief get the current_body_processed_reset_mode
     * @return true - if the mode is ON
     * @return false - if the mode is OFF
     */
    bool getCurrentBodyProcessedResetMode();

    /**
     * @brief Reset the PLC values by turning flag on that'll reset all values in PLC
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool resetPLCValues();

    /**
     * @brief Reset the current body processed.
     * @return true - if the request was successful
     * @return false - if the reques failed
     */
    bool resetCurrentBodyProcessed();

    /**
     * @brief Set the Green Light to ON/OFF.
     *
     * @param state - true to turn on the green light.
     *              - false to turn off the green light.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setGreenLightTo(bool state);


    /**
     * @brief Set the Yellow Light to ON/OFF.
     *
     * @param state - true to turn on the yellow light.
     *              - false to turn off the yellow light.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setYellowLightTo(bool state);

    /**
     * @brief Set the Red Light to ON/OFF.
     *
     * @param state - true to turn on the red light.
     *              - false to turn off the red light.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setRedLightTo(bool state);

    /**
     * @brief Set the Inspection Light to ON/OFF.
     *
     * @param state - true to turn on the inspection light.
     *              - false to turn off the inspection light.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setInspectionLightTo(bool state);

    /**
     * @brief Set the Induction Motor To state
     *
     * @param state - true to turn on the induction motor.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setInductionMotorTo(bool state);

    /**
     * @brief Set the Auto Mode To state
     *
     * @param state - true to turn on the auto mode.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setAutoModeTo(bool state);

    /**
     * @brief Set the Cycle Start To state
     *
     * @param state - true to turn on the cycle start.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setCycleStartTo(bool state);


    /**
     * @brief Get if Machine is available or not
     *
     * @return true - if available
     * @return false - if not available
     */
    bool getMachineAvailableState();

    /**
     * @brief Set the Buzzer To to ON/OFF.
     *
     * @param state - true to turn on the buzzer.
     *              - false to turn off the buzzer.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setBuzzerTo(bool state);

    /**
     * @brief Set the Punch Blocking to ON/OFF for number of beeps.
     *
     * @param number_of_beeps - number of beeps to be made by the buzzer.
     *
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool beepTheBuzzer(int number_of_beeps);

    /**
     * @brief Write the cut length to the PLC and also write if the punching is on or not.
     *
     * @param cut_length - cut length in mm.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool writeCutLength(uint16_t cut_length);

    /**
     * @brief It will turn on the PLC flag to calculate the length of the fabric.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool calculateLengths();

    /**
     * @brief Reset the frame counters and the fabric moved under camera counters.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool resetFrameCounters();

    /**
     * @brief Request the PLC to stop the machine by setting next stopping distance.
     *
     * @param next_stopping_distance_mm - next stopping distance in mm.
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool requestToStopMM(uint16_t next_stopping_distance_mm);

    /**
     * @brief Check if stopping operation is done or not.
     *
     * @return true - if stopping operation is done by PLC.
     * @return false - if PLC is still busy.
     */
    bool isStoppingDone();

    /**
     * @brief Check if constant distance sensor is on or not.
     *
     * @return true - if constant distance sensor is on.
     * @return false - if constant distance sensor is off.
     */
    bool isConstantDistanceSensorOn();

    /**
     * @brief Check if cutter is on or not.
     *
     * @return true - if cutter is on.
     * @return false - if cutter is off.
     */
    bool isCutterOn();

    /**
     * @brief Request the PLC to make punch blocking enabled or disabled
     *
     * @param state - true to enable punch blocking, false to disable
     * @return true - if the request was successful
     * @return false - if the request failed
     */
    bool setPunchBlockedTo(bool state);

    /**
     * @brief Get the mm since Defect from PLC register.GetCurrentPulses
     * This is the mm since the last defect was found.
     *
     * @return uint64_t - mm since defect from PLC.
     */
    uint64_t getMMSinceDefect();

    /**
     * @brief Get the Current Body Processed in mm from PLC register.
     *
     * @return int - current body processed in mm from PLC.
     */
    uint64_t getCurrentBodyProcessedMM();

    /**
     * @brief Get the Fabric Moved Under cutter in mm from PLC register.
     * After the fabric is cut, the fabric moved under cutter mm are calculated.
     *
     * @return int - fabric moved under cutter in mm from PLC.
     */
    bool getFabricMovedUnderCutterMM(uint64_t &value);

    /**
     * @brief To reset the PLC values by reseting the plc state, punch blocking,
     * and reseting the plc counters.
     */
    void resetPLC();

    /**
     * @brief Get the Last Cmd Success state
     *
     * @return true - if the last command was successful
     * @return false - if the last command was failed
     */
    bool getLastCmdSuccess();

    /**
     * @brief Set spoutcut enable or disable during setcutbeforestart
     *
     * @param state - true to enable spout during cutbeforestart
     * @return true - if request successful.
     * @return false - if request failure.
     */
    bool setCCutModeTo(bool state);

    /**
     * @brief turn spout and cutter before start
     *
     * @param state - true to enable cutter spout is depend on ccutmode
     * @return true - if request successfull.
     * @return false - if request failure.
     */
    bool setCutBeforeStartTo(bool state);
};


#endif 