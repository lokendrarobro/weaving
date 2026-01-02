/**
 * @file cutterPLCComm.hpp
 * @author Sanyam Bhawsar (sanyam@robrosystems.com)
 * @brief Blue print of cutting machine controller class
 * @version 1.0.0
 * @date 2024-02-05
 *
 * @copyright Copyright (c) 2023-2029
 *
 */
#ifndef CUTTER_PLC_COMM_HPP
#define CUTTER_PLC_COMM_HPP

#include <robroModbus.h>
#include <iostream>
#include <json.hpp>
#include <iomanip>
/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Class to handle communication with the cutting machine
 *
 */
class CuttingMachinePLCComm
{

private:
    /**
     * @brief Simulation mode. This is used to run the application in simulation mode.
     * In simulation mode, the application will not communicate with the PLC.
     */
    bool simulation_mode{true};

    /**
     * @brief Store the latest command sent is successful with PLC or not in any case.
     */
    bool is_last_cmd_success{false};

    /**
     * @brief Store the last value of machine ready
     */
    bool machine_ready_last_state{false};

    /**
     * @brief Map to map human readable strings to hex addresses in PLC. We can use this map
     * to read/write values from/to PLC. Which can be used to control the machine.
     */
    std::map<std::string, int> plc_addresses;

    /**
     * @brief Cutting machine configuration parameters. Which contains the addresses of the
     * registers and bits in the PLC and connection parameters to communicate with the PLC.
     */
    json cutting_machine_config;

    /**
     * @brief Object to communicate with the PLC over Serial port.
     * It uses libmodbus library to communicate with the PLC.
     * It provides functions to read/write bits and registers from/to PLC.
     */
    RobroModbusLib *cutter_comm{nullptr};

    /**
     * @brief cutting machine start is register or not
     *
     */
    bool is_cycle_start_register{false};

    /**
     * @brief cutting machine ready is register or not
     */
    bool is_machine_ready_register{false};

    /**
     * @brief cutting machine spout status is register or not.
     */

    bool is_spout_status_register{false};

  
    bool is_cycle_start_electrical{false};  // use the electrical instead of register for cycle start


public:
    /**
     * @brief Destroy a the Cutter PLC Comm object
     */
    ~CuttingMachinePLCComm();

    /**
     * @brief Set the PLC Communication and Address Configuration.
     *
     * @param config - Configuration parameters to communicate with the PLC.
     * @return true - if all the parameters are read successfully.
     * @return false - otherwise.
     */
    bool setPLCCommAndAddresses(json config);

    /**
     * @brief set the required length of the fabric to be cut to machine plc. and start the machine cycle.
     *
     * @param body_length - Length of the fabric to be cut.
     * @return true - if the fabric cut length is set successfully.
     * @return false - otherwise.
     */
    bool setNextFabricCutLength(uint16_t body_length);

    /**
     * @brief Set the Spout Mode object
     *
     * @param spout_mode - Mode of the spout.
     * @return true - if the spout mode is set successfully.
     * @return false - otherwise.
     */
    bool setSpoutMode(bool spout_mode);

    /**
     * @brief Get the machine ready status.
     *
     * @return true - if the machine is ready to cut the fabric.
     * @return false - otherwise.
     */
    bool getMachineReadyState();

    /**
     * @brief Get the current cutter PLC Address and Communication Information.
     *
     * @return json - Current cutter PLC address and communication information.
     */
    json getCutterPLCAddressAndCommInfo();

    /**
     * @brief Check if the last command was successful.
     *
     * @return true - If the last command was successful.
     * @return false - If the last command was not successful.
     */
    bool getIsLastCmdSuccess();

    /**
     * @brief Write Cycle Start
     *
     * @return true - If the last command was successful.
     * @return false - If the last command was not successful.
     */
    bool writeCycleStart();


    /**
     * @brief get Cycle Start
     *
     * @return
     */
     bool getCycleStartState();


    /**
     * @brief Write Cut Length Value in Register
     *
     * @return true - If the last command was successful.
     * @return false - If the last command was not successful.
     */
    bool writeCutLengthRegister(uint16_t body_length);
};

#endif // CUTTER_PLC_COMM_HPP