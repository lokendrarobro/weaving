/**
 * @file cutterPLCComm.cpp
 * @author Sanyam Bhawsar (sanyam@robrosystems.com)
 * @brief Class to communicate with FIBC Cutting machine and control it to cut the fabric of required length.
 * @version 0.1
 * @date 2024-02-01
 *
 * @copyright Copyright (c) 2023-2029
 *
 */

#include <cuttingMachinePLCComm.hpp>

/**
 * @brief Destroy a the Cutter PLC Comm object
 */
CuttingMachinePLCComm::~CuttingMachinePLCComm()
{
    if (cutter_comm != nullptr)
    {
        delete cutter_comm;
    }
}

/**
 * @brief Set PLC Communication and Address Configuration.
 *
 * @param config - Configuration parameters to communicate with the PLC.
 * @return true - if all the parameters are read successfully.
 * @return false - otherwise.
 */
bool CuttingMachinePLCComm::setPLCCommAndAddresses(json config)
{
    try
    {
        std::cout << "Cutting Machine Controller Configuration: " << std::endl;
        std::cout << std::setw(4) << config << std::endl;
        if (cutter_comm != nullptr)
        {
            delete cutter_comm;
        }
        if (config.contains("addresses"))
        {
            // get the address of the PLC.
            plc_addresses.clear();
            json cutting_machine_addresses = config["addresses"];
            for (auto it = cutting_machine_addresses.begin(); it != cutting_machine_addresses.end(); ++it)
            {
                std::string addressName = it.key();
                std::string hexString = it.value();
                int intValue = std::stoi(hexString, 0, 16); // Convert hex string to int
                plc_addresses[addressName] = intValue;
            }

            cutting_machine_config = config;

            // check port address, baud rate, parity, data bit and stop bit.
            if (config.contains("port_address") &&
                config.contains("baud") &&
                config.contains("parity") &&
                config.contains("data_bit") &&
                config.contains("stop_bit") &&
                config.contains("slave_address") &&
                config.contains("simulation_mode"))
            {
                // simulation mode
                simulation_mode = config["simulation_mode"];
                if (simulation_mode)
                {
                    return true;
                }

                // checking if tcp connection is available.
                bool is_tcp_conn_available = false;
                int tcp_port;
                std::string tcp_ip;
                if (config.contains("tcp_port") &&
                    config.contains("tcp_ip"))
                {
                    is_tcp_conn_available = true;
                    tcp_ip = config["tcp_ip"].get<std::string>();
                    tcp_port = config["tcp_port"].get<int>();
                }

                // get the port address, baud rate, parity, data bit and stop bit.
                std::string port_address = config["port_address"].get<std::string>();
                int baud = config["baud"].get<int>();
                std::string parity = config["parity"].get<std::string>();
                int data_bit = config["data_bit"].get<int>();
                int stop_bit = config["stop_bit"].get<int>();
                int slave_address = config["slave_address"].get<int>();
                simulation_mode = config["simulation_mode"];

                if (is_tcp_conn_available)
                {
                    cutter_comm = new RobroModbusLib(tcp_ip, tcp_port);
                }
                else
                {
                    cutter_comm = new RobroModbusLib(port_address, baud, parity[0], data_bit, stop_bit);
                }

                if (cutter_comm != nullptr)
                {
                    cutter_comm->setSlaveAddress(slave_address);
                    cutter_comm->setAddresses(plc_addresses);
                    is_last_cmd_success = true;
                }
                else
                {
                    is_last_cmd_success = false;
                    std::cerr << "[CuttingMachinePLCComm] Unable to create RobroModbusLib object for Cutting Machine PLC" << std::endl;
                    return false;
                }

                if (config.contains("is_cycle_start_register"))
                {
                    is_cycle_start_register = config["is_cycle_start_register"];
                    if (is_cycle_start_register)
                    {
                        std::cout << "Setting Cycle Start to Register" << std::endl;
                    }
                }

                if (config.contains("is_machine_ready_register"))
                {
                    is_machine_ready_register = config["is_machine_ready_register"];
                    if (is_machine_ready_register)
                    {
                        std::cout << "Setting Machine Ready to Register" << std::endl;
                    }
                }

                if (config.contains("is_spout_status_register"))
                {
                    is_spout_status_register = config["is_spout_status_register"];
                    if (is_spout_status_register)
                    {
                        std::cout << "Setting Spout Status to Register" << std::endl;
                    }
                }

                if (config.contains("is_cycle_start_electrical"))
                {
                    is_cycle_start_electrical = config["is_cycle_start_electrical"];
                    if (is_cycle_start_electrical)
                    {
                        std::cout << "Setting use electrical cycle start" << std::endl;
                    }
                }

                return true;
            }
            else
            {
                std::cout << "[ERROR] Port Address, Baud Rate, Parity, Data Bit or Stop Bit not provided in the config." << std::endl;
                return false;
            }
        }
        return false;
    }
    catch (const std::exception &e)
    {
        std::cout << "[ERROR] Unable to read all the required parameters form config json: " << e.what() << std::endl;
        return false;
    }
}

/// @brief Write cut length to the register of the cutting machine
/// @param body_length
/// @return
bool CuttingMachinePLCComm::writeCutLengthRegister(uint16_t body_length)
{
    bool success = true;
    if (!simulation_mode && cutter_comm != nullptr)
    {
        // Write the length of the fabric to be cut.
        success &= cutter_comm->writeSequenceOfRegisters("cut_length_register", 1, {body_length});
        is_last_cmd_success = success;
        return success;
    }
    else
    {
        return false;
    }
}

/// @brief Write cycle start command
/// @return success
bool CuttingMachinePLCComm::writeCycleStart()
{
    bool success = true;
    if (!simulation_mode && cutter_comm != nullptr)
    {
        if (is_cycle_start_register)
        {
            success &= cutter_comm->writeSequenceOfRegisters("cycle_start", 1, {1});
        }
        else
        {
            success &= cutter_comm->writeSequenceOfBits("cycle_start", 1, {true});
        }

        is_last_cmd_success = success;
    }

    if (cutter_comm != nullptr)
    {
        return success;
    }
    else
    {
        return false;
    }
}

/**
 * @brief set the required length of the fabric to be cut to machine plc. and start the machine cycle.
 *
 * @param body_length - Length of the fabric to be cut.
 */
bool CuttingMachinePLCComm::setNextFabricCutLength(uint16_t body_length)
{
    bool success = true;
    if (!simulation_mode && cutter_comm != nullptr)
    {
        // Write the length of the fabric to be cut.
        success &= cutter_comm->writeSequenceOfRegisters("cut_length_register", 1, {body_length});

        // sleep added between cutlength given and cycle starting.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Start the machine cycle.
        if (!is_cycle_start_electrical)
        {
            if (is_cycle_start_register)
            {
                success &= cutter_comm->writeSequenceOfRegisters("cycle_start", 1, {1});
            }
            else
            {
                success &= cutter_comm->writeSequenceOfBits("cycle_start", 1, {true});
            }
        }

        is_last_cmd_success = success;
    }
    else
    {
        std ::cout << "Running setNextFabricCutLength in Simulation Mode fabric length: " << body_length << std::endl;
    }
    return success;
}

/**
 * @brief set the spout mode to machine plc.
 *
 * @param spout_mode - Mode of the spout.
 */
bool CuttingMachinePLCComm::setSpoutMode(bool spout_mode)
{
    bool success = true;
    if (!simulation_mode && cutter_comm != nullptr)
    {
        if (is_spout_status_register)
        {
            if (spout_mode)
            {
                success &= cutter_comm->writeSequenceOfRegisters("spout_mode", 1, {1});
            }
            else
            {
                success &= cutter_comm->writeSequenceOfRegisters("spout_mode", 1, {0});
            }
        }
        else
        {
            success &= cutter_comm->writeSequenceOfBits("spout_mode", 1, {spout_mode});
        }
        is_last_cmd_success = success;
    }
    else
    {
        std ::cout << "Running setSpoutMode in Simulation Mode spout mode: " << spout_mode << std::endl;
    }
    return success;
}

/**
 * @brief Get the machine ready status.
 *
 * @return true - if the machine is ready to cut the fabric.
 * @return false - otherwise.
 */
bool CuttingMachinePLCComm::getMachineReadyState()
{
    if (!simulation_mode && cutter_comm != nullptr)
    {
        bool raising_edge_status = false;
        // Read the machine status.
        if (is_machine_ready_register)
        {
            std::vector<uint16_t> values = cutter_comm->readSequenceOfRegisters("machine_waiting", 1);
            if (!values.empty())
            {
                raising_edge_status = !machine_ready_last_state && values[0] == 1;
                machine_ready_last_state = (values[0] == 1) ? true : false;
                return raising_edge_status;
            }
            else
            {
                is_last_cmd_success = false;
                return is_last_cmd_success;
            }
        }
        else
        {
            auto status = cutter_comm->readSequenceOfBits("machine_waiting", 1);
            //  If the machine is waiting for the fabric to be cut, it'll return true.
            if (!status.empty())
            {
                // Return true if it is raising edge
                raising_edge_status = !machine_ready_last_state && status[0];
                machine_ready_last_state = status[0];
                return raising_edge_status;
            }
            else
            {
                is_last_cmd_success = false;
                return is_last_cmd_success;
            }
        }
    }
    else
    {
        std::cout << "Running getMachineReadyState in Simulation Mode" << std::endl;
        return true;
    }
}

/**
 * @brief Get the current cycle start signal state.
 *
 * @return true - if the cycle start signal is ON.
 * @return false - if OFF or failed to read.
 */
bool CuttingMachinePLCComm::getCycleStartState()
{
    if (!simulation_mode && cutter_comm != nullptr)
    {
        if (is_cycle_start_register)
        {
            std::vector<uint16_t> values = cutter_comm->readSequenceOfRegisters("cycle_start", 1);
            if (!values.empty())
            {
                return values[0] == 1;
            }
            else
            {
                is_last_cmd_success = false;
                return false;
            }
        }
        else
        {
            auto status = cutter_comm->readSequenceOfBits("cycle_start", 1);
            if (!status.empty())
            {
                return status[0];
            }
            else
            {
                is_last_cmd_success = false;
                return false;
            }
        }
    }
    else
    {
        std::cout << "Running getCycleStartState in Simulation Mode" << std::endl;
        return true;
    }
}

/**
 * @brief Get the current cutter PLC Address and Communication Information.
 *
 * @return json - Current cutter PLC address and communication information.
 */
json CuttingMachinePLCComm::getCutterPLCAddressAndCommInfo()
{
    return cutting_machine_config;
}

/**
 * @brief Check if the last command was successful.
 *
 * @return true - If the last command was successful.
 * @return false - If the last command was not successful.
 */
bool CuttingMachinePLCComm::getIsLastCmdSuccess()
{
    return is_last_cmd_success;
}