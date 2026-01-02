/**
 * @file plcComm.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Definations of Plc communication for Weaving inspection application.
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <plcComm.hpp>

/**
 * @brief Construct a new PLCComm::PLCComm object
 *
 */

PLCComm::PLCComm()
{

    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[PLCComm] Unable to Read all ROS Params of PLC Comm. Please check names and types!";
    }

    initPLC();
}

/**
 * @brief Destroy the PLCComm::PLCComm object
 *
 */
PLCComm::~PLCComm()
{
    delete plcComm;
}

/**
 * @brief It reads the parameters from the ros parameter server and
 * stores them in the class variables.
 *
 * @return true - if parameters are read successfully.
 * @return false - otherwise.
 */
bool PLCComm::readROSParams()
{
    is_rtu_conn_available = (ros::param::get("~port", port_address) &&
                             ros::param::get("~baudrate", baud) &&
                             ros::param::get("~parity", parity) &&
                             ros::param::get("~bytesize", data_bit) &&
                             ros::param::get("~stopbit", stop_bit) &&
                             ros::param::get("~slave_address", slave_address) &&
                             ros::param::get("current_body_processed_auto_reset_mode", current_body_processed_auto_reset_mode));
    is_tcp_conn_available = (ros::param::get("~plc_tcp_port", plc_tcp_port) &&
                             ros::param::get("~plc_tcp_ip", plc_tcp_ip));

    bool any_connection_available = is_rtu_conn_available || is_tcp_conn_available;

    if (!any_connection_available)
    {
        std::cout << "\033[1;31m ERROR : \033[0m[PLCComm] Unable to Read PLC Connection Parameters. Please check names and types!\n";
    }
    if (is_rtu_conn_available && !is_tcp_conn_available)
    {
        std::cout << "\033[1;34m [PLCComm] Using RTU Connection for PLC Communication \033[0m\n";
    }
    if (is_tcp_conn_available)
    {
        std::cout << "\033[1;34m [PLCComm] Using TCP Connection for PLC Communication \033[0m\n";
    }
    if (!ros::param::get("is_cutter_conn_nc", is_cutter_conn_nc))
    {
        is_cutter_conn_nc = true;
    }

    std::cout << "\033[1;34m [PLCComm]Using Cutter Connection as NC " << is_cutter_conn_nc << " \033[0m\n";

    return any_connection_available && (ros::param::get("simulation_mode_on", simulation_mode) &&
                                        ros::param::get("~plc_pulse_per_mm", plc_pulse_per_mm));
}

/**
 * @brief It initializes the PLC.
 */
void PLCComm::initPLC()
{
    // Initialize the PLC addresses.
    plc_addresses = std::map<std::string, int>{
        {"stopping_duration_10ms", 0x1001},
        {"light_register", 0x0810},            // M16
        {"pulses_since_defect_found", 0x9000}, // 0x0723;//Counter: C235 //0x0EFD;
        {"stopping_distance", 0x1005},
        {"start_calculation", 0x0814}, // M20
        {"plc_busy", 0x080B},          // M11
        {"init_plc", 0x080A},

        {"green", 0x080E}, // M14
        {"yellow", 0x0822}, // M34
        {"red", 0x080D},    // M14

        {"buzzer", 0x080F},
        {"override_current_defect", 0x0811},
        {"constant_position_sensor", 0x0812},
        {"cutter_sensor", 0x0412},
        {"reset", 0x0805},
        {"bad_body_count", 0x101E},
        {"good_body_count", 0x101C},
        {"current_pulses", 0x100E},                                 // D14
        {"body_length_counter_target", 0x1020},                     // D32
        {"cam2stopper_counter_target", 0x1036},                     // D54
        {"cut_length_mm", 0x1016},                                  // D22
        {"calculate_lengths", 0x0818},                              // M24
        {"current_body_processed_pulses", 0x1044},                  // D68//0X1032; //body processed in pulses
        {"current_fabric_moved_under_plc_pulses", 0x1049},          // D73 // fabric moved under plc in pulses since last dancer on.
        {"frame_counters_reset", 0x085A},                           // M90
        {"distance_after_last_cutter_sensor_on_interrupt", 0x081D}, // M29 //Get Frame Pulses stored.
        {"distance_after_last_dancer_sensor_on_interrupt", 0x082C}, // M44 //To get fabric moved.
        {"reset_dancer_sensor_fabric_counter", 0x0844},             // M68 - To reset fabric moved under plc
        {"block_punching", 0x0802},                                 // M2 in office simulation setup
                                                                    // At ComSyn this is  this, else it'll be 0x0501

        {"auto_mode", 0x081F},
        {"cycle_start", 0x0820},
        {"motor_forward", 0x0821},
        {"spout_cut_on_cycle_start_enable", 0x0822}, // M34 if we enable spout cutting will happen at while enabling cut
        {"cut_befor_start", 0x0821},

        {"current_body_processed_auto_reset_mode", 0x0824}, // M36 if this ON the plc will not reset the current body processed
        {"plc_program_version", 0x1003},                    // D3 Program version
        {"reset_current_body_processed", 0x0823}            // M35 Rest current body processed
        // TODO: make this from the launch file if wiring is not changing in ComSyn. In that PLC - Y1 is damaged.
    };

    // Initialize the PLC communication.
    if (!simulation_mode)
    {
        if (is_tcp_conn_available)
        {
            plcComm = new RobroModbusLib(plc_tcp_ip, plc_tcp_port);
        }
        else
        {
            plcComm = new RobroModbusLib(port_address, baud, parity[0], data_bit, stop_bit);
        }

        // Set the slave address of the PLC
        plcComm->setSlaveAddress(slave_address);

        plcComm->setAddresses(plc_addresses);
        plcComm->writeSequenceOfBits("init_plc", 1, {true});

        // Setting auto reset flag inside plc to false
        uint16_t plc_program_version = getPLCProgramVersion();
        ROS_INFO("PLC Program Version: %u", plc_program_version);
        if (current_body_processed_auto_reset_mode)
        {

            if (plc_program_version != 0)
            {
                if (setCurrentBodyProcessedResetModeOFF())
                {
                    reset_current_body_processed = true;
                    ROS_INFO("Disabled a Current_Body_Processed_Auto_Reset_Mode in PLC");
                }
                else
                {
                    ROS_ERROR("Couldn't Disable Current_Body_Processed_Auto_Reset_Mode in PLC");
                }
            }
            else
            {
                ROS_ERROR("Requested to Disable Current_Body_Processed_Auto_Reset_Mode But Plc Program version Don't support the functionality");
            }
        }
    }
    else
    {
        is_last_cmd_success = false;
        ROS_INFO("Running PLC Communication in Simulation Mode");
    }

    resetPLC();
}

/**
 * @brief read the plc program version from the plc
 * @return value - version of the plc program
 */
uint16_t PLCComm::getPLCProgramVersion()
{
    if (!simulation_mode)
    {
        std::vector<uint16_t> values = plcComm->readSequenceOfRegisters("plc_program_version", 1);
        if (values.empty())
        {
            ROS_ERROR("Couldn't read processed body length from PLC.");
            is_last_cmd_success = false;
            return 0;
        }
        return values[0];
    }
    return 0;
}

/**
 * @brief disabling current body processed auto reset mode
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setCurrentBodyProcessedResetModeOFF()
{
    if (!simulation_mode)
    {
        // We are setting it ON because it is normally close in the plc
        is_last_cmd_success = plcComm->writeSequenceOfBits("current_body_processed_auto_reset_mode", 1, {true});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief get current_body_prcessed_reset_mode
 * @return true - if the mode is ON
 * @return false - if the mode is OFF
 */
bool PLCComm::getCurrentBodyProcessedResetMode()
{
    return reset_current_body_processed;
}

/**
 * @brief Resetting current body processed values
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::resetCurrentBodyProcessed()
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("reset_current_body_processed", 1, {true});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Reset the PLC values by turning flag on that'll reset all values in PLC
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::resetPLCValues()
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("reset", 1, {true});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Green Light to ON/OFF.
 *
 * @param state - true to turn on the green light.
 *              - false to turn off the green light.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setGreenLightTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("green", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Yellow Light to ON/OFF.
 *
 * @param state - true to turn on the yellow light.
 *              - false to turn off the yellow light.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setYellowLightTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("yellow", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Red Light to ON/OFF.
 *
 * @param state - true to turn on the red light.
 *              - false to turn off the red light.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setRedLightTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("red", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Inspection Light to ON/OFF.
 *
 * @param state - true to turn on the inspection light.
 *              - false to turn off the inspection light.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setInspectionLightTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("light_register", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Induction Motor To state
 *
 * @param state - true to turn on the induction motor.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setInductionMotorTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("motor_forward", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Auto Mode To state
 *
 * @param state - true to turn on the auto mode.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setAutoModeTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("auto_mode", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Cycle Start To state
 *
 * @param state - true to turn on the cycle start.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setCycleStartTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("cycle_start", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}


/**
 * @brief Get if Machine is available or not
 *
 * @return true - if available
 * @return false - if not available
 */
bool PLCComm::getMachineAvailableState()
{
    if (!simulation_mode)
    {
        bool raising_edge_status = false;
        std::vector<uint16_t> values = plcComm->readSequenceOfRegisters("auto_mode", 1);
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
    return false;
}

/**
 * @brief Set the Buzzer To to ON/OFF.
 *
 * @param state - true to turn on the buzzer.
 *              - false to turn off the buzzer.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setBuzzerTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("buzzer", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Set the Punch Blocking to ON/OFF for number of beeps.
 *
 * @param number_of_beeps - number of beeps to be made by the buzzer.
 *
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::beepTheBuzzer(int number_of_beeps)
{
    if (!simulation_mode)
    {
        for (int i = 0; i < number_of_beeps; i++)
        {
            is_last_cmd_success = plcComm->writeSequenceOfBits("buzzer", 1, {true});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            is_last_cmd_success = plcComm->writeSequenceOfBits("buzzer", 1, {false});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Write the cut length to the PLC and also write if the punching is on or not.
 *
 * @param cut_length - cut length in mm.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::writeCutLength(uint16_t cut_length_mm)
{
    if (!simulation_mode)
    {
        bool req = true;
        // Write the cut length to the PLC.
        req &= plcComm->writeSequenceOfRegisters("cut_length_mm", 1, {cut_length_mm});

        req &= plcComm->writeSequenceOfRegisters("stopping_duration_10ms", 1, {100});

        plc_counter_increment_target_mm = cut_length_mm + 1000;

        // Write the target counter value to the PLC.
        req &= plcComm->writeSequenceOfRegisters("body_length_counter_target", 1, {plc_counter_increment_target_mm});
        req &= plcComm->writeSequenceOfRegisters("cam2stopper_counter_target", 1, {plc_counter_increment_target_mm});
        is_last_cmd_success = req;
        return req;
    }
    return true;
}

/**
 * @brief It will turn on the PLC flag to calculate the length of the fabric.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::calculateLengths()
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("calculate_lengths", 1, {true});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief Reset the frame counters and the fabric moved under camera counters.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::resetFrameCounters()
{
    if (!simulation_mode)
    {
        bool req = true;
        req &= plcComm->writeSequenceOfBits("frame_counters_reset", 1, {true});
        req &= plcComm->writeSequenceOfBits("reset_dancer_sensor_fabric_counter", 1, {true});
        is_last_cmd_success = req;
        return req;
    }
    return true;
}

/**
 * @brief Request the PLC to stop the machine by setting next stopping distance.
 *
 * @param next_stopping_distance_mm - next stopping distance in mm.
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::requestToStopMM(uint16_t next_stopping_distance_mm)
{
    bool req = true;
    if (!simulation_mode)
    {
        // Write the next stopping distance to the PLC.
        req &= plcComm->writeSequenceOfRegisters("stopping_distance", 1, {next_stopping_distance_mm});
        req &= plcComm->writeSequenceOfBits("start_calculation", 1, {true});
        is_last_cmd_success = req;
    }
    else
    {
        stopper_counter = next_stopping_distance_mm;
        std::thread([this]()
                    {
                    while(stopper_counter){
                        // Assuming 1 mm requires 10 millisecond to move
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        stopper_counter--;
                    } })
            .detach();
    }
    return req;
}

/**
 * @brief Check if stopping operation is done or not.
 *
 * @return true - if stopping operation is done by PLC.
 * @return false - if PLC is still busy.
 */
bool PLCComm::isStoppingDone()
{
    std::vector<bool> values;
    if (!simulation_mode)
    {
        values = plcComm->readSequenceOfBits("plc_busy", 1);

        if (values.empty())
        {
            ROS_ERROR("Error in Checking Stopping done by PLC.");
            is_last_cmd_success = false;
            return is_last_cmd_success;
        }

        // if plc is busy, stopping is not done
        return !values[0];
    }
    else
    {
        // if we're in simulation, let's just say that the plc is not busy i.e. defect was cut.
        return !stopper_counter;
    }
}

/**
 * @brief Check if constant distance sensor is on or not.
 *
 * @return true - if constant distance sensor is on.
 * @return false - if constant distance sensor is off.
 */
bool PLCComm::isConstantDistanceSensorOn()
{
    std::vector<bool> values;

    if (!simulation_mode)
    {
        values = plcComm->readSequenceOfBits("constant_position_sensor", 1);

        if (values.empty())
        {
            ROS_ERROR("Couldn't read constant distance sensor bit from PLC");
            is_last_cmd_success = false;
            return is_last_cmd_success;
        }
        return values[0];
    }
    else
    {
        // if we're in simulation, let's just say it's on
        return true;
    }
}

/**
 * @brief Check if cutter is on or not.
 *
 * @return true - if cutter is on.
 * @return false - if cutter is off.
 */
bool PLCComm::isCutterOn()
{
    std::vector<bool> values;

    if (!simulation_mode)
    {
        values = plcComm->readSequenceOfInputBits("cutter_sensor", 1);

        if (values.empty())
        {
            ROS_ERROR("Couldn't read cutter sensor bit from PLC");
            is_last_cmd_success = false;
            return is_last_cmd_success;
        }
        return is_cutter_conn_nc ? !values[0] : values[0];
    }
    else
    {
        // if we're in simulation, let's just say it's on
        return true;
    }
}

/**
 * @brief Request the PLC to make punch blocking enabled or disabled
 *
 * @param state - true to enable punch blocking, false to disable
 * @return true - if the request was successful
 * @return false - if the request failed
 */
bool PLCComm::setPunchBlockedTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("block_punching", 1, {state});
        return is_last_cmd_success;
    }

    return true;
}

/**
 * @brief Get the mm Since Defect from PLC register.
 * This is the mm since the last defect was found.
 *
 * @return uint64_t - mm since defect from PLC.
 */
uint64_t PLCComm::getMMSinceDefect()
{
    std::vector<uint16_t> values;

    if (!simulation_mode)
    {
        values = plcComm->readSequenceOfRegisters("pulses_since_defect_found", 1);

        if (values.empty())
        {
            ROS_ERROR("Couldn't read pulses since defect from PLC.");
            is_last_cmd_success = false;
            return 0;
        }

        // handle backlash, since when encoder turns negative, values overflow. So keep some buffer and handle.
        uint64_t pulse = (values[0] < 0 || values[0] > (UINT16_MAX - 3000)) ? 0 : (uint64_t)values[0];

        // convert plc pulses to mm
        return (uint64_t)(pulse / plc_pulse_per_mm);
    }
    else
    {
        // in simulation mode, just send 100. There we will recv immediate ack.
        return 100;
    }
}

/**
 * @brief Get the Current Body Processed in mm from PLC register.
 *
 * @return uint64_t - current body processed in mm from PLC.
 */
uint64_t PLCComm::getCurrentBodyProcessedMM()
{
    if (!simulation_mode)
    {
        // This will give an interrupt to the PLC Counter that is calculating fabric moved under cutter after the last cutter sensor on.
        if (plcComm->writeSequenceOfBits("distance_after_last_cutter_sensor_on_interrupt", 1, {true}))
        {
            std::vector<uint16_t> values;

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            values = plcComm->readSequenceOfRegisters("current_body_processed_pulses", 1);

            if (values.empty())
            {
                ROS_ERROR("Couldn't read processed body length from PLC.");
                is_last_cmd_success = false;
                return 0;
            }
            // handle backlash, since when encoder turns negative, values overflow. So keep some buffer and handle.
            uint64_t pulse = values[0] > (UINT16_MAX - 3000) ? 0 : (uint64_t)values[0];

            // This is because we added a division of 10 in PLC
            pulse *= 10;

            // convert plc pulses to mm
            return (uint64_t)(pulse / plc_pulse_per_mm);
        }
        else
        {
            return 0;
        }
    }

    // in simulation mode, just send 1000mm.
    return (100 * 7.2);
}

/**
 * @brief Get the Fabric Moved Under Cutter in mm from PLC register.
 * After the fabric is cut, the fabric moved under cutter in mm are calculated.
 *
 * @return int - fabric moved under cutter in mm from PLC.
 */
bool PLCComm::getFabricMovedUnderCutterMM(uint64_t &value)
{
    if (!simulation_mode)
    {
        // This will give an interrupt to the PLC Counter tha tis calculating fabric moved under cutter after the last cutter sensor on.
        if (plcComm->writeSequenceOfBits("distance_after_last_dancer_sensor_on_interrupt", 1, {true}))
        {
            std::vector<uint16_t> values;

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            values = plcComm->readSequenceOfRegisters("current_fabric_moved_under_plc_pulses", 1);

            if (values.empty())
            {
                ROS_ERROR("Couldn't read fabric moved under cutter from PLC.");
                is_last_cmd_success = false;
                return false;
            }

            // handle backlash, since when encoder turns negative, values overflow. So keep some buffer and handle.
            uint64_t pulse = values[0] > (UINT16_MAX - 3000) ? 0 : (uint64_t)values[0];

            // this is because we added a division of 10 on plc
            pulse *= 10;

            // convert plc pulses to mm
            value = (uint64_t)(pulse / plc_pulse_per_mm);
            return true;
        }
        else
        {
            return false;
        }
    }
    // in simulation mode, just send a random value
    value = (100 * 7.2);
    return true;
}

/**
 * @brief To reset the PLC values by reseting the plc state, punch blocking,
 * and reseting the plc counters.
 */
void PLCComm::resetPLC()
{
    if (!simulation_mode)
    {
        is_last_cmd_success = true;
        setGreenLightTo(false);
        resetPLCValues();
        setPunchBlockedTo(false);
        calculateLengths();
    }
}

/**
 * @brief Get the Last Cmd Success state
 *
 * @return true - if the last command was successful
 * @return false - if the last command was failed
 */
bool PLCComm::getLastCmdSuccess()
{
    return is_last_cmd_success;
}

/**
 * @brief Set spoutcut enable or disable during setcutbeforestart
 *
 * @param state - true to enable spout during cutbeforestart
 * @return true - if request successful.
 * @return false - if request failure.
 */
bool PLCComm::setCCutModeTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("spout_cut_on_cycle_start_enable", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}

/**
 * @brief turn spout and cutter before start
 *
 * @param state - true to enable cutter spout is depend on ccutmode
 * @return true - if request successfull.
 * @return false - if request failure.
 */
bool PLCComm::setCutBeforeStartTo(bool state)
{
    if (!simulation_mode)
    {
        is_last_cmd_success = plcComm->writeSequenceOfBits("cut_befor_start", 1, {state});
        return is_last_cmd_success;
    }
    return true;
}
