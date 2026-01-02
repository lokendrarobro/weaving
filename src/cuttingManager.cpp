#include <cuttingManager.hpp>

/**
 * @brief Construct a new Cutting Manager object
 */
CuttingManager::CuttingManager()
{
    std::string port_address;
    int baud;
    std::string parity;
    int data_bit;
    int stop_bit;
    bool simulation_mode;
    std::map<std::string, std::string> plc_addresses;
    if (!(ros::param::get("~auto_cut_defect_offset_mm", defect_offset_mm) &&
          ros::param::get("~cutting_mc_port", port_address) &&
          ros::param::get("~cutting_mc_baud", baud) &&
          ros::param::get("~cutting_mc_parity", parity) &&
          ros::param::get("~cutting_mc_bytesize", data_bit) &&
          ros::param::get("~cutting_mc_stop_bit", stop_bit) &&
          ros::param::get("~cutting_mc_slave_address", slave_address) &&
          ros::param::get("~is_cycle_start_register", is_cycle_start_register) &&
          ros::param::get("~is_machine_ready_register", is_machine_ready_register) &&
          ros::param::get("~is_cycle_start_electrical", is_cycle_start_electrical) &&
          ros::param::get("~is_spout_status_register", is_spout_status_register) &&
          ros::param::get("~cutting_mc_addresses", plc_addresses) &&
          ros::param::get("half_cut_mode", half_cut_mode) &&
          ros::param::get("simulation_mode_on", simulation_mode)))
    {
        ROS_ERROR("[CuttingManager] not found in the parameter server");
    }
    else
    {
        int tcp_port;
        std::string tcp_ip;
        bool is_tcp_conn_available = (ros::param::get("~cutting_mc_plc_tcp_port", tcp_port) &&
                                      ros::param::get("~cutting_mc_plc_tcp_ip", tcp_ip));
        system_plc_config["port_address"] = port_address;
        system_plc_config["baud"] = baud;
        system_plc_config["parity"] = parity;
        system_plc_config["data_bit"] = data_bit;
        system_plc_config["stop_bit"] = stop_bit;
        system_plc_config["slave_address"] = slave_address;
        system_plc_config["simulation_mode"] = simulation_mode;
        system_plc_config["addresses"] = plc_addresses;
        system_plc_config["is_cycle_start_register"] = is_cycle_start_register;
        system_plc_config["is_cycle_start_electrical"] = is_cycle_start_electrical;
        system_plc_config["is_machine_ready_register"] = is_machine_ready_register;
        system_plc_config["is_spout_status_register"] = is_spout_status_register;

        if (is_tcp_conn_available)
        {
            system_plc_config["tcp_port"] = tcp_port;
            system_plc_config["tcp_ip"] = tcp_ip;
        }

        if (setSystemPLCConnection(system_plc_config))
        {
            is_cutting_manager_initialized = true;
            std::cout << "[CuttingManager] Cutting Manager Initialized\n";
        }
        else
        {
            is_cutting_manager_initialized = false;
            std::cout << "[CuttingManager] Cutting Manager Initialization Failed\n";
        }
    }
}

/**
 * @brief Set the Cut Lengths to be used for the cutting process.
 *
 * @param cut_lengths - Array of cut lengths available for the fabric cutting.
 * @return true - If the cut lengths are set successfully.
 * @return false - If the cut lengths are not set successfully.
 */
bool CuttingManager::setCutLengths(const std::vector<int> &cut_lengths)
{
    // convert to map
    this->cut_lengths = cut_lengths;
    body_counters.clear();
    body_count.clear();
    body_counters.resize(cut_lengths.size() + 1, 0);
    body_count.resize(cut_lengths.size(), 0);
    return true;
}

/**
 * @brief Return if the cutting manager is initialized properly
 *
 */
bool CuttingManager::isCuttingManagerInitialized()
{
    return is_cutting_manager_initialized;
}

/**
 * @brief Set the System PLC Connection object.
 *
 * @param system_plc_config - System PLC configuration to be used for the cutting process.
 * @return true - If the system PLC connection is set successfully.
 * @return false - If the system PLC connection is not set successfully.
 */
bool CuttingManager::setSystemPLCConnection(json system_plc_config)
{
    // this will set the plc communication and addresses. But it doesn't return if the connection is proper or not
    //  this is because our library doesn't have a function to check if the connection is proper or not.
    cutterPLC.setPLCCommAndAddresses(system_plc_config);

    // this will fail if my object isn't proper.
    cutterPLC.getMachineReadyState();

    return cutterPLC.getIsLastCmdSuccess();
}

/**
 * @brief Send the cutting length to the system PLC.
 *
 * @param cut_length - The cut length which needs to be sent to the system PLC.
 * @return true - If the cutting length is sent successfully.
 * @return false - If the cutting length is not sent successfully.
 */
bool CuttingManager::sendCutLengthToCuttingMachine(int cut_length)
{
    return cutterPLC.setNextFabricCutLength(cut_length);
}

/**
 * @brief Send the spout on flag to the cutting machine.
 *
 * @param spout_on - The flag to check if the spout on flag needs to be issued.
 * @return true - If the spout on flag is sent successfully.
 * @return false - If the spout on flag is not sent successfully.
 */
bool CuttingManager::sendSpoutToCuttingMachine(bool spout_on)
{
    return cutterPLC.setSpoutMode(spout_on);
}

/**
 * @brief Check if the machine is available for the cutting plan to write.
 *
 * @return true - If the machine is available for the cutting plan to write.
 * @return false - If the machine is not available for the cutting plan to write.
 */
bool CuttingManager::isMachineAvailable()
{
    return cutterPLC.getMachineReadyState();
}

/**
 * @brief Plan the next cut length and spout on flag to be used for the cutting process.
 *
 * @param defects - The defects to be used for the cutting process.
 * @param need_to_issue_spout - The flag to check if the spout on flag needs to be issued.
 * @param send_to_plc - The flag to check if the cutting plan needs to be sent to the PLC.
 * @return true - If the next cut length and spout on flag are planned successfully.
 * @return false - If the next cut length and spout on flag are not planned successfully.
 */

//  add struct for stroring all the old info of previous body
bool CuttingManager::planNextCutLength(std::vector<WorldMapDefect> &defects, bool need_to_issue_spout, bool send_to_plc, bool top_bottom_selected)
{
    // lastPlannedBody.length = next_available_cut_length_mm;
    // lastPlannedBody.type = getCutLengthInitials();

    cycle_event_tracker.recordPlanningEvent();

    next_available_cut_length_mm = 0;
    next_spout_on = need_to_issue_spout;
    region_start_defect = WorldMapDefect();
    region_end_defect = WorldMapDefect();
    is_next_body_contains_defect = false;
    // Sort the vector based on detection.bbox.center.y using std::sort
    std::sort(defects.begin(), defects.end(),
              [](const WorldMapDefect &a, const WorldMapDefect &b)
              {
                  return a.detection.bbox.center.y < b.detection.bbox.center.y;
              });

    // remove all defects with negative y-coordinates
    for (auto it = defects.begin(); it != defects.end();)
    {
        if (it->detection.bbox.center.y <= 0.0f)
        {
            it = defects.erase(it);
        }
        else
        {
            break;
        }
    }

    if (defects.size() == 0)
    {

        next_available_cut_length_mm = cut_lengths[0];
    }
    else
    {
        // If top bottom mode is not selected
        int start_index = -1;
        // get the first defect who's is_defect_class_disabled is false
        for (int i = 0; i < defects.size(); i++)
        {
            if (!defects[i].is_defect_class_disabled)
            {
                region_start_defect = defects[i];
                region_end_defect = defects[i];
                start_index = i;
                break;
            }
        }

        if (start_index == -1)
        {
            next_available_cut_length_mm = cut_lengths[0];
        }
        else
        {
            // Check if form first defect any cut length is available
            int start_defect_start_y = region_start_defect.detection.bbox.center.y - region_start_defect.detection.bbox.size_y - defect_offset_mm;
            int available_cut_length = checkIfCutLengthsAvailable(0, start_defect_start_y);

            if (available_cut_length)
            {
                next_available_cut_length_mm = available_cut_length;
            }
            else
            {
                force_to_cut_fixed_length = false;
                is_next_body_contains_defect = true;
                for (int i = start_index + 1; i < defects.size(); i++)
                {
                    if (defects[i].is_defect_class_disabled)
                    {
                        continue;
                    }

                    int end_defect_end_y = region_end_defect.detection.bbox.center.y + region_end_defect.detection.bbox.size_y;
                    WorldMapDefect next_defect = defects[i];

                    if (next_defect.detection.bbox.center.y < end_defect_end_y)
                    {
                        int next_defect_end_y = next_defect.detection.bbox.center.y + next_defect.detection.bbox.size_y;
                        if (next_defect_end_y > end_defect_end_y)
                        {
                            region_end_defect = next_defect;
                        }
                        continue;
                    }
                    int next_defect_start_y = next_defect.detection.bbox.center.y - next_defect.detection.bbox.size_y - defect_offset_mm;
                    available_cut_length = checkIfCutLengthsAvailable(end_defect_end_y, next_defect_start_y);
                    if (available_cut_length)
                    {
                        break;
                    }
                    else
                    {
                        region_end_defect = next_defect;
                        int end_defect_end_y = region_end_defect.detection.bbox.center.y + region_end_defect.detection.bbox.size_y;
                        available_cut_length = checkIfCutLengthsAvailable(0, end_defect_end_y);
                        // if the defective body length is more than 80% of the cut length then force to cut the cut length by 2
                        if (available_cut_length >= cut_lengths[0] * 0.8 && half_cut_mode)
                        {
                            force_to_cut_fixed_length = true;
                            break;
                        }
                    }
                }
                if (force_to_cut_fixed_length)
                {
                    // if it is a defective body then forcing it to cut cutlength by 2
                    next_available_cut_length_mm = cut_lengths[0] / 2;
                }
                else
                {
                    if (region_start_defect.defect_id == region_end_defect.defect_id)
                    {
                        next_available_cut_length_mm = region_start_defect.detection.bbox.center.y + region_start_defect.detection.bbox.size_y;
                    }
                    else
                    {
                        next_available_cut_length_mm = region_end_defect.detection.bbox.center.y + region_end_defect.detection.bbox.size_y;
                    }
                    next_available_cut_length_mm += defect_offset_mm;

                    // if the defective body length is more than 80% of the cut length then force to cut the cut length by 2
                    if (next_available_cut_length_mm >= cut_lengths[0] * 0.8 && half_cut_mode)
                    {
                        // if it is a defective body then forcing it to cut cutlength by 2
                        next_available_cut_length_mm = cut_lengths[0] / 2;
                        force_to_cut_fixed_length = true;
                    }
                }
            }
        }
    }
    next_spout_on &= ((cut_lengths[0] == next_available_cut_length_mm) & !is_next_body_contains_defect);

    // we are forcefully changing to full cutlength due to top bottom mode on.
    if (top_bottom_selected)
    {
        next_available_cut_length_mm = cut_lengths[0];
        force_to_cut_fixed_length = false;
    }
    lastPlannedBody.length = next_available_cut_length_mm;
    lastPlannedBody.type = getCutLengthInitials();
    lastPlannedBody.isValid = true;

    // int index = getCutLengthIndex(next_available_cut_length_mm);

    // // if the cut length is available and the next body does not contain defect
    // // then increment the counter for the cut length
    // // else increment the counter for the default cut length
    // if (index != -1 && !is_next_body_contains_defect)
    // {
    //     body_counters[index]++;
    // }
    // else
    // {
    //     body_counters[body_counters.size() - 1]++;
    // }

    if (send_to_plc)
    {

        bool ok = sendCutLengthToCuttingMachine(next_available_cut_length_mm);
        cycle_event_tracker.recordCycleStartEvent();
        return ok;
    }
    return true;
}

void CuttingManager::acknowledgeCycleStart(bool is_cycle_start_electrical)
{
    if (cycle_event_tracker.getCurrentState() == CycleEventTracker::State::Start)
    {
        if (is_cycle_start_electrical)
        {
            cycle_event_tracker.recordAcknowledgeEvent();
        }
        else if (!cutterPLC.getCycleStartState())
        {
            cycle_event_tracker.recordAcknowledgeEvent();
        }
    }
}

/**
 * @brief Check if the cut lengths are available for the cutting process.
 *
 * @param current_defect_y - The current defect y coordinate to be used for the cutting process.
 * @param next_defect_y - The next defect y coordinate to be used for the cutting process.
 * @return int - The available cut length for the cutting process.
 */
int CuttingManager::checkIfCutLengthsAvailable(int current_defect_y, int next_defect_y)
{
    int cut_length = 0;
    int distance = next_defect_y - current_defect_y;
    for (int i = 0; i < cut_lengths.size(); i++)
    {
        if (cut_lengths[i] <= distance)
        {
            cut_length = cut_lengths[i];
            break;
        }
    }
    // Return the cut length 0 if no cut length is available with in the defect start
    return cut_length;
}

/**
 * @brief Get the Cut Length Index.
 *
 * @param cut_length - The cut length to be used for the cutting process.
 * @return int - The index of the cut length.
 */
int CuttingManager::getCutLengthIndex(int cut_length)
{
    int index = -1;
    for (int i = 0; i < cut_lengths.size(); i++)
    {
        if (cut_lengths[i] == cut_length)
        {
            index = i;
            break;
        }
    }
    return index;
}

int CuttingManager::getCutLengthFromIndex(int index)
{
    if (index < cut_lengths.size())
    {
        return cut_lengths[index];
    }
    return 0;
}

/**
 * @brief Get the PLC Connection Details.
 *
 * @return json - The PLC connection details to be used for the cutting process.
 */
json CuttingManager::getPLCConnectionDetails()
{
    json connection_details;
    connection_details["cutting_machine"] = cutterPLC.getCutterPLCAddressAndCommInfo();
    connection_details["is_last_cmd_success"] = cutterPLC.getIsLastCmdSuccess();
    return connection_details;
}

/**
 * @brief Get the Next Available Cut Length value
 *
 * @return int - The next available cut length value
 */
int CuttingManager::nextCutLength()
{
    return next_available_cut_length_mm;
}

/**
 * @brief Retrieves the last planned body.
 *
 * @return LastPlan The last planned body object.
 */
LastPlan CuttingManager::getLastPlannedBody()
{
    return lastPlannedBody;
}

/**
 * @brief Increments the body count by the specified length.
 *
 * @param length The number of bodies to increment the count by.
 */
void CuttingManager::incrementBodyCount(std::string type)
{
    if (type == "P"){
        body_counters[0]++;
        body_count[0]++;
    }
    else if (type == "S")
    {
        body_counters[1]++;
        body_count[1]++;
    }
    else if (type == "T")
    {
        body_counters[2]++;
        body_count[2]++;
    }
    else
    {
        body_counters[body_counters.size() - 1]++;
        body_count[body_count.size() - 1]++;
    }
}

void CuttingManager::resetBodyCounters(int primary_count, int secondary_count, int tertiary_count,int defective_count)
{
    if(body_counters.size() == 4)
    {
        body_counters[0] = primary_count;
        body_counters[1] = secondary_count;
        body_counters[2] = tertiary_count;
        body_counters[3] = defective_count;
    }
    else
    {
        body_counters[0] = primary_count;
        body_counters[1] = defective_count;
    }
}

/**
 * @brief Get the Next Spout On value
 *
 * @return true - If the next spout on flag is true
 * @return false - If the next spout on flag is false
 */
bool CuttingManager::spoutOn()
{
    return next_spout_on;
}

/**
 * @brief Get the Is Next Body Contains Defect value
 *
 * @return true - If the next body contains defect flag is true
 * @return false - If the next body contains defect flag is false
 */
bool CuttingManager::defectPresent()
{
    return is_next_body_contains_defect;
}

/**
 * @brief Is the cutting length forced to cut fixed length due to contine defect
 * or the defect is at the end of the body.
 */
bool CuttingManager::isForcedToCutFixedLength()
{
    return force_to_cut_fixed_length;
}

/**
 * @brief Is the cutting length primary.
 *
 * @return true - If the cutting length is primary.
 * @return false - If the cutting length is not primary.
 */
bool CuttingManager::isPrimary()
{
    int index = getCutLengthIndex(next_available_cut_length_mm);

    return (index == 0);
}
/**
 * @brief Is the cutting length primary with 10%.
 *
 * @return true - If the cutting length is primary.
 * @return false - If the cutting length is not primary.
 */
bool CuttingManager::isPrimaryLength(int length)
{
    if (length >= 0.9 * cut_lengths[0] && length <= 1.1 * cut_lengths[0])
    {
        return true;
    }
    return false;
}

/**
 * @brief Write Cut len
 *
 * @return
 */
bool CuttingManager::writeCutLength(uint16_t cut_len)
{
    return cutterPLC.writeCutLengthRegister(cut_len);
}

/**
 * @brief Write Cycle Start
 *
 * @return
 */
bool CuttingManager::writeCycleStart()
{
    return cutterPLC.writeCycleStart();
}

/**
 * @brief Get the Color For Cutting Plan object
 *
 * @return cv::Scalar - color for the cutting plan.
 */
cv::Scalar CuttingManager::getColorForCuttingPlan()
{
    int index = getCutLengthIndex(next_available_cut_length_mm);
    if (is_next_body_contains_defect)
    {
        if (force_to_cut_fixed_length)
        {
            return cv::Scalar(100, 0, 207);
        }
        return cv::Scalar(0, 207, 255);
    }
    switch (index)
    {
    case 0:
        return cv::Scalar(255, 255, 255);
    case 1:
        return cv::Scalar(0, 255, 0);
    case 2:
        return cv::Scalar(112, 128, 128);
    }
    return cv::Scalar(0, 207, 255);
}

/**
 * @brief Get the Color For Defect Popup
 *
 * @return cv::Scalar - color for the cutting plan.
 */
cv::Scalar CuttingManager::getColorForDefectPopup()
{
    int index = getCutLengthIndex(next_available_cut_length_mm);
    if (is_next_body_contains_defect)
    {
        return cv::Scalar(48, 7, 112);
    }
    switch (index)
    {
    case 0:
        return cv::Scalar(112, 48, 7);
    case 1:
        return cv::Scalar(7, 71, 112);
    case 2:
        return cv::Scalar(7, 112, 48);
    }
    return cv::Scalar(112, 48, 7);
}

json CuttingManager::getSystemPLCConfig()
{
    return system_plc_config;
}

/**
 * @brief Get the Next Available Cut Length Initials.
 *
 * @return std::string - The next available cut length initials
 */
std::string CuttingManager::getCutLengthInitials()
{
    if (is_next_body_contains_defect)
    {
        return "D";
    }
    switch (getCutLengthIndex(next_available_cut_length_mm))
    {
    case 0:
        return "P";
        break;
    case 1:
        return "S";
        break;
    case 2:
        return "T";
    }
    return "D";
}

/**
 * @brief  Get the cutting counter and next cut length
 *
 * @return json - The cutting counter and next cut length.
 */
json CuttingManager::getCuttingCounterAndNextCutLength()
{
    json counters;
    counters["next_available_cut_length"] = next_available_cut_length_mm / 10.0;
    counters["body_counters"] = body_counters;
    counters["is_next_body_contains_defect"] = is_next_body_contains_defect;
    cv::Scalar color = getColorForCuttingPlan();
    int blue = static_cast<int>(color[0]);
    int green = static_cast<int>(color[1]);
    int red = static_cast<int>(color[2]);

    // Convert each channel to hexadecimal strings
    std::stringstream stream;
    stream << "#" << std::setfill('0') << std::setw(2) << std::hex << blue
           << std::setw(2) << std::hex << green << std::setw(2) << std::hex << red << std::setw(2);

    counters["color"] = stream.str();

    return counters;
}