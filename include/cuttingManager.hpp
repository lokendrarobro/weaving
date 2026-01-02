/**
 * @file cuttingManager.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Cutting Manager class to manage the cutting process of the fabric.
 * @version 1.0
 * @date 2024-02-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef CUTTING_MANAGER_HPP
#define CUTTING_MANAGER_HPP

#include <vector>
#include <sstream>
#include <json.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <opencv2/core.hpp>
#include <worldMapDefect.hpp>
#include <cuttingMachinePLCComm.hpp>

#include <cycleEventTracker.hpp>

using json = nlohmann::json;

struct LastPlan
{
    bool isValid = false;
    int length = 0;         // Length of the last planned cut
    std::string type = "D"; // Type of body (you may define what values this can take)

    void reset()
    {
        isValid = false;
        length = 0;
        type = "D"; // Default to defect
    }
};

/**
 * @brief Cutting Manager class to manage the cutting process of the fabric.
 */
class CuttingManager
{
    /**
     * @brief Array of cut lengths available for the fabric cutting.
     */
    std::vector<int> cut_lengths;

    /**
     * @brief Array of counter which will be used to count the number of defects in each cut length.
     */
    std::vector<int> body_counters;

    std::vector<int> body_count;

    /**
     * @brief Auto cut defect offset in mm. This is the offset in mm for the defect to be cut or
     * re-cut in auto cut mode as well as add margin to the defect in front and back to defect position.
     */
    int defect_offset_mm{0};

    /**
     * @brief Region start defect to be used for the cutting process.
     */
    WorldMapDefect region_start_defect;

    /**
     * @brief Region end defect to be used for the cutting process.
     */
    WorldMapDefect region_end_defect;

    /**
     * @brief Next available cut length to be used for the cutting process.
     */
    int next_available_cut_length_mm{0};

    /**
     * @brief Next spout on flag to be used for the cutting process.
     */
    bool next_spout_on{false};

    /**
     * @brief Is the next body contains defect flag to be used for the cutting process.
     */
    bool is_next_body_contains_defect{false};

    /**
     * @brief Cutter PLC communication object to communicate with the cutter PLC.
     */
    CuttingMachinePLCComm cutterPLC;

    /**
     * @brief Is the cycle start a bit or register in the other PLC?
     *
     */
    /*
     *object of LatPlannedBody
     */
    LastPlan lastPlannedBody;

    bool is_cycle_start_register{false};

    /**
     * @brief is the machine ready a bit or register in the other PLC?
     */

    bool is_machine_ready_register{false};

    bool is_cycle_start_electrical{false}; // use the electrical instead of register for cycle start

    /**
     * @brief is spout status in the machine is registor or not
     */

    bool is_spout_status_register{false};

    /**
     * @brief slave address of the cutting machine plc
     */
    int slave_address{1};

    /**
     * @brief is cutting manager initialized
     */
    bool is_cutting_manager_initialized{false};

    /**
     * @brief true if you want to cut half length if defected body is 80% closer to cutlength
     *         flase otherwise
     */

    bool half_cut_mode{true};

    /**
     * @brief it will be set to true if the defective body is 80% closer to cutlength
     * we have to force the system to cut the fixed length.
     */
    bool force_to_cut_fixed_length{false};

    /**
     * @brief the configuration of the plc
     */
    json system_plc_config;

public:
    /**
     * @brief the cycle event tracker
     */
    CycleEventTracker cycle_event_tracker;

    /**
     * @brief Construct a new Cutting Manager object
     */
    CuttingManager();

    /**
     * @brief Check for initialized cutting manager object
     */
    bool isCuttingManagerInitialized();

    /**
     * @brief Set the Cut Lengths to be used for the cutting process.
     *
     * @param cut_lengths - Array of cut lengths available for the fabric cutting.
     * @return true - If the cut lengths are set successfully.
     * @return false - If the cut lengths are not set successfully.
     */
    bool setCutLengths(const std::vector<int> &cut_lengths);

    /**
     * @brief Set the System PLC Connection object.
     *
     * @param system_plc_config - System PLC configuration to be used for the cutting process.
     * @return true - If the system PLC connection is set successfully.
     * @return false - If the system PLC connection is not set successfully.
     */
    bool setSystemPLCConnection(json system_plc_config);

    /**
     * @brief Send the cutting plan to the system PLC.
     *
     * @param cut_length - The cut length which needs to be sent to the system PLC.
     * @return true - If the cutting plan is sent successfully.
     * @return false - If the cutting plan is not sent successfully.
     */
    bool sendCutLengthToCuttingMachine(int cut_length);

    /**
     * @brief Send the spout on flag to the cutting machine.
     *
     * @param spout_on - The flag to check if the spout on flag needs to be issued.
     * @return true - If the spout on flag is sent successfully.
     * @return false - If the spout on flag is not sent successfully.
     */
    bool sendSpoutToCuttingMachine(bool spout_on);

    /**
     * @brief Check if the machine is available for the cutting plan to write.
     *
     * @return true - If the machine is available for the cutting plan to write.
     * @return false - If the machine is not available for the cutting plan to write.
     */
    bool isMachineAvailable();

    /**
     * @brief Plan the next cut length and spout on flag to be used for the cutting process.
     *
     * @param defects - The defects to be used for the cutting process.
     * @param need_to_issue_spout - The flag to indicate if the spout on flag needs to be issued.
     * @param send_to_plc - The flag to indicate if the cutting plan needs to be sent to the system PLC.
     * @param top_bottom_selected - The flag to indicate if the top bottom mode is selected or not.
     * @return true - If the next cut length and spout on flag are planned successfully.
     * @return false - If the next cut length and spout on flag are not planned successfully.
     */
    bool planNextCutLength(std::vector<WorldMapDefect> &defects, bool need_to_issue_spout, bool send_to_plc = true, bool top_bottom_selected = false);

    /**
     * @brief Check if the cut lengths are available for the cutting process.
     *
     * @param current_defect_y - The current defect y coordinate to be used for the cutting process.
     * @param next_defect_y - The next defect y coordinate to be used for the cutting process.
     * @return int - The available cut length for the cutting process.
     */
    int checkIfCutLengthsAvailable(int current_defect_y, int next_defect_y);

    /**
     * @brief Get the Cut Length Index.
     *
     * @param cut_length - The cut length to be used for the cutting process.
     * @return int - The index of the cut length.
     */
    int getCutLengthIndex(int cut_length);

    /**
     * @brief Get the Cut Length.
     *
     * @return cut_length - The cut length to be used for the cutting process.
     * @param int - The index of the cut length.
     */
    int getCutLengthFromIndex(int index);

    /**
     * @brief Get the PLC Connection Details.
     *
     * @return json - The PLC connection details to be used for the cutting process.
     */
    json getPLCConnectionDetails();

    /**
     * @brief Get the Next Available Cut Length value
     *
     * @return int - The next available cut length value
     */
    int nextCutLength();

    /**
     * @brief Retrieves the last planned body.
     *
     * @return LastPlan The last planned body object.
     */
    LastPlan getLastPlannedBody();

    /**
     * @brief Reset last planned body.
     */
    void resetLastPlannedBody()
    {
        lastPlannedBody.reset();
    }

    /**
     * @brief Increments the body count by the specified length.
     *
     * @param length The number of bodies to increment the count by.
     */
    void incrementBodyCount(std::string type);

    /**
     * @brief Reset the body counters.
     *
     * @param primary_count - The primary body count to be used for the cutting process.
     * @param secondary_count - The secondary body count to be used for the cutting process.
     * @param tertiary_count - The tertiary body count to be used for the cutting process.
     * @param defective_count - The defective body count to be used for the cutting process.
     */

    void resetBodyCounters(int primary_count, int secondary_count, int tertiary_count, int defective_count);

    void acknowledgeCycleStart(bool is_cycle_start_electrical);

    /**
     * @brief Get the Next Spout On value
     *
     * @return true - If the next spout on flag is true
     * @return false - If the next spout on flag is false
     */
    bool spoutOn();

    /**
     * @brief Get the Is Next Body Contains Defect value
     *
     * @return true - If the next body contains defect flag is true
     * @return false - If the next body contains defect flag is false
     */
    bool defectPresent();

    /**
     * @brief Get the Next Available Cut Length is Force to cut fixed length.
     *
     * @return false - If the cutting length is not forced to cut fixed length.
     * @return true - If the cutting length is forced to cut fixed length.
     */
    bool isForcedToCutFixedLength();

    /**
     * @brief Is the cutting length  primary.
     *
     * @return true - If the cutting length is primary.
     * @return false - If the cutting length is not primary.
     */
    bool isPrimary();

    /**
     * @brief Is the cutting length primary with 10% of tolerance.
     *
     * @return true - If the cutting length is primary.
     * @return false - If the cutting length is not primary.
     */
    bool isPrimaryLength(int length);

    /**
     * @brief Get the Color For Cutting Plan object
     *
     * @return cv::Scalar - color for the cutting plan.
     */
    cv::Scalar getColorForCuttingPlan();
    /**
     * @brief Get the Color For popup of defect in autocut mode
     *
     * @return cv::Scalar - color for the cutting plan.
     */
    cv::Scalar getColorForDefectPopup();

    /**
     * @brief  Get the cutting counter and next cut length
     *
     * @return json - The cutting counter and next cut length.
     */
    json getCuttingCounterAndNextCutLength();

    /**
     * @brief Get the Next Available Cut Length Initials.
     *
     * @return std::string - The next available cut length initials
     */
    std::string getCutLengthInitials();

    /**
     * @brief Write Cut len
     *
     * @return
     */
    bool writeCutLength(uint16_t cut_len);

    /**
     * @brief Write Cycle Start
     *
     * @return
     */
    bool writeCycleStart();

    /**
     * @brief get the config params
     *
     * @return
     */
    json getSystemPLCConfig();
};

#endif // CUTTING_MANAGER_HPP
