/**
 * @file worldMap.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Map all panel positions and defects with the world map
 * @version 1.0
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <realWorldMapNode.hpp>

/**
 * @brief Construct a new Real World Map:: Real World Map object
 *
 */
RealWorldMapNode::RealWorldMapNode() : interval_task_pulse_error_logger(5 * 10000)
{
    ROS_INFO("Loading basic params...");

    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[RealWorldMap Node]Couldn't read all params from launch file. Please check names and types!";
    }
    else
    {
        std::cout << "\033[1;34m [RealWorldMapNode] dancer_end2stopper_distance_mm:" << dancer_end2stopper_distance_mm << "\033[0m" << std::endl;
    }
    std::vector<SystemState> realworld_state = {
        {"AVAIBLE", 0},
        {"CONNECTED", 1},
        {"RUNNING", 2},
        {"FAULT", 3}};

    initPubSub();
    initClient();
    initParams();
    waitUntilCamerasAreReady();

    sst = new componentStateTrackerLogger(&logger, std::chrono::seconds(60)); // System state tracker object to track the system state.

    body_tower_light = new BodyTowerLight(&plcComm);

    processAndUpdateWorldMap();
}

/**
 * @brief Destroy the Real World Map:: Real World Map object
 *
 */
RealWorldMapNode::~RealWorldMapNode()
{
    panel_positions.clear();
    new_master_defects.clear();
    delete report_image_saver;
    delete sst;
}

/**
 * @brief Function to read the ROS parameters.
 *
 * @return true - If the parameters are read successfully.
 * @return false - If the parameters are not read successfully.
 */
bool RealWorldMapNode::readROSParams()
{
    if (!ros::param::get("~distance_from_bottom_to_top_camera_mm", distance_from_bottom_to_top_camera_mm))
    {
        distance_from_bottom_to_top_camera_mm = 0;
    }
    std::cout << "[RealWorldMapNode] distance_from_bottom_to_top_camera_mm:" << distance_from_bottom_to_top_camera_mm << std::endl;

    if (!ros::param::get("~system_installed_in_front_of_dancer", system_installed_in_front_of_dancer))
    {
        system_installed_in_front_of_dancer = false;
    }
    if (system_installed_in_front_of_dancer)
    {
        std::cout << "\033[1;34m [RealWorld Node] System installed in front of dancer sensor.\033[0m\n";
    }
    else
    {
        std::cout << "\033[1;34m [RealWorld Node] System installed with dancer sensor in it.\033[0m\n";
    }

    if (!ros::param::get("~auto_cut_debug_in_semi_auto_mode", auto_cut_debug_in_semi_auto_mode))
    {
        auto_cut_debug_in_semi_auto_mode = false;
    }
    return (ros::param::get("pixel_per_mm", y_px_per_mm) &&
            ros::param::get("number_of_cameras", number_of_cameras) &&
            ros::param::get("pulse_per_mm", pulse_per_mm) &&
            ros::param::get("frame_width", frame_width) &&
            ros::param::get("frame_height", frame_height) &&
            ros::param::get("project_id", project_id) &&
            ros::param::get("~dancer_control_on_just_cut_mode", dancer_control_on_just_cut_mode) &&
            ros::param::get("~spout_and_cutter_control_on_just_cut_mode", spout_and_cutter_control_on_just_cut_mode) &&
            // ros::param::get("~report_image_save_path", report_image_save_path) &&
            ros::param::get("~block_punching_for_defect_behind_punch", block_punching_for_defect_behind_punch) &&
            ros::param::get("~spout_control_via_cutting_machine_com", spout_control_via_cutting_machine_com) &&
            ros::param::get("~use_cutter_sensor_instead_machine_ready", use_cutter_sensor_instead_machine_ready) &&

            ros::param::get("~is_cycle_start_electrical", is_cycle_start_electrical) &&
            ros::param::get("~dancer_end2stopper_distance_mm", dancer_end2stopper_distance_mm) &&
            ros::param::get("~reporting_folder_name", reporting_folder_name) &&
            ros::param::get("~auto_cut_defect_offset_mm", auto_cut_defect_offset_mm) &&

            ros::param::get("~error_popup_threshold_m", error_popup_threshold_m) &&
            ros::param::get("~error_stop_threshold_m", error_stop_threshold_m) &&

            ros::param::get("buzzer_at_defect", buzzer_at_defect) &&
            ros::param::get("stop_time_after_defect", stop_time_after_defect) &&

            // current_cam2stopper_distance_mm is the distance from the bottom camera line to the stopper
            // for double layer, this is the distance from the bottom camera line to stopper
            ros::param::get("~cam2stopper_distance", const_original_cam2stopper_mm));
}

/**
 * @brief Initialize the camera node parameters.
 */
void RealWorldMapNode::initParams()
{
    report_image_save_path = boost::filesystem::path("/images/").string();

    // initialize the image saver for reporting
    report_image_saver = new RobroImageSaver("/images/", "image_", ".jpg");
    full_image_saver = new RobroImageSaver("/images/", "full_image_", ".jpg");

    current_cam2stopper_distance_mm = original_cam2stopper_mm = const_original_cam2stopper_mm;
    mapDrawer.updateMapSize(total_distance_x_mm, current_cam2stopper_distance_mm);
    mapDrawer.updateMapDistances(original_cam2stopper_mm, cut_length_mm, dancer_end2stopper_distance_mm_with_offset);
    cuttingManager.setCutLengths({cut_length_mm});
    body_meter_tracker.reset();
    resetPanelsPos();
}

/**
 * @brief Function to initialize the ros publishers and subscribers.
 */
void RealWorldMapNode::initPubSub()
{
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    notification_pub = n.advertise<std_msgs::String>("/gui/label/notification", 1);
    system_config_pub = n.advertise<std_msgs::String>("/gui/value/real_world_map", 1);
    stopped_defect_image_pub = n.advertise<weaving_inspection::StoppingDefectImages>("/gui/stopped_defect_image", 1);
    defect_map_image_pub = it.advertise("/gui/defect_map_image", 1);
    defect_log_pub = n.advertise<std_msgs::String>("/report/defect_log", 10);
    next_stopping_pub = n.advertise<std_msgs::String>("/gui/label/next_stopping", 1);
    stopping_window_close_pub = n.advertise<std_msgs::Empty>("/gui/value/stopping_window_close", 1);
    body_log_pub = n.advertise<std_msgs::String>("/report/body_log", 10);
    camera_config_sync_pub = n.advertise<std_msgs::Empty>("/cam/config_sync", 10);
    dancer_sensor_on_pub = n.advertise<std_msgs::Empty>("/cam/record_loop_close", 1);
    ai_cut_master_data_pub = n.advertise<std_msgs::String>("/gui/value/ai_cut_master_modal_data", 1);
    loading_screen_data_and_status_pub = n.advertise<std_msgs::String>("/gui/value/loading_screen_data_and_status", 1);
    ai_cut_master_modal_connection_status_pub = n.advertise<std_msgs::Bool>("/gui/value/ai_cut_master_modal_plc_connection_status", 1);
    ai_cut_master_modal_machine_ready_pub = n.advertise<std_msgs::Bool>("/gui/value/ai_cut_master_modal_machine_ready_status", 1);
    system_sync_pub = n.advertise<std_msgs::Empty>("/gui/sync/configuration", 1);

    metadata_change_sub = n.subscribe("/main/roll/metadata", 10, &RealWorldMapNode::metadataChangeCallback, this);
    gui_sync_sub = n.subscribe("/gui/sync/configuration", 1, &RealWorldMapNode::guiSyncCallback, this);
    reset_system_sub = n.subscribe("/main/roll/reset", 1, &RealWorldMapNode::resetWorldMapCallback, this);
    camera_config_sub = n.subscribe("/cam/config", 100, &RealWorldMapNode::recordCameraPositionCallback, this);
    job_set_ack_sub = n.subscribe("/cam/ack/job", 100, &RealWorldMapNode::updateCurrentRecipeCallback, this);
    combine_defects_sub = n.subscribe("/world_map/combine_defects", 1000, &RealWorldMapNode::combineDefectsCallback, this);
    show_last_popub_sub = n.subscribe("/gui/button/show_last_popup", 1, &RealWorldMapNode::showLastPopupCallback, this);
    cycle_start_button_sub = n.subscribe("/gui/button/cycle_start", 1000, &RealWorldMapNode::cycleStartCallback, this);
    write_cut_len_sub = n.subscribe("/gui/button/write_cut_length", 1000, &RealWorldMapNode::writeCutLengthCallback, this);
    ai_cut_master_modal_open_sub = n.subscribe("/gui/button/ai_cut_master_modal_open", 1, &RealWorldMapNode::aiCutMasterModalOpenCallback, this);

    logger_disconnected_state_sub = n.subscribe("/logger/disconnected", 1, &RealWorldMapNode::loggerDisconnectedCallback, this);
    running_meter_sub = n.subscribe("/gui/label/running_meter", 1, &RealWorldMapNode::runningMeterCallback, this);
}

/**
 * @brief Function to initialize the ros client.
 */
void RealWorldMapNode::initClient()
{
    ros::NodeHandle n;

    cam_pulse_client = n.serviceClient<weaving_inspection::GetPulseDistance>("get_pulse_distance");
    cam_last_loop_close_pulse_client = n.serviceClient<weaving_inspection::GetCurrentPulses>("get_camera_pulses_from_last_loop_close");
    get_last_id_client = n.serviceClient<weaving_inspection::InsertDataLog>("/report/insert/log");
}

void RealWorldMapNode::guiSyncCallback(const std_msgs::Empty &msg)
{
    json loader_message;
    std_msgs::String loaderdata_msg;
    if (ready_camera_ids.size() == number_of_cameras)
    {
        loader_message["message"] = "Cameras are ready!";
        loader_message["close_loader"] = true;
        loaderdata_msg.data = loader_message.dump();
        loading_screen_data_and_status_pub.publish(loaderdata_msg);
    }
    else
    {
        loader_message["message"] = "Waiting for cameras to be ready: " + std::to_string(ready_camera_ids.size()) + "/" + std::to_string(number_of_cameras);
        loader_message["close_loader"] = false;
        loaderdata_msg.data = loader_message.dump();
        loading_screen_data_and_status_pub.publish(loaderdata_msg);
    }
}

/**
 * @brief Function will be waiting until all the cameras are ready.
 * it uses camera configuration publisher and subscriber
 * to check the camera status.
 */
bool RealWorldMapNode::waitUntilCamerasAreReady()
{
    ready_camera_ids.clear();
    int number_of_times_to_check = 120;
    while (ready_camera_ids.size() < number_of_cameras)
    {
        // publish the camera config sync message
        camera_config_sync_pub.publish(std_msgs::Empty());
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (!number_of_times_to_check)
        {
            sendNotification("Code: CSY008 - Cameras are not ready!", "popup");
            addSystemLog("ERROR", "[RealWorld] Cameras are not ready! Check camera connections or Settings.", "CSY008");
            return false;
        }

        number_of_times_to_check--;
    }

    // publish the dancer sensor on message
    dancer_sensor_on_pub.publish(std_msgs::Empty());
    return true;
}

/**
 * @brief: Update the positions of the panel and add a new one if needed
 *
 * @param panel_move_forward_mm - The distance the panel has moved forward
 */
void RealWorldMapNode::updatePanelPositions(int panel_move_forward_mm)
{
    for (auto &position : panel_positions)
    {
        position -= panel_move_forward_mm;
    }
    // do not add panel if the cam2stopper is less than 2 times the original cam2stopper
    if (current_cam2stopper_distance_mm < original_cam2stopper_mm * 2 && current_cam2stopper_distance_mm > 0)
    {
        // can't check panel logic if we don't have panel positions
        // Add the panel position if the distance is greater than 2 times the cut length.
        if (!panel_positions.empty() && (current_cam2stopper_distance_mm - panel_positions.back()) >= cut_length_mm)
        {
            panel_positions.push_back(panel_positions.back() + cut_length_mm);
        }
    }
}

/**
 * @brief Update the world map with the new defects and camera positions and calculate the
 * punch block state, and defect for next stopping.
 *
 * @param move_forward_mm - The distance the panel has moved forward
 */
void RealWorldMapNode::updateRealWorld(int move_forward_mm)
{
    defect_present_within_cut_length = false;
    defect_present_within_half_cut_length = false;
    is_defect_available_for_next_stopping = false;
    is_mode_top_bottom_and_defect_passed = false;
    int sequence_id = -1;
    int64 cam2frame_start_distance_mm{0};

    updatePanelPositions(move_forward_mm);

    // These must be sorted with begin() pointing to the defect with least y value (can be -ve)
    for (auto it = new_master_defects.begin(); it != new_master_defects.end(); it++)
    {
        // Move the defect forward and recalculate it's position if it is still behind the dancer_end
        if (!system_installed_in_front_of_dancer && it->detection.bbox.center.y > dancer_end2stopper_distance_mm_with_offset)
        {
            if (sequence_id != it->sequence_id)
            {
                sequence_id = it->sequence_id;
                cam2frame_start_distance_mm = getDistanceFromCameraMM(sequence_id);
            }
            it->detection.bbox.center.y = current_cam2stopper_distance_mm - it->defect_y_coord_wrt_frame - cam2frame_start_distance_mm;
        }
        else
        {
            it->detection.bbox.center.y -= move_forward_mm;
        }

        double defect_end = it->detection.bbox.center.y - it->detection.bbox.size_y;
        double defect_start = it->detection.bbox.center.y + it->detection.bbox.size_y;

        if (defect_end > current_cam2stopper_distance_mm)
        {
            ROS_WARN("Defect end is greater than current_cam2stopper_distance_mm %ld mm=> %f defect end mm", current_cam2stopper_distance_mm, defect_end);
        }
        // if defect is disabled, we don't need to check for it
        if (it->is_defect_class_disabled)
        {
            continue;
        }

        // Check if multiple panels are present
        if (panel_positions.size() > 1)
        {
            // Do stopping only if defect is in current panel
            if (defect_end <= panel_positions[1])
            {
                // we're only stopping at last defect in panel
                if (get_last_defect_in_panel)
                {
                    is_defect_available_for_next_stopping = true;
                    defect_for_next_stopping = *it;
                }
            }
        }
        else if (get_last_defect_in_panel)
        {
            // if only one panel is present, we're stopping at very end of it we know
            is_defect_available_for_next_stopping = true;
            defect_for_next_stopping = *it;
        }

        if (top_bottom_panel_selected && it->detection.bbox.center.y < 0 && !ignore_inspection_on)
        {
            is_mode_top_bottom_and_defect_passed = true;
        }

        // We're stopping at all defects.
        // we want to stop at the first defect behind the cutter
        // is_defect_available_for_next_stopping won't be true for the first defect
        if (!is_defect_available_for_next_stopping && !get_last_defect_in_panel && it->detection.bbox.center.y > 0)
        {
            is_defect_available_for_next_stopping = true;
            defect_for_next_stopping = *it;
        }
        // works for autocut, recut and justcut
        if (cutting_mode != "semiauto" && cutting_mode != "justcut")
        {
            // add auto cut offset in auto cut mode and recut mode
            defect_start -= auto_cut_defect_offset_mm;
            defect_end += auto_cut_defect_offset_mm;

            // defect_present_within_cut_length = true;

            if (defect_end > 0 && defect_end < cut_length_mm + auto_cut_defect_offset_mm)
            {
                defect_present_within_cut_length = true;
            }

            if (defect_start > 0 && defect_start < cut_length_mm)
            {
                defect_present_within_cut_length = true;
            }
        }
        else
        {
            // Mark if defect present within cut_length
            // be careful
            if (defect_end < (cut_length_mm - 200) && defect_start > 0)
            {
                defect_present_within_cut_length = true;
            }

            // Mark if defect present within half cut length
            if (defect_end < (cut_length_mm / 2) && defect_start > 0)
            {
                defect_present_within_half_cut_length = true;
            }
        }
    }
}

/**
 * @brief Record the defects on the world map as well as add the defects to the list.
 *
 * @param detections - list of detections
 */
void RealWorldMapNode::recordDefects(vision_msgs::Detection2DArray &detections)
{
    json j = json::parse(detections.header.frame_id);
    int64 sequence_id = j["seq"];
    float current_meters = j["roll_info_total_meters"];
    int64 cam2frame_start_distance_mm = getDistanceFromCameraMM(sequence_id);
    // Convert the detections from image frame to world frame.
    for (auto &detection : detections.detections)
    {

        json j = json::parse(detection.header.frame_id);
        std::string cam_id = j["cam_id"];
        float x_px_per_mm = cam_config_map[cam_id]["x_px_per_mm"];
        int fov_x_start_mm = cam_config_map[cam_id]["fov_x_start_mm"];
        j["px_top_left_y"] =  detection.bbox.center.y - detection.bbox.size_y;
        j["px_top_left_x"] =  detection.bbox.center.x - detection.bbox.size_x;
        detection.bbox.center.x = fov_x_start_mm + detection.bbox.center.x / x_px_per_mm;
        detection.bbox.center.y = (frame_height - detection.bbox.center.y) / y_px_per_mm;
        detection.bbox.size_x = detection.bbox.size_x / x_px_per_mm;
        detection.bbox.size_y = detection.bbox.size_y / y_px_per_mm;
        j["top_left_y_mm"] =  detection.bbox.center.y - detection.bbox.size_y;

        detection.header.frame_id = j.dump();

        if (cam_config_map[cam_id]["is_top_camera"])
        {
            // current_cam2stopper_distance_mm is the distance from the bottom camera line to the stopper
            // for double layer, this is the distance from the bottom camera line to stopper
            // cam2stopper for is top camera should be less than that of cam2stopper.
            detection.bbox.center.y -= distance_from_bottom_to_top_camera_mm;
        }
    }

    // Keep defects sorted front to back
    if (!detections.detections.empty())
    {
        sort(detections.detections.begin(), detections.detections.end(),
             [](const vision_msgs::Detection2D &lhs,
                const vision_msgs::Detection2D &rhs)
             {
                // Sort based on which box will be closer to the cutter.
                return (lhs.bbox.center.y - lhs.bbox.size_y) <
                 (rhs.bbox.center.y - rhs.bbox.size_y); });
    }

    for (auto &detection : detections.detections)
    {
        last_defect_body_id++;
        Defect defect;
        WorldMapDefect this_defect;
        json j = json::parse(detection.header.frame_id);
        this_defect.cam_id = j["cam_id"];
        this_defect.full_frame_path = j["full_frame_path"];
        this_defect.defect_id = last_defect_body_id;
        this_defect.sequence_id = sequence_id;
        this_defect.defect_y_coord_wrt_frame = detection.bbox.center.y;
        detection.bbox.center.y = current_cam2stopper_distance_mm - detection.bbox.center.y - cam2frame_start_distance_mm;
        this_defect.detection = detection;
        float x_px_per_mm = cam_config_map[this_defect.cam_id]["x_px_per_mm"];

        this_defect.is_defect_class_disabled = j["is_disabled"];

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(detection.source_img, sensor_msgs::image_encodings::BGR8);
        cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(512, 256), cv::INTER_LINEAR);
        std::string cropped_defect_path = report_image_saver->save(cv_ptr->image);

        logger.add_dataset_file(dataset_id, cropped_defect_path);

        int defect_status = !this_defect.is_defect_class_disabled ? 1 : 0;
        double roll_info_total_meter = current_meters;
        defect.initDefect(
            this_defect.defect_id,         // defect_id
            roll_id,                       // roll_id
            std::stoi(this_defect.cam_id), // camid
            group_id,                      // group_id
            cropped_defect_path,           // cropped image_path
            this_defect.full_frame_path,   // full image_path
            !j["is_disabled"],            //_is_enabled
            detection.bbox.size_x * 2,    // _defect_width_mm
            detection.bbox.size_y * 2,    // _defect_height_mm
            j["top_left_y_mm"].get<double>() + roll_info_total_meter * 1000,  // _defect_top_left_y_mm
            detection.bbox.center.x - detection.bbox.size_x,  // _defect_top_left_x_mm
            detection.bbox.size_x * 2 * x_px_per_mm,  // _defect_width_px
            detection.bbox.size_y * 2 * y_px_per_mm,  // _defect_height_px
            j["px_top_left_y"], // _defect_top_left_y_px
            j["px_top_left_x"], // _defect_top_left_x_px
            j["sensitivity_x"],
            j["sensitivity_y"],
            j["required_sensitivity_x"],
            j["required_sensitivity_y"],
            j["title"], // defect_type
            detection.results[0].score  // confidence
        );

        if (is_roll_started)
        {
            std_msgs::String defect_msg;
            defect_msg.data = defect.toString();
            defect_log_pub.publish(defect_msg);
        }
        new_master_defects.push_back(this_defect);
    }
}


/**
 * @brief Record the cutter action. This will update the cut panel defects and
 * panel at cutter defects in sorted order. As well as update the panel positions.
 */
void RealWorldMapNode::recordCutterAction(bool stopping_is_underway)
{
    resetPanelsPos();

    // Here we can add to show the Defect Popup window in case of auto-cut and re-cut
    // For now same thing that is being done in recordStoppingAction can be done.

    // since we have cut, remove the defects that were beyond the cutter
    if (!stopping_is_underway)
    {
        for (auto it = new_master_defects.begin(); it != new_master_defects.end();)
        {
            if (it->detection.bbox.center.y - it->detection.bbox.size_y < 0)
            {
                it = new_master_defects.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
}

/**
 * @brief Record the stopping action. This will update the cut panel defects and
 * panel at cutter defects in sorted order. As well as update the panel positions.
 */
void RealWorldMapNode::recordStoppingAction(int stopped_for_seq_id)
{
    defect_pop_msg.defects.clear();
    std::vector<uint8_t> jpeg_data;
    cv::Mat defect_img = getProcessedBodyMap(stopped_for_seq_id);
    cv::imencode(".jpg", defect_img, jpeg_data);
    defect_pop_msg.stopping_img.header.frame_id = getWorldMapCamNames().dump();
    defect_pop_msg.stopping_img.format = "jpeg";
    defect_pop_msg.stopping_img.data = jpeg_data;
    int counter = 0;
    for (auto img : getStoppingPanelDefectsImages(stopped_for_seq_id))
    {
        counter++;
        sensor_msgs::CompressedImage defect;
        cv::Mat image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
        std::vector<uint8_t> jpeg_data;
        cv::imencode(".jpg", image, jpeg_data);
        defect.format = "jpeg";
        defect.data = jpeg_data;
        defect.header.frame_id = img.header.frame_id;
        defect_pop_msg.defects.push_back(defect);
    }
    if (counter > 0)
    {
        stopped_defect_image_pub.publish(defect_pop_msg);
    }
    for (auto it = new_master_defects.begin(); it != new_master_defects.end();)
    {
        if (it->detection.bbox.center.y - it->detection.bbox.size_y < 0 || it->sequence_id <= stopped_for_seq_id)
        {
            it = new_master_defects.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

/**
 * @brief update the panel positions.
 */
void RealWorldMapNode::resetPanelsPos()
{
    panel_positions.clear();
    for (int i = 0; i <= current_cam2stopper_distance_mm; i += cut_length_mm)
    {
        panel_positions.push_back(i);
    }
}

/**
 * @brief Reset the world map. This will release all the vectors and maps.
 */
void RealWorldMapNode::resetWorldMap()
{
    resetPanelsPos();
    new_master_defects.clear();

    defect_present_within_cut_length = false;
    defect_present_in_last_body = false;
    defect_present_within_half_cut_length = false;
    is_defect_available_for_next_stopping = false;
    defect_present_within_cut_length_before_reset = false;
    punching_block_state = false;
    check_for_stopping = false;
    // last_defect_body_id=0;
    stopping_defect_frame_id = -1;
    running_meter_at_cutter = 0;
    batchCountNotificationSend = false;
}

/**
 * @brief Get the Processed Body Map visualization image
 *
 * @return cv::Mat - processed body map
 */
cv::Mat RealWorldMapNode::getProcessedBodyMap(int stopped_for_seq_id)
{
    if (get_last_defect_in_panel)
    {
        return mapDrawer.getLastDefectInProcessPanelMap(cam_config_map, new_master_defects, stopped_for_seq_id);
    }
    return mapDrawer.getFirstDefectInProcessPanelMap(cam_config_map, new_master_defects, stopped_for_seq_id);
}

/**
 * @brief Get stopping panel defects images from the processing panel.
 *
 * @param stopped_for_seq_id - sequence id for which the stopping is issued.
 * @return std::vector<sensor_msgs::Image> - list of images of defects in processing panel.
 */
std::vector<sensor_msgs::Image> RealWorldMapNode::getStoppingPanelDefectsImages(int stopped_for_seq_id)
{
    std::vector<sensor_msgs::Image> cut_panel_defects_img;
    int count{1};
    int i = 0;
    for (auto defect : new_master_defects)
    {
        if (defect.detection.bbox.center.y - defect.detection.bbox.size_y < 0 || defect.sequence_id <= stopped_for_seq_id)
        {
            auto detection = defect.detection;
            json info;
            info["id"] = "D" + std::to_string(count);
            count++;
            info["type"] = detection.source_img.header.frame_id;
            info["cam_pose"] = cam_config_map[defect.cam_id]["position"];
            info["defect_disabled"] = defect.is_defect_class_disabled;
            detection.source_img.header.frame_id = info.dump();
            // for creating vector such that all stopping defect comes first and disabled defect comes last.
            if (defect.is_defect_class_disabled)
            {
                cut_panel_defects_img.push_back(detection.source_img);
            }
            else
            {
                cut_panel_defects_img.insert(cut_panel_defects_img.begin() + i, detection.source_img);
                i++;
            }
        }
    }
    return cut_panel_defects_img;
}

/**
 * @brief Get defect images in the next cut panel
 *
 * @return std::vector<sensor_msgs::Image> - list of images of defects in processing panel.
 */
std::vector<sensor_msgs::Image> RealWorldMapNode::getAutoCutNextBodyDefectImages(int next_cut_len)
{
    std::vector<sensor_msgs::Image> cut_panel_defects_img;
    int count{1};
    int i = 0;
    for (auto defect : new_master_defects)
    {
        if (defect.detection.bbox.center.y - defect.detection.bbox.size_y < next_cut_len)
        {
            auto detection = defect.detection;
            json info;
            info["id"] = "D" + std::to_string(count);
            count++;
            info["type"] = detection.source_img.header.frame_id;
            info["cam_pose"] = cam_config_map[defect.cam_id]["position"];
            info["defect_disabled"] = defect.is_defect_class_disabled;
            detection.source_img.header.frame_id = info.dump();
            // for creating vector such that all stopping defect comes first and disabled defect comes last.
            if (defect.is_defect_class_disabled)
            {
                cut_panel_defects_img.push_back(detection.source_img);
            }
            else
            {
                cut_panel_defects_img.insert(cut_panel_defects_img.begin() + i, detection.source_img);
                i++;
            }
        }
    }
    return cut_panel_defects_img;
}

/**
 * @brief Get the world map camera names
 *
 * @return json - list of camera names
 */
json RealWorldMapNode::getWorldMapCamNames()
{
    json names;
    for (auto config : cam_config_map)
    {
        std::string cam_pose = config.second["position"];
        names.push_back(cam_pose);
    }
    return names;
}

/**
 * @brief Send notification to the UI.
 *
 * @param msg - Message to be sent.
 * @param severity - Severity of the message.
 */
void RealWorldMapNode::sendNotification(std::string msg, std::string severity)
{
    json message;
    std_msgs::String notification_msg;
    message["msg"] = msg;
    message["severity"] = severity;
    notification_msg.data = message.dump();
    notification_pub.publish(notification_msg);
}

/**
 * @brief Log body info to reporting node using publisher.
 */
void RealWorldMapNode::logBodyInfo()
{
    std_msgs::String msg;
    weaving_inspection::InsertDataLog srv;
    int save_fabric_mm = 0;
    if (previous_body_processed_before_reset > 50 && roll_id != 0 && job_id != 0)
    {

        // autocut , recut mode
        if (cutting_mode != "semiauto" && cutting_mode != "justcut")
        {
            // don't just use the planned initials, verify that the actual length
            // is in the vicinity of the planned length. Else call it Defected. Initial = "D";
            auto last_planned_body = cuttingManager.getLastPlannedBody();

            if (last_planned_body.isValid)
            {
                // Generate new body log.
                last_inserted_body_id += 1;
                body_log = Body();
                body_log.body_id = last_inserted_body_id;
                body_log.job_id = job_id;
                body_log.robro_roll_id = roll_id;

                if (last_planned_body.type != "D")
                {
                    if (previous_body_processed_before_reset >= 0.9 * last_planned_body.length && previous_body_processed_before_reset <= 1.1 * last_planned_body.length)
                    {
                        body_log.body_cut_type = last_planned_body.type;
                        running_meter_at_cutter += last_planned_body.length / 1000.0;

                        // log the actual value of the cut length that was cut
                        body_log.actual_cut_length = previous_body_processed_before_reset;

                        // setting the fabric saved based on cutlenght given
                        save_fabric_mm = cut_length_mm - body_log.actual_cut_length;
                        body_log.estimated_length_saved = save_fabric_mm <= 0 ? 0 : save_fabric_mm;

                        cuttingManager.incrementBodyCount(body_log.body_cut_type);

                        body_log_checker = 0;
                    }
                    // If difference log it as D (defect) body
                    else
                    {
                        body_log.body_cut_type = "D";
                        running_meter_at_cutter += last_planned_body.length / 1000.0;

                        // Log the actual value of the cut length that was cut
                        body_log.actual_cut_length = previous_body_processed_before_reset;

                        // setting the fabric saved based on cutlenght given
                        save_fabric_mm = cut_length_mm - body_log.actual_cut_length;
                        body_log.estimated_length_saved = save_fabric_mm <= 0 ? 0 : save_fabric_mm;

                        cuttingManager.incrementBodyCount(body_log.body_cut_type);
                        body_log_checker++;

                        std::string message = "Code: CUTT011 - The cutting is not according to the plan, please check! "
                                              "Expected length: " +
                                              std::to_string(last_planned_body.length / 10.0) + " cm, " +
                                              "Actual length: " + std::to_string(previous_body_processed_before_reset / 10.0) + " cm, " +
                                              "Deviation: " + std::to_string(std::abs(static_cast<int64_t>(previous_body_processed_before_reset) - static_cast<int64_t>(last_planned_body.length)) / 10.0) + " cm.";

                        sendNotification(message, "warning");

                        // When we won't cut according to the plan continuously 10 times, then there is a notification and log
                        if (body_log_checker >= 10)
                        {
                            sendNotification(message, "popup");
                            addSystemLog("WARNING", message, "CUTT011");
                            body_log_checker = 0;
                            buzzerControl.beep(2);
                        }
                    }
                }
                else
                {
                    if (previous_body_processed_before_reset >= 0.9 * last_planned_body.length && previous_body_processed_before_reset <= 1.1 * last_planned_body.length)
                    {
                        body_log.body_cut_type = last_planned_body.type;
                        running_meter_at_cutter += last_planned_body.length / 1000.0;

                        // log the actual value of the cut length that was cut
                        body_log.actual_cut_length = previous_body_processed_before_reset;

                        // setting the fabric saved based on cutlenght given
                        save_fabric_mm = cut_length_mm - body_log.actual_cut_length;
                        body_log.estimated_length_saved = save_fabric_mm <= 0 ? 0 : save_fabric_mm;

                        cuttingManager.incrementBodyCount(body_log.body_cut_type);
                        defected_body_log_checker = 0;
                    }
                    else
                    {
                        // It still consider as defect body
                        body_log.body_cut_type = last_planned_body.type;
                        running_meter_at_cutter += last_planned_body.length / 1000.0;

                        // Log the actual value of the cut length that was cut
                        body_log.actual_cut_length = previous_body_processed_before_reset;

                        // setting the fabric saved based on cutlenght given
                        save_fabric_mm = cut_length_mm - body_log.actual_cut_length;
                        body_log.estimated_length_saved = save_fabric_mm <= 0 ? 0 : save_fabric_mm;

                        cuttingManager.incrementBodyCount(body_log.body_cut_type);
                        defected_body_log_checker++;

                        std::string message = "Code: CUTT011 - The cutting is not according to the plan, please check! "
                                              "Expected length: " +
                                              std::to_string(last_planned_body.length / 10.0) + " cm, " +
                                              "Actual length: " + std::to_string(previous_body_processed_before_reset / 10.0) + " cm, " +
                                              "Deviation: " + std::to_string(std::abs(static_cast<int64_t>(previous_body_processed_before_reset) - static_cast<int64_t>(last_planned_body.length)) / 10.0) + " cm.";

                        sendNotification(message, "warning");

                        // When we won't cut according to the plan continuously 10 times, then there is a notification and log
                        if (defected_body_log_checker >= 10)
                        {
                            sendNotification(message, "popup");
                            addSystemLog("WARNING", message, "CUTT011");
                            defected_body_log_checker = 0;
                            buzzerControl.beep(2);
                        }
                    }
                }
                cuttingManager.resetLastPlannedBody();
                is_body_planed_checker = 0;
                body_tower_light->resetLights();
            }
            else
            {
                // Generate new body log.
                last_inserted_body_id += 1;
                body_log = Body();
                body_log.body_id = last_inserted_body_id;
                body_log.job_id = job_id;
                body_log.robro_roll_id = roll_id;
                if (cuttingManager.isPrimaryLength(previous_body_processed_before_reset))
                {
                    body_log.body_cut_type = "P";
                }
                else
                {
                    body_log.body_cut_type = "D";
                }
                body_log.actual_cut_length = previous_body_processed_before_reset;
                cuttingManager.incrementBodyCount(body_log.body_cut_type);
                std::string message = "Code: CUTT011 - The cutting doesn’t match the plan. "
                                      "Please check the cutting settings. No plan found for cutting.";

                sendNotification(message, "warning");
                is_body_planed_checker++;
                if (is_body_planed_checker >= 10)
                {
                    sendNotification(message, "popup");
                    addSystemLog("WARNING", message, "CUTT011");
                    is_body_planed_checker = 0;
                    buzzerControl.beep(2);
                }
            }

            // Resting because we logged the body
            previous_body_processed_before_reset = 0;
        }
        else // semiauto mode
        {
            last_inserted_body_id += 1;
            body_log = Body();
            body_log.body_id = last_inserted_body_id;
            body_log.job_id = job_id;
            body_log.robro_roll_id = roll_id;
            std::string _body_cut_type = "N/A";

            int cutLen = cut_length_mm; // primary cut length only

            if (previous_body_processed_before_reset >= 0.9 * cutLen && previous_body_processed_before_reset <= 1.1 * cutLen && !defect_present_in_last_body)
            {
                _body_cut_type = "P";
            }
            else
            {

                _body_cut_type = "D";
                defect_present_in_last_body = false;
            }

            cuttingManager.incrementBodyCount(_body_cut_type);

            body_log.body_cut_type = _body_cut_type;
            running_meter_at_cutter += (previous_body_processed_before_reset) / 1000.0;
            body_log.actual_cut_length = previous_body_processed_before_reset;

            // setting the fabric saved based on cutlenght given
            save_fabric_mm = cut_length_mm - previous_body_processed_before_reset;
            body_log.estimated_length_saved = save_fabric_mm <= 0 ? 0 : save_fabric_mm;

            // Resting because we logged the body
            previous_body_processed_before_reset = 0;
        }
        body_log.cut_position_in_roll = running_meter_at_cutter;
        // body_log.punch_saved = is_prev_punch_blocked;
        body_log.punch_saved = 0;
        if (is_roll_started)
        {

            if (body_log.body_cut_type == "P")
            {
                body_meter_tracker.primary_meter += cuttingManager.getCutLengthFromIndex(0) / 1000.0;
                if (current_work_order.isValid)
                {

                    current_work_order.current_pcs += 1;
                    current_work_order.batch_current_pcs += 1;
                    body_log.work_order_id = work_order_id;
                    // update in DB
                    logger.update_work_order_current_pcs(current_work_order.work_order_id, current_work_order.current_pcs);
                }

                if (current_batch_count.isValid)
                {
                    batchCountNotificationSend = false;
                    current_batch_count.current_pcs += 1;
                }
            }
            else if (body_log.body_cut_type == "S")
            {
                body_meter_tracker.secondary_meter += cuttingManager.getCutLengthFromIndex(1) / 1000.0;
            }
            else if (body_log.body_cut_type == "T")
            {
                body_meter_tracker.tertiary_meter += cuttingManager.getCutLengthFromIndex(2) / 1000.0;
            }
            else if (body_log.body_cut_type == "D")
            {

                body_meter_tracker.defective_meter += body_log.actual_cut_length / 1000.0;
            }
            msg.data = body_log.toString();
            body_log_pub.publish(msg);
        }
    }
}

/**
 * @brief Process the cutting action and update the cutter status.
 *
 */
void RealWorldMapNode::resetValuesAtCuttingAction()
{
    if (defect_present_within_cut_length)
    {
        defect_present_within_cut_length_before_reset = defect_present_within_cut_length;
    }

    is_prev_punch_blocked = false;
    defect_present_within_cut_length = false;
    defect_present_within_half_cut_length = false;

    is_defect_available_for_next_stopping = false;

    if (previous_body_processed_mm != 0)
    {
        previous_body_processed_before_reset = previous_body_processed_mm;
    }

    // make sure previous body is zero when cutter gets turn on.
    previous_body_processed_mm = 0;

    is_stopping_window_open = false;
    stopping_window_close_pub.publish(std_msgs::Empty());
    total_processed_body_mm_after_last_stopping = 0;
    // to make sure that current body process should be zero.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

/**
 * @brief Get the Distance Move By Current Panel in mm in Real World.
 *
 * @param current_body_processed_mm - current body processed in mm.
 * @return uint64_t - distance moved by current panel.
 */
int64_t RealWorldMapNode::getMoveForwardMM(uint64_t current_body_processed_mm)
{
    int64_t moved_distance = 0;
    if (current_body_processed_mm == 0)
    {
        if (previous_body_processed_mm != 0)
        {
            previous_body_processed_before_reset = previous_body_processed_mm;
        }
        previous_body_processed_mm = 0;
    }

    if (current_body_processed_mm != previous_body_processed_mm)
    {

        moved_distance = current_body_processed_mm - previous_body_processed_mm;

        // HANDLING DEFECT JUMPING - happens when the PLC Makes the current_body 0, before we record our cutting event.
        if (moved_distance < 0 && abs(moved_distance) > (0.7 * cut_length_mm))
        {
            moved_distance = cut_length_mm - previous_body_processed_mm;

            if (previous_body_processed_mm != 0)
            {
                previous_body_processed_before_reset = previous_body_processed_mm;
            }

            previous_body_processed_mm = 0;
            sendNotification("[RealWorld] Defect Jumping Handled. Current_body: " + std::to_string(current_body_processed_mm) + " Previous_body: " + std::to_string(previous_body_processed_mm), "warning");
            addSystemLog("WARNING", "[RealWorld] Defect Jumping Handled. Current_body: " + std::to_string(current_body_processed_mm) + " Previous_body: " + std::to_string(previous_body_processed_mm), "APP018");
        }
        else
        {
            previous_body_processed_mm = current_body_processed_mm;
        }
    }
    return moved_distance;
}

/**
 * @brief Estimate the camera to stopper distance in real world.
 *
 * @param current_fabric_moved_under_cutter_mm - current fabric moved under cutter in mm.
 * @param current_fabric_moved_under_camera_mm - current fabric moved under camera in mm.
 */
void RealWorldMapNode::updateCurrentCam2StopperDistance(uint64_t current_fabric_moved_under_cutter_mm, uint64_t current_fabric_moved_under_camera_mm)
{
    if (prev_fabric_move_under_cutter != current_fabric_moved_under_cutter_mm ||
        prev_fabric_moved_under_camera_mm != current_fabric_moved_under_camera_mm)
    {
        prev_fabric_move_under_cutter = current_fabric_moved_under_cutter_mm;
        prev_fabric_moved_under_camera_mm = current_fabric_moved_under_camera_mm;

        uint64_t total_camera_mm = original_cam2stopper_mm + (int)current_fabric_moved_under_camera_mm;

        // Prevent reverse overflow of uint64_t. If camera motion is less than cutter motion, it may cause a Back Encoder Error.
        if (total_camera_mm >= current_fabric_moved_under_cutter_mm)
        {
            current_cam2stopper_distance_mm = total_camera_mm - current_fabric_moved_under_cutter_mm;
        }
        else
        {
            uint64_t diff = current_fabric_moved_under_cutter_mm - total_camera_mm;
            addSystemLog("WARNING", "[updateCurrentCam2StopperDistance] Not updating current cam2Stopper distance. "
                                    "Total fabric motion under camera (" +
                                        std::to_string(total_camera_mm) + " mm) is less than under cutter (" + std::to_string(current_fabric_moved_under_cutter_mm) + " mm). Difference: " + std::to_string(diff) + " mm.",
                         "APP020");
        }

        mapDrawer.updateMapSize(total_distance_x_mm, current_cam2stopper_distance_mm);
    }

    // Since I have a limit of 90 Meter after which my buffer will overflow in PLC.
    // 65000/7.2 = 9. something. We have a *10 factor, so 90 meter.
}

/**
 * @brief Publish Last autocut pop data.
 *
 */
void RealWorldMapNode::showLastPopupCallback(const std_msgs::Empty &msg)
{
    if (!defect_pop_msg.stopping_img.data.empty())
    {
        stopped_defect_image_pub.publish(defect_pop_msg);
        total_processed_body_mm_after_last_stopping = 0;
        is_stopping_window_open = true;
    }
}

/**
 * @brief Process the defects and update the world map.This function is contains infinite loop which will
 * be running in main thread and do all service calling from PLC and update the world map.
 */
void RealWorldMapNode::processAndUpdateWorldMap()
{

    // Can Add state machine for ON, OFF, ENGAGED here.

    ros::Rate rate(20);
    uint64_t defect_distance_from_stopper_mm{0};
    uint64_t last_stopping_command_given_at_mm{0};
    auto last_notification_time = std::chrono::high_resolution_clock::now();
    json real_world_values;
    real_world_values["last_loop_close_time"] = getTimeStr();
    uint64_t current_fabric_moved_under_camera_mm{0};
    uint64_t current_fabric_moved_under_cutter_mm{0};
    ExecutionTimer timer;
    // Error Tracker
    ErrorTracker errorTracker(error_popup_threshold_m, error_stop_threshold_m);
    // interval task monitor objects
    IntervalTask real_world_values_publish_interval(500);
    IntervalTask interval_task_roll_off_read_front_encoder(5000);
    IntervalTask interval_task_publish_machine_ready_for_display(250);
    IntervalTask interval_task_db_disconnection(10 * 1000);
    IntervalTask interval_task_log_function_fail(10 * 1000);

    uint64_t last_distance_system_not_engaged_popup = 0;

    int auto_cut_next_pos = -1;
    bool is_cutter_on_in_this_run_cycle = false;
    bool kwis_plc_connection_pop_up_once = false;
    bool cutting_mc_connection_pop_up_once = false;
    bool log_dancer_sensor_or_fabric_sync_error{false};
    bool log_cutter_sensor_error{false};
    bool planning_next_cut_length_enabled{true};

    // Set a parameter server to announce to everyone if cutting manager (cutting machine plc is connected)
    // We can use this if we want to show option of using AUto-Cut/Re-Cut in the UI
    ros::NodeHandle nh;
    nh.setParam("cutting_machine_plc_connected", cuttingManager.isCuttingManagerInitialized());

    while (ros::ok())
    {
        timer.start();

        if (is_reset_requested)
        {
            is_reset_requested = false;
            plcComm.resetPLC();
            resetWorldMap();
            errorTracker.reset();
            body_tower_light->resetLights();
            cuttingManager.resetLastPlannedBody();
            buzzerControl.turnOff();

            defected_body_log_checker = 0; // checking for defected body log
            is_body_planed_checker = 0;
            body_log_checker = 0; // cheking for body log
        }

        if (is_roll_started)
        {
            if (sst->changeState(ComponentState::RUNNING_))
            {

                last_inserted_system_id = sst->getLastStateId();

                addSystemLog("INFO", "Component State: ON_ENGAGED", "INFO011");
            }
            else if (sst->checkLogTimeout())
            {

                sst->updateStateLog(last_inserted_system_id);
            }

            bool need_to_issue_block_signal = false;

            std_msgs::String msg;

            is_cutter_on_in_this_run_cycle = plcComm.isCutterOn();

            if (previous_body_processed_mm == 0)
            {
                cutter_sensor.updateState(is_cutter_on_in_this_run_cycle, previous_body_processed_before_reset);
            }
            else
            {
                cutter_sensor.updateState(is_cutter_on_in_this_run_cycle, previous_body_processed_mm);
            }

            bool is_cutter_falling_edge_on = cutter_sensor.hasFallingEdge();

            if (is_cutter_falling_edge_on)
            {

                // Reset the current body processed value here because cutter is on
                if (plcComm.getCurrentBodyProcessedResetMode())
                {
                    // auto reset is disabled we have to reset.
                    plcComm.resetCurrentBodyProcessed();
                }

                errorTracker.update(running_meter, true); // update error on cutter sensor on

                recordCutterAction(check_for_stopping);

                resetValuesAtCuttingAction();

                if (cutting_mode == "semiauto" || cutting_mode == "justcut")
                {
                    logBodyInfo();
                }

                if (cutting_mode == "justcut")
                {
                    if (auto_cut_next_pos <= 0)
                    {
                        // Increment the body count
                        // auto last_planned_body = cuttingManager.getLastPlannedBody();
                        // if (last_planned_body.isValid)
                        // {
                        //     cuttingManager.incrementBodyCount(body_log.body_cut_type);
                        // }

                        cuttingManager.planNextCutLength(new_master_defects,
                                                         roll_info_punching_on, false);
                        auto_cut_next_pos = cuttingManager.nextCutLength();
                    }
                }
                if (log_cutter_sensor_error)
                {
                    log_cutter_sensor_error = false;
                }

                if (defect_present_within_cut_length_before_reset)
                {
                    defect_present_in_last_body = true;
                    defect_present_within_cut_length_before_reset = false;
                }
            }

            if (cutting_mode != "semiauto" && cutting_mode != "justcut")
            {
                // HS: If this is the first body after start is pressed, then don't wait for cutter_falling_edge

                std::string last_body_initials = cuttingManager.getCutLengthInitials();

                if (is_cutter_falling_edge_on)
                {
                    logBodyInfo();
                    if (cuttingManager.defectPresent() && buzzer_at_defect == "end")
                    {
                        buzzerControl.beep(2);
                    }
                }

                if ((use_cutter_sensor_instead_machine_ready && (is_cutter_falling_edge_on || bypass_cutter_sensor_once)) ||
                    (!use_cutter_sensor_instead_machine_ready && cuttingManager.isMachineAvailable()))
                {
                    if (bypass_cutter_sensor_once)
                    {
                        bypass_cutter_sensor_once = false; // setting it to false after one cycle
                    }

                    // We will not sleep if the body is forced to cut fixed length because the next body is also going to be a defect.
                    if (cuttingManager.defectPresent() || !cuttingManager.isPrimary() && !cuttingManager.isForcedToCutFixedLength())
                    {
                        std::this_thread::sleep_for(std::chrono::seconds(stop_time_after_defect));
                    }
                    if (planning_next_cut_length_enabled)
                    {

                        if (cuttingManager.planNextCutLength(new_master_defects,
                                                             roll_info_punching_on, true, top_bottom_panel_selected))
                        {
                            if (is_cycle_start_electrical)
                            {
                                plcComm.setCycleStartTo(true);
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                plcComm.setCycleStartTo(false);
                            }

                            // move log body info to line 981
                            auto_cut_next_pos = cuttingManager.nextCutLength();

                            // Tower lamp
                            auto last_planned_body = cuttingManager.getLastPlannedBody();
                            body_tower_light->setBodyColor(last_planned_body.type);

                            // Popup to show situation when non-primary or defect panel is cut but not a force to cut fixed length
                            if (cuttingManager.defectPresent() && !cuttingManager.isForcedToCutFixedLength())
                            {
                                defect_pop_msg.defects.clear();

                                std::vector<uint8_t> jpeg_data;

                                cv::Mat defect_img = mapDrawer.getAutoCutMapVisualization(cam_config_map, new_master_defects, auto_cut_next_pos, cuttingManager.getColorForDefectPopup());
                                cv::imencode(".jpg", defect_img, jpeg_data);
                                defect_pop_msg.stopping_img.header.frame_id = getWorldMapCamNames().dump();
                                defect_pop_msg.stopping_img.format = "jpeg";
                                defect_pop_msg.stopping_img.data = jpeg_data;
                                int c_counter = 0;
                                for (auto img : getAutoCutNextBodyDefectImages(auto_cut_next_pos))
                                {
                                    c_counter++;
                                    sensor_msgs::CompressedImage defect;
                                    cv::Mat image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
                                    std::vector<uint8_t> jpeg_data;
                                    cv::imencode(".jpg", image, jpeg_data);
                                    defect.format = "jpeg";
                                    defect.data = jpeg_data;
                                    defect.header.frame_id = img.header.frame_id;
                                    defect_pop_msg.defects.push_back(defect);
                                }

                                stopped_defect_image_pub.publish(defect_pop_msg);
                                total_processed_body_mm_after_last_stopping = 0;
                                is_stopping_window_open = true;
                            }

                            if (cuttingManager.defectPresent() && buzzer_at_defect == "start")
                            {
                                buzzerControl.beep(2);
                            }
                        }
                    }
                }

                cuttingManager.acknowledgeCycleStart(is_cycle_start_electrical);
            }
            else
            {
                if (auto_cut_debug_in_semi_auto_mode)
                {
                    if (auto_cut_next_pos <= 0)
                    {
                        cuttingManager.planNextCutLength(new_master_defects,
                                                         roll_info_punching_on, false);
                        auto_cut_next_pos = cuttingManager.nextCutLength();
                    }
                }
                else
                {
                    auto_cut_next_pos = -1;
                }
            }

            // check logger state and if ready for popup then show the notification
            if (isLoggerDisconnected() && interval_task_db_disconnection.isReady())
            {
                is_immediate_stopping_requested = true;
                sendNotification("Code: DB001 - Database disconnection has occurred.", "popup");
                interval_task_db_disconnection.reset();
            }

            uint64_t current_body_processed_mm = plcComm.getCurrentBodyProcessedMM();
            int64_t current_move_forward_mm = getMoveForwardMM(current_body_processed_mm);

            // Cutter sensor is not working
            if (current_body_processed_mm > 2 * cut_length_mm)
            {
                // track error for the specific code
                Error err({ComponentType::SENSOR, ComponentModel::CUTTER_SENSOR}, ErrorCode::SHW003, " Cutter Sensor May not be working.", running_meter);
                errorTracker.addError(err); // tracking error

                if (!log_cutter_sensor_error)
                {
                    buzzerControl.beep(2); // Update here
                    log_cutter_sensor_error = true;
                    addSystemLog("ERROR", "[RealWorld] Cutter Sensor May not be working. Current_body: " + std::to_string(current_body_processed_mm) + " Cut Length: " + std::to_string(cut_length_mm), "SHW003");
                }
            }

            if (cutting_mode != "semiauto" && cutting_mode != "justcut")
            {
                if (is_stopping_window_open)
                {
                    // close the stopping window
                    total_processed_body_mm_after_last_stopping += current_move_forward_mm;
                    if (total_processed_body_mm_after_last_stopping >= 500)
                    {
                        total_processed_body_mm_after_last_stopping = 0;
                        stopping_window_close_pub.publish(std_msgs::Empty());
                        is_stopping_window_open = false;
                    }
                }
            }

            //  Get call is success or not
            bool get_cam_pulse_success = getCameraPulsesFromLastLoopClose(cam_last_loop_close_pulse);
            bool get_plc_fabric_success = plcComm.getFabricMovedUnderCutterMM(current_fabric_moved_under_cutter_mm);

            // update if get camera pulse success
            if (get_cam_pulse_success)
            {
                current_fabric_moved_under_camera_mm = cam_last_loop_close_pulse / pulse_per_mm;
            }

            // proceed only if both value fetching is success
            if (get_cam_pulse_success && get_plc_fabric_success)
            {

                // Only Update Cam to Stopper Distance if system is not installed in front of dancer.
                if (!system_installed_in_front_of_dancer)
                {
                    updateCurrentCam2StopperDistance(current_fabric_moved_under_cutter_mm, current_fabric_moved_under_camera_mm);
                }

                speed_calculator.calculatesSpeedAndStore(current_fabric_moved_under_cutter_mm);
                if (speed_calculator.shouldLog())
                {
                   logger.logSpeedData(roll_id,running_meter,speed_calculator.getAverageSpeed());
                    speed_calculator.markLogged();
                }

                // with awareness of which is bigger, get the difference in int.
                int motion_differece_between_back_and_front_encoders =
                    (current_fabric_moved_under_camera_mm > current_fabric_moved_under_cutter_mm)
                        ? (current_fabric_moved_under_camera_mm - current_fabric_moved_under_cutter_mm)
                        : (current_fabric_moved_under_cutter_mm - current_fabric_moved_under_camera_mm);

                // My Current fabric under cutter isn't getting reset - so I am sure dancer sensor hasn't turned on.
                // Check Dancer sensor not on at 20m
                if (current_fabric_moved_under_cutter_mm >= 20 * 1000)
                {
                    Error err({ComponentType::SENSOR, ComponentModel::DANCER_SENSOR}, ErrorCode::SHW009, " Dancer Sensor not turning on. Cam2Stopper: " + std::to_string(current_cam2stopper_distance_mm) + "mm Cutter: " + std::to_string(current_fabric_moved_under_cutter_mm) + " mm Camera: " + std::to_string(current_fabric_moved_under_camera_mm) + " mm", running_meter);
                    errorTracker.addError(err); // tracking error

                    if (!log_dancer_sensor_or_fabric_sync_error)
                    {
                        buzzerControl.beep(2); // Update here
                        log_dancer_sensor_or_fabric_sync_error = true;
                        addSystemLog("ERROR", err.codeStr() + err.message, err.codeStr());
                    }
                }

                // If motion difference, 90% of cut length
                if (motion_differece_between_back_and_front_encoders > cut_length_mm * 0.9)
                {
                    // Check which is behind front or back encoder
                    ErrorCode err_code = (current_fabric_moved_under_camera_mm > current_fabric_moved_under_cutter_mm) ? ErrorCode::SHW004 : ErrorCode::SHW011;
                    ComponentModel component_model = (err_code == ErrorCode::SHW004) ? ComponentModel::FRONT_ENCODER : ComponentModel::BACK_ENCODER;
                    std::string error_msg_perfix = (err_code == ErrorCode::SHW004) ? " Front Encoder Error." : " Back Encoder Error.";

                    /*** Tracking encoder error ***/
                    Error err({ComponentType::ENCODER, component_model}, err_code, error_msg_perfix + " Cam2Stopper: " + std::to_string(current_cam2stopper_distance_mm) + "mm Cutter: " + std::to_string(current_fabric_moved_under_cutter_mm) + " mm Camera: " + std::to_string(current_fabric_moved_under_camera_mm) + " mm", running_meter);
                    errorTracker.addError(err);

                    if (!log_dancer_sensor_or_fabric_sync_error)
                    {
                        buzzerControl.beep(2);
                        log_dancer_sensor_or_fabric_sync_error = true;
                        addSystemLog("ERROR", err.codeStr() + err.message, err.codeStr());
                    }
                }
            }
            else
            {
                // Add system log
                // ToDo : Add error code
                std::string errorMessage = "Failed ";

                if (!get_cam_pulse_success)
                {
                    errorMessage += "[ Call to getCameraPulsesFromLastLoopClose() ] ";
                }

                if (!get_plc_fabric_success)
                {
                    errorMessage += "[ Call to getFabricMovedUnderCutterMM() ].";
                }

                // log at particual frequency (10sec interval)
                if (!errorMessage.empty() && interval_task_log_function_fail.isReady())
                {
                    addSystemLog("ERROR", errorMessage, "CAM015");
                    interval_task_log_function_fail.reset();
                }
            }

            // Planning enabled only if current cam to stopper distance is greater than cut length + auto cut defect offset + 500mm
            // This is to ensure that we have enough fabric to cut and also to avoid false positives
            planning_next_cut_length_enabled = current_cam2stopper_distance_mm > (cut_length_mm + auto_cut_defect_offset_mm + 500);

            // update error tracker with current running meter
            errorTracker.update(running_meter);

            /********** Check Error Tracker for Popup and Stopping **********/
            Error error;
            if (errorTracker.shouldPop(error, running_meter))
            {
                // sendNotification(error.codeStr() + error.message, "popup");
                // Not giving popup only giving beep()
                buzzerControl.beep(2);
            }
            if (errorTracker.shouldStop(error, running_meter))
            {
                is_immediate_stopping_requested = true;
                sendNotification(error.codeStr() + error.message, "popup");
                buzzerControl.beep(4);
            }
            /***************************************************************/

            // Check work order is completed or not
            if (current_work_order.isCompleted())
            {
                is_immediate_stopping_requested = true;

                std::string message = "Work order \"" + current_work_order.order_name +
                                      "\" completed successfully with " +
                                      std::to_string(current_work_order.target_pcs) + " pieces.";

                sendNotification(message, "popup");
                buzzerControl.beep(2);

                // reset current work order
                logger.update_work_order_status(current_work_order.work_order_id, 1);
                current_work_order.reset();

                // sync with UI
                system_sync_pub.publish(std_msgs::Empty());
            }

            if (current_batch_count.isCompleted() && !batchCountNotificationSend)
            {
                is_immediate_stopping_requested = true;

                sendNotification("The batch is completed", "info");
                batchCountNotificationSend = true;
                buzzerControl.beep(2);
            }

            auto_cut_next_pos -= current_move_forward_mm;
            bool is_dancer_on = plcComm.isConstantDistanceSensorOn();

            real_world_values["cutter_sensor_on_off_state"] = is_cutter_on_in_this_run_cycle;
            real_world_values["dancer_sensor_on_off_state"] = is_dancer_on;
            real_world_values["cutting_plan_status"] = cuttingManager.cycle_event_tracker.getCycleStatusColorHex();
            real_world_values["current_body_processed_mm"] = current_body_processed_mm;

            real_world_values["cutting_cycle_state"] = cuttingManager.cycle_event_tracker.getCurrentStateString();

            real_world_values["last_five_cutter_encoder_values"] = cutter_sensor.getLastFiveEncoderValues();
            real_world_values["last_five_cutter_activation_times"] = cutter_sensor.getLastFiveTimestamps();

            dancer_sensor.updateState(is_dancer_on, current_fabric_moved_under_cutter_mm, current_fabric_moved_under_camera_mm);

            real_world_values["last_five_dancer_front_encoder_values"] = dancer_sensor.getLastFiveFrontEncoderValues();
            real_world_values["last_five_dancer_back_encoder_values"] = dancer_sensor.getLastFiveBackEncoderValues();
            real_world_values["last_five_dancer_activation_times"] = dancer_sensor.getLastFiveActivationTimes();
            real_world_values["last_five_loop_close"] = dancer_sensor.getLastFiveDifferences();

            real_world_values["last_five_planning_times"] = cuttingManager.cycle_event_tracker.getLastFivePlanningTimes();
            real_world_values["last_five_start_times"] = cuttingManager.cycle_event_tracker.getLastFiveCycleStartTimes();
            real_world_values["last_five_waiting_times"] = cuttingManager.cycle_event_tracker.getLastFiveAcknowledgeTimes();

            real_world_values["average_loop_time_ms"] = timer.getAverageDurationInMilliseconds();

            if (dancer_sensor.hasRisingEdge())
            {
                // reset loop close
                plcComm.resetFrameCounters();
                dancer_sensor_on_pub.publish(std_msgs::Empty());

                last_loop_close_fabric_mm_diff = current_fabric_moved_under_cutter_mm - current_fabric_moved_under_camera_mm;

                if (abs(last_loop_close_fabric_mm_diff) > 500)
                {
                    addSystemLog("WARNING", "[RealWorld] Loop Not Closed. MISMATCH BY: " + std::to_string(last_loop_close_fabric_mm_diff), "SHW010");
                }

                real_world_values["last_cam_mm"] = current_fabric_moved_under_camera_mm;
                real_world_values["last_cutter_mm"] = current_fabric_moved_under_cutter_mm;
                real_world_values["last_loop_close_value"] = last_loop_close_fabric_mm_diff;
                real_world_values["last_loop_close_time"] = getTimeStr();

                errorTracker.update(running_meter, false, true); // update error on dancer sensor on

                if (log_dancer_sensor_or_fabric_sync_error)
                {
                    log_dancer_sensor_or_fabric_sync_error = false;
                    std::string err_msg = "Dancer sensor turned on after Fabric Sync Error";
                    addSystemLog("INFO", err_msg, "INFO008");
                }
            }

            // Add new defects into the WorldMap
            while (!combine_defects.empty())
            {
                recordDefects(combine_defects.front());
                combine_defects.pop();
            }

            updateRealWorld(current_move_forward_mm);

            real_world_values["current_dancer_back_encoder_mm"] = current_fabric_moved_under_camera_mm;
            real_world_values["current_dancer_front_encoder_mm"] = current_fabric_moved_under_cutter_mm;
            real_world_values["dancer_indicator_color"] = dancer_sensor.getIndicatorColor();
            // real_world_values["cutter_indicator_color"] = "#00FF00";

            // Dancer and Cutter Sensor, Back and Front Encoder
            real_world_values["dancer_sensor_indicator"] = errorTracker.getComponentIndicatorColor(ComponentModel::DANCER_SENSOR);
            real_world_values["cutter_sensor_indicator"] = errorTracker.getComponentIndicatorColor(ComponentModel::CUTTER_SENSOR);
            real_world_values["front_encoder_indicator"] = errorTracker.getComponentIndicatorColor(ComponentModel::FRONT_ENCODER);
            real_world_values["back_encoder_indicator"] = errorTracker.getComponentIndicatorColor(ComponentModel::BACK_ENCODER);

            // real_world_values["dancer_sync_indicator_color"] = "#00FF00";
            real_world_values["fabric_sync_indicator_color"] = "#00FF00";
            real_world_values["camera_encoder_indicator_color"] = "#00FF00";
            real_world_values["cutter_encoder_indicator_color"] = "#00FF00";
            real_world_values["current_body_mm"] = previous_body_processed_mm;
            real_world_values["current_cam_mm"] = current_fabric_moved_under_camera_mm;
            real_world_values["current_cutter_mm"] = current_fabric_moved_under_cutter_mm;

            // In case of semi auto and just cut, avoid overwriting stopping command to plc in case stopping already in progress
            if (is_immediate_stopping_requested && !check_for_stopping)
            {
                is_immediate_stopping_requested = false;

                // call service to stop
                if (!ignore_inspection_on)
                {
                    plcComm.requestToStopMM(10);
                }
            }

            if (cutting_mode == "semiauto" || cutting_mode == "justcut")
            {
                if (is_defect_available_for_next_stopping)
                {
                    auto defect_to_stop = defect_for_next_stopping.detection;
                    // distance from stopper for each defect's start position = cam2stopper - distance from camera
                    defect_distance_from_stopper_mm = abs(defect_to_stop.bbox.center.y - defect_to_stop.bbox.size_y);

                    Defect defect_log;
                    defect_log.defect_id = defect_for_next_stopping.defect_id;
                    defect_log.robro_roll_id = roll_id;
                    defect_log.stopping_command_issued = defect_distance_from_stopper_mm;
                    defect_log.body_id = body_log.body_id;

                    if (is_roll_started && is_cutter_falling_edge_on)
                    {
                        msg.data = defect_log.toString();
                        defect_log_pub.publish(msg);
                    }
                    if (!check_for_stopping)
                    {
                        // stop if there is a defect present in the current panel. Stop for the last defect in the current panel.
                        if (defect_distance_from_stopper_mm > 0 &&
                            defect_distance_from_stopper_mm <= cut_length_mm &&
                            stopping_defect_frame_id != defect_for_next_stopping.sequence_id)
                        {
                            // ignore inspection on remains
                            if (defect_distance_from_stopper_mm >= 0 && defect_distance_from_stopper_mm < dancer_end2stopper_distance_mm_with_offset)
                            {
                                if (!ignore_inspection_on && !top_bottom_panel_selected)
                                {
                                    // call service to stop
                                    plcComm.requestToStopMM(defect_distance_from_stopper_mm);
                                    last_stopping_command_given_at_mm = defect_distance_from_stopper_mm;

                                    check_for_stopping = true;
                                    is_stopping_done = false;
                                    stopping_defect_frame_id = defect_for_next_stopping.sequence_id;

                                    json stopping_info;
                                    stopping_info["next_stopping_distance"] = defect_distance_from_stopper_mm;
                                    stopping_info["is_punch_blocked"] = roll_info_punching_on && punching_block_state;
                                    msg.data = stopping_info.dump();
                                    next_stopping_pub.publish(msg);
                                    ROS_WARN("Last Distance Written: %ld", defect_distance_from_stopper_mm);
                                }
                            }
                        }
                    }
                }

                if (is_mode_top_bottom_and_defect_passed)
                {
                    buzzerControl.beep(2);
                }

                if (block_punching_for_defect_behind_punch && defect_present_within_cut_length)
                {
                    need_to_issue_block_signal = true;
                }
                else if (!block_punching_for_defect_behind_punch && defect_present_within_half_cut_length)
                {
                    need_to_issue_block_signal = true;
                }
                if (!ignore_inspection_on && !top_bottom_panel_selected && roll_info_punching_on && need_to_issue_block_signal)
                {
                    if (!punching_block_state)
                    {
                        punching_block_state = true;
                        is_prev_punch_blocked = true;
                        plcComm.setPunchBlockedTo(punching_block_state);
                    }
                }
                // if punching is blocked but now no more defects are there, unblock it.
                if (roll_info_punching_on && !need_to_issue_block_signal && punching_block_state)
                {
                    punching_block_state = false;
                    plcComm.setPunchBlockedTo(punching_block_state);
                    std::cout << "Punch Blocking Turned OFF outside for\n";
                }
                json stopping_info;
                if (check_for_stopping)
                {
                    uint64_t mm_since_defect = plcComm.getMMSinceDefect();
                    is_stopping_done = plcComm.isStoppingDone();
                    if (is_stopping_done)
                    {
                        if (stopping_defect_frame_id != -1)
                        {
                            recordStoppingAction(stopping_defect_frame_id);
                        }
                        total_processed_body_mm_after_last_stopping = 0;
                        is_stopping_window_open = true;
                        check_for_stopping = false;
                        stopping_info["next_stopping_distance"] = 0;

                        if (cutting_mode == "justcut")
                        {
                            // Changing Auto to Manual in Cutting Machine
                            msg.data = "Auto Mode: False";
                            next_stopping_pub.publish(msg);

                            plcComm.setAutoModeTo(true);

                            if (dancer_control_on_just_cut_mode)
                            {
                                plcComm.setInductionMotorTo(true);
                                // wait until dancer sensor is on
                                int max_number_of_tries = 500;
                                while (!plcComm.isConstantDistanceSensorOn() && max_number_of_tries-- > 0)
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                    msg.data = "Dancer: " + std::to_string(max_number_of_tries);
                                    next_stopping_pub.publish(msg);
                                }
                                if (max_number_of_tries <= 0)
                                {
                                    sendNotification("Dancer Sensor didn't turn on in 10 sec.", "warning");
                                }
                                plcComm.setInductionMotorTo(false);
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(500));

                            if (spout_and_cutter_control_on_just_cut_mode)
                            {
                                if (roll_info_punching_on)
                                {
                                    plcComm.setCCutModeTo(true);
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                plcComm.setCutBeforeStartTo(true);
                                msg.data = "Cut: True";
                                next_stopping_pub.publish(msg);
                                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                                plcComm.setCutBeforeStartTo(false);
                                plcComm.setCCutModeTo(false);
                            }

                            // Changing Manual to Auto Mode
                            plcComm.setAutoModeTo(false);
                            msg.data = "Auto: True";
                            next_stopping_pub.publish(msg);

                            // Sleeping for giving time to operator to remove the defect piece
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                            // Starting a Machine
                            msg.data = "Start: True";
                            next_stopping_pub.publish(msg);

                            plcComm.setCycleStartTo(true);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            plcComm.setCycleStartTo(false);

                            msg.data = "Auto: True";
                            next_stopping_pub.publish(msg);
                        }
                    }
                    else
                    {
                        stopping_info["next_stopping_distance"] = last_stopping_command_given_at_mm - mm_since_defect;
                    }
                }
                else
                {
                    if (is_stopping_done && is_stopping_window_open)
                    {
                        // close the stopping window
                        total_processed_body_mm_after_last_stopping += current_move_forward_mm;
                        if (total_processed_body_mm_after_last_stopping >= 500)
                        {
                            total_processed_body_mm_after_last_stopping = 0;
                            stopping_window_close_pub.publish(std_msgs::Empty());
                            is_stopping_window_open = false;
                        }
                    }
                    stopping_info["next_stopping_distance"] = 0;
                }

                stopping_info["is_punch_blocked"] = roll_info_punching_on && punching_block_state;
                if (auto_cut_debug_in_semi_auto_mode || cutting_mode == "justcut" || cutting_mode == "semiauto")
                {
                    stopping_info["cutting_info"] = cuttingManager.getCuttingCounterAndNextCutLength();
                    stopping_info["cutting_info"]["p_target"] = current_work_order.getTargetPcs();
                    stopping_info["cutting_info"]["p_meter"] = body_meter_tracker.primary_meter;
                    stopping_info["cutting_info"]["s_meter"] = body_meter_tracker.secondary_meter;
                    stopping_info["cutting_info"]["t_meter"] = body_meter_tracker.tertiary_meter;
                    // stopping_info["cutting_info"]["d_meter"] = body_meter_tracker.defective_meter;
                    stopping_info["cutting_info"]["d_meter"] = body_meter_tracker.defective_meter * fabric_gsm * (fabric_width * 0.01) * 0.001 * body_meter_tracker.defective_weight_factor; // Convert into KG
                }
                msg.data = stopping_info.dump();
                next_stopping_pub.publish(msg);
            }
            else
            {
                json stopping_info;
                if (defect_present_within_cut_length)
                {
                    need_to_issue_block_signal = true;
                }
                if (roll_info_punching_on && need_to_issue_block_signal && !top_bottom_panel_selected)
                {
                    if (!punching_block_state)
                    {
                        punching_block_state = true;
                        is_prev_punch_blocked = true;
                        plcComm.setPunchBlockedTo(punching_block_state);
                        // cuttingManager.sendSpoutToCuttingMachine(!punching_block_state);
                    }
                }

                // if (is_mode_top_bottom_and_defect_passed)
                // {
                //     buzzerControl.beep(2);
                // }

                // if punching is blocked but now no more defects are there, unblock it.
                if (roll_info_punching_on && !need_to_issue_block_signal && punching_block_state)
                {
                    punching_block_state = false;
                    // cuttingManager.sendSpoutToCuttingMachine(!punching_block_state);
                    plcComm.setPunchBlockedTo(punching_block_state);
                    std::cout << "Punch Blocking Turned OFF outside for\n";
                }
                stopping_info["cutting_info"] = cuttingManager.getCuttingCounterAndNextCutLength();
                stopping_info["cutting_info"]["p_target"] = current_work_order.getTargetPcs();
                stopping_info["cutting_info"]["p_meter"] = body_meter_tracker.primary_meter;
                stopping_info["cutting_info"]["s_meter"] = body_meter_tracker.secondary_meter;
                stopping_info["cutting_info"]["t_meter"] = body_meter_tracker.tertiary_meter;
                // stopping_info["cutting_info"]["d_meter"] = body_meter_tracker.defective_meter;

                stopping_info["cutting_info"]["d_meter"] = body_meter_tracker.defective_meter * fabric_gsm * (fabric_width * 0.01) * 0.001 * body_meter_tracker.defective_weight_factor; // Convert into KG

                stopping_info["is_punch_blocked"] = roll_info_punching_on && punching_block_state;
                msg.data = stopping_info.dump();
                next_stopping_pub.publish(msg);
            }

            if (is_cut_length_updated)
            {
                is_cut_length_updated = false;
                cut_length_mm = updated_cut_length_mm;
                plcComm.writeCutLength(cut_length_mm);
                resetPanelsPos();
            }

            auto defect_map = mapDrawer.getWorldMap(panel_positions,
                                                    cam_config_map,
                                                    new_master_defects,
                                                    auto_cut_next_pos,
                                                    cuttingManager.getColorForCuttingPlan());
            auto defect_map_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", defect_map)
                                      .toImageMsg();
            defect_map_img->header.frame_id = getWorldMapCamNames().dump();
            defect_map_image_pub.publish(defect_map_img);

            real_world_values["kwis_plc_indicator_color"] = plcComm.getLastCmdSuccess() ? "#187318" : "#ff0000";
            if (!plcComm.getLastCmdSuccess())
            {
                // one time popup and log
                if (!kwis_plc_connection_pop_up_once)
                {
                    sendNotification("Code: COM002 - KWIS PLC Read/Write Error", "popup");
                    buzzerControl.beep(2);

                    addSystemLog("ERROR", "KWIS PLC Connection Broken", "COM002");
                    kwis_plc_connection_pop_up_once = true;
                }
            }
            else if (kwis_plc_connection_pop_up_once)
            {
                kwis_plc_connection_pop_up_once = false;
            }

            json cutting_mc_info = cuttingManager.getPLCConnectionDetails();
            if (cutting_mc_info.contains("is_last_cmd_success"))
            {
                bool is_last_cmd_success = cutting_mc_info["is_last_cmd_success"];
                real_world_values["cutting_mc_plc_indicator_color"] = is_last_cmd_success ? "#187318" : "#ff0000";
                if (cutting_mode != "semiauto" && cutting_mode != "justcut" && !is_last_cmd_success)
                {
                    if (!cutting_mc_connection_pop_up_once)
                    {
                        sendNotification("Code: COM002 - CUTTING M/C PLC Read/Write Error", "popup");
                        buzzerControl.beep(2);

                        addSystemLog("ERROR", "CUTTING M/C PLC Connection Broken", "COM002");
                        cutting_mc_connection_pop_up_once = true;
                    }
                }
                else if (cutting_mc_connection_pop_up_once)
                {
                    cutting_mc_connection_pop_up_once = false;
                }
            }
            else
            {
                real_world_values["cutting_mc_plc_indicator_color"] = "#ff0000";
            }

            if (real_world_values_publish_interval.isReady())
            {
                msg.data = real_world_values.dump();
                system_config_pub.publish(msg);
                real_world_values_publish_interval.reset();
            }
        }
        else
        {
            // if (sst->changeState("ON_IDLE"))
            // {
            //     logger.add_state_log(sst->getCurrentStateId(),
            //                          component_name,
            //                          sst->getCurrentStateCode(),
            //                          sst->getStateStartTime(),
            //                          sst->getStateEndTime(),
            //                          sst->getStateDuration(),
            //                          "NA",
            //                          "System Is Ideal");
            //     addSystemLog("INFO", "System State: ON_IDLE", "INFO011");
            // }
            // else if (sst->checkLogTimeout())
            // {
            //     logger.update_state_log(sst->getCurrentStateId(),
            //                             sst->getStateEndTime(),
            //                             sst->getStateDuration());
            // }

            if (interval_task_roll_off_read_front_encoder.isReady())
            {
                bool fabric_under_cutter_success = plcComm.getFabricMovedUnderCutterMM(current_fabric_moved_under_cutter_mm);
                interval_task_roll_off_read_front_encoder.reset();

                if (fabric_under_cutter_success)
                {
                    // if we have moved 30M without turning the system on and we've not given the popup for the last 10m.
                    if (current_fabric_moved_under_cutter_mm >= 30000 && current_fabric_moved_under_cutter_mm - last_distance_system_not_engaged_popup > 10000)
                    {

                        // If cutting machine running change state to ON_IDLE_MACHINE_RUNNING
                        if (sst->changeState(ComponentState::RUNNING_))
                        {
                            last_inserted_system_id = sst->getLastStateId();
                            addSystemLog("INFO", "Component State: ON_IDLE_MACHINE_RUNNING", "INFO011");
                        }
                        else if (sst->checkLogTimeout())
                        {

                            sst->updateStateLog(last_inserted_system_id);
                        }

                        sendNotification("Code: INFO010 - Start KWIS System", "popup");
                        buzzerControl.beep(2);
                        last_distance_system_not_engaged_popup = current_fabric_moved_under_cutter_mm;
                        addSystemLog("INFO", "CUTTING M/C Running. System not ON.", "INFO010");
                    }
                    else
                    {
                        if (sst->changeState(ComponentState::CONNECTED_))
                        {
                            last_inserted_system_id = sst->getLastStateId();
                            addSystemLog("INFO", "Component State: Connected", "INFO011");
                        }
                        else if (sst->checkLogTimeout())
                        {

                            sst->updateStateLog(last_inserted_system_id);
                        }
                    }
                }
            }
        }

        if (is_auto_cut_modal_open && interval_task_publish_machine_ready_for_display.isReady())
        {
            std_msgs::Bool machine_ready_msg;
            machine_ready_msg.data = cuttingManager.isMachineAvailable();
            ai_cut_master_modal_machine_ready_pub.publish(machine_ready_msg);

            std_msgs::Bool connection_status_msg;
            json connection_status_json = cuttingManager.getPLCConnectionDetails();
            connection_status_msg.data = connection_status_json["is_last_cmd_success"];
            ai_cut_master_modal_connection_status_pub.publish(connection_status_msg);

            interval_task_publish_machine_ready_for_display.reset();
        }

        // light toggle
        if (is_light_toggle_requested)
        {
            is_light_toggle_requested = false;
            if (plcComm.setInspectionLightTo(is_light_off))
            {
                sendNotification(is_light_off ? "Light Off" : "Light On", "info");
                if (is_light_off)
                {
                    addSystemLog("ACTIVITY", "Inspection Light Turned Off", "INFO014");
                }
                else
                {
                    addSystemLog("ACTIVITY", "Inspection Light Turned On", "INFO015");
                }
            }
            else
            {
                sendNotification("Unable to perform light ops", "warning");
            }
        }

        buzzerControl.update();
        if (buzzerControl.isSignalUpdated())
        {
            plcComm.setBuzzerTo(buzzerControl.getSignal());
        }
        // update logger state
        logger.update();
        timer.stop();
        rate.sleep();
    }
}

/**
 * @brief Get the Time in string format of YYYY-MM-DD HH:MM:SS
 *
 * @return std::string - Time in string format of YYYY-MM-DD HH:MM:SS
 */
std::string RealWorldMapNode::getTimeStr()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    s.resize(std::strlen(s.c_str()));
    return s;
}

/**
 * @brief gives the time difference in milliseconds between the given time and current time.
 *
 * @param point - time point to compare with current time.
 * @return int - time difference in milliseconds.
 */
int RealWorldMapNode::timeDiffFromCurrentTime(std::chrono::high_resolution_clock::time_point point)
{
    return (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - point)
                .count());
}

/**
 * @brief Returns the distance between given defect and the fabric currently under camera
 * in mm.
 * @param current_frame_seq - current frame sequence number.
 * @return double - distance between given defect and the fabric currently under camera
 */
double RealWorldMapNode::getDistanceFromCameraMM(int current_frame_seq)
{
    weaving_inspection::GetPulseDistance srv;

    srv.request.req_frame_id = current_frame_seq;

    if (cam_pulse_client.call(srv))
    {
        // we got the total pulses from the camera. Convert them into mm
        return (srv.response.pulse_difference) / pulse_per_mm;
    }
    else
    {
        if (interval_task_pulse_error_logger.isReady())
        {
            addSystemLog("ERROR", "[RealWorld] Total Pulse Request Not Processed: " + std::to_string(current_frame_seq), "APP019");
            interval_task_pulse_error_logger.reset();
        }

        return 0;
    }
}

/**
 * @brief Gets the difference of pulses in Primary Camera Counter since last time
 * loop was closed (dancer sensor was turned on).
 * If values are negative (due to small backlash) - just send 0.
 *
 * @return int64_t 0 if backlash, else actual value.
 */
bool RealWorldMapNode::getCameraPulsesFromLastLoopClose(uint64_t &value)
{
    weaving_inspection::GetCurrentPulses srv;

    cam_last_loop_close_pulse_client.waitForExistence();

    if (cam_last_loop_close_pulse_client.call(srv))
    {
        // negative value
        if (srv.response.pulses < 0)
        {
            return false;
        }
        value = static_cast<uint64_t>(srv.response.pulses);
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Add the system log to into system log server.
 *
 * @param severity - Severity of the log.
 * @param msg - Message to be logged.
 */
void RealWorldMapNode::addSystemLog(std::string severity, std::string msg, std::string msg_code)
{
    // System log needs a json with severity and msg.
    msg = "[RealWorld Node] " + msg;
    logger.add_log(getLogSeverityFromString(severity), getLogCodeFromString(msg_code), msg, LogComponent::APP);
}

/**
 * @brief  Metadata change callback
 *
 * @param msg - metadata change status
 */
void RealWorldMapNode::metadataChangeCallback(const std_msgs::String &msg)
{
    json metadata = json::parse(msg.data);
    if (metadata.contains("type") && metadata["type"] == "roll_metadata")
    {
        if (metadata.contains("offset") &&
            metadata.contains("is_roll_started") &&
            metadata.contains("cut_length") &&
            metadata.contains("punching_status") &&
            metadata.contains("roll_id") &&
            metadata.contains("customer_roll_id") &&
            metadata.contains("job_id") &&
            metadata.contains("top_bottom_panel_selected") &&
            metadata.contains("ignore_inspection") &&
            metadata.contains("recipe_name") &&
            metadata.contains("cutting_mode") &&
            metadata["is_roll_started"])
        {
            offset_cm = metadata["offset"];
            current_cam2stopper_distance_mm = original_cam2stopper_mm = (const_original_cam2stopper_mm + (offset_cm * 10));
            dancer_end2stopper_distance_mm_with_offset = dancer_end2stopper_distance_mm + (offset_cm * 10);
            mapDrawer.updateMapDistances(original_cam2stopper_mm, cut_length_mm, dancer_end2stopper_distance_mm_with_offset);
            // need to update plc
            updated_cut_length_mm = metadata["cut_length"];

            roll_id = metadata["roll_id"];
            job_id = metadata["job_id"];
            if (metadata.contains("work_order_id") && !metadata["work_order_id"].is_null())
            {
                work_order_id = std::stol(metadata["work_order_id"].get<std::string>());
            }
            ignore_inspection_on = metadata["ignore_inspection"];
            updated_cut_length_mm *= 10;
            roll_info_punching_on = metadata["punching_status"];
            last_defect_body_id = metadata["total_defects"];

            if (spout_control_via_cutting_machine_com)
            {
                cuttingManager.sendSpoutToCuttingMachine(roll_info_punching_on);
                std::cout << "Setting Spout Mode in Cutting Machine: " << roll_info_punching_on << std::endl;
            }

            top_bottom_panel_selected = metadata["top_bottom_panel_selected"];

            // body_log.id = body_log.genNewID();
            body_log.job_id = job_id;
            body_log.robro_roll_id = roll_id;
            previous_body_processed_before_reset = 0;

            cutting_mode = metadata["cutting_mode"];

            if (metadata.contains("work_order_id"))
            {
                current_work_order.loadJSON(logger.get_work_order_by_id(std::stoi(metadata["work_order_id"].get<std::string>())));
            }

            if (metadata.contains("batch_count"))
            {
                current_batch_count.setTargetBatchCount(metadata["batch_count"]);
            }

            if (metadata.contains("layer_value"))
            {
                body_meter_tracker.setDefectiveWeightFactor(metadata["layer_value"]);
            }

            std::vector<int> cut_lengths{updated_cut_length_mm};
            if (metadata.contains("secondary_cut_lengths"))
            {
                auto lengths = metadata["secondary_cut_lengths"].get<std::vector<int>>();
                for (auto &cut_length : lengths)
                {
                    cut_lengths.push_back(cut_length * 10);
                }
            }
            cuttingManager.setCutLengths(cut_lengths);
            body_meter_tracker.reset();
            cuttingManager.cycle_event_tracker.resetCycles();

            std::map<int, std::string> cam2stopper_distance_values = {
                {1000, "#ff0000"},
                {current_cam2stopper_distance_mm, "#187318"},
                {current_cam2stopper_distance_mm * 1.5, "#ffff00"},
                {INT_MAX, "#ff0000"},
            };
            std::map<int, std::string> current_body_processed_values = {
                {-1, "#ff0000"},
                {updated_cut_length_mm, "#187318"},
                {updated_cut_length_mm * 2, "#ffff00"},
                {INT_MAX, "#ff0000"},
            };

            is_cut_length_updated = true;
            is_roll_started = true;
            bypass_cutter_sensor_once = true;

            this_dataset_id = logger.create_dataset_id(metadata["customer_roll_id"].get<std::string>(), metadata["recipe_name"].get<std::string>(), metadata["id"].get<std::string>());
            dataset_id = metadata["roll_id"];
            boost::filesystem::path base_path(report_image_save_path);

            // Creating a folder in app_id
            boost::filesystem::path project_path = base_path / project_id / std::to_string(dataset_id);
            boost::filesystem::create_directories(project_path);

            // Updating a base path to dataset_path
            base_path = project_path / this_dataset_id;

            std::string current_cropped_defect_images_save_path = boost::filesystem::path(base_path / "cropped_defect_images/").string();
            boost::filesystem::create_directories(current_cropped_defect_images_save_path);
            report_image_saver->base_path = current_cropped_defect_images_save_path;
            std::cout << "For Cropped images - created path and directory: " << current_cropped_defect_images_save_path << std::endl;
        }
        else
        {
            is_roll_started = false;
            bypass_cutter_sensor_once = false;
            if (metadata.contains("reset_batch_count"))
            {
                current_batch_count.reset();
            }
        }

        if (metadata.contains("fabric_width") && metadata.contains("gsm"))
        {
            fabric_gsm = metadata["gsm"];
            fabric_width = metadata["fabric_width"];
        }

        is_reset_requested = true;
    }
    if (metadata.contains("type") && metadata["type"] == "real_world_metadata")
    {
        if (metadata.contains("stop_at_last_defect"))
        {
            get_last_defect_in_panel = metadata["stop_at_last_defect"].get<bool>();
            if (get_last_defect_in_panel)
            {
                sendNotification("Stop at last defect ON\nStop at every defect OFF", "info");
                addSystemLog("ACTIVITY", "Stop at last defect On", "NA");
            }
            else
            {
                sendNotification("Stop at last defect OFF\nStop at every defect ON", "info");
                addSystemLog("ACTIVITY", "Stop at last defect Off", "INFO009");
            }
        }
        if (metadata.contains("ignore_inspection_on"))
        {
            ignore_inspection_on = metadata["ignore_inspection_on"].get<bool>();
            sendNotification(ignore_inspection_on ? "Ignore Inspection ON" : "Ignore Inspection OFF", "info");
            if (ignore_inspection_on)
                addSystemLog("INFO", "Ignore Inspection On", "INFO009");
        }
        if (metadata.contains("pixel_per_mm"))
        {
            double px_per_mm_msg = metadata["pixel_per_mm"];
            if (px_per_mm_msg < 0.0 || px_per_mm_msg > 100.0)
            {
                return;
            }

            float px_per_mm = px_per_mm_msg;
            // Update the defects
            for (auto &defect : new_master_defects)
            {
                defect.detection.bbox.center.y = (defect.detection.bbox.center.y * y_px_per_mm) / px_per_mm;
                defect.detection.bbox.size_y = (defect.detection.bbox.size_y * y_px_per_mm) / px_per_mm;
            }

            y_px_per_mm = px_per_mm;
        }
    }
    if (metadata.contains("type") && metadata["type"] == "plc_metadata")
    {
        if (metadata.contains("is_light_off"))
        {
            is_light_off = metadata["is_light_off"].get<bool>();
            is_light_toggle_requested = true;
        }
        if (metadata.contains("is_immediate_stopping_requested"))
        {
            is_immediate_stopping_requested = true;
        }
    }
    if (metadata.contains("total_bodies"))
    {
        last_inserted_body_id = metadata["total_bodies"];
    }
    if (metadata.contains("primary_body_count") && metadata.contains("defective_body_count"))
    {
        cuttingManager.resetBodyCounters(metadata["primary_body_count"], metadata["secondary_body_count"], metadata["tertiary_body_count"], metadata["defective_body_count"]);
        body_meter_tracker.primary_meter = metadata["primary_body_count"].get<float>() * (cuttingManager.getCutLengthFromIndex(0) / 1000.0);
        body_meter_tracker.secondary_meter = metadata["secondary_body_count"].get<float>() * (cuttingManager.getCutLengthFromIndex(1) / 1000.0);
        body_meter_tracker.tertiary_meter = metadata["tertiary_body_count"].get<float>() * (cuttingManager.getCutLengthFromIndex(2) / 1000.0);

        body_meter_tracker.defective_meter = metadata["total_defective_length"].get<float>() / 1000.0f;
    }
}

/// @brief Cycle Start
/// @param msg
void RealWorldMapNode::cycleStartCallback(const std_msgs::Empty &msg)
{
    if (setCycleStart(true))
    {
        sendNotification("Cycle Started", "success");
    }
    else
    {
        sendNotification("Error in Cycle Start", "alert");
    }
}

/// @brief Cut Length Write
/// @param msg
void RealWorldMapNode::writeCutLengthCallback(const std_msgs::Int16 &msg)
{
    if (cuttingManager.writeCutLength(static_cast<uint16_t>(msg.data)))
    {
        sendNotification("Cut Length Written", "success");
    }
    else
    {
        sendNotification("Error in Write Cut Len", "alert");
    }
}

/**
 * @brief Callback for the defects from the master node
 *
 * @param msg - defects from the master node
 */
void RealWorldMapNode::combineDefectsCallback(const vision_msgs::Detection2DArray &msg)
{
    combine_defects.push(msg);
}

/**
 * @brief Callback to reset the world map from master node
 *
 * @param msg - ping message
 */
void RealWorldMapNode::resetWorldMapCallback(const std_msgs::Empty &msg)
{
    is_reset_requested = true;
}

/**
 * @brief Callback to update the current recipe from master node.
 *
 * @param msg - current recipe
 */
void RealWorldMapNode::updateCurrentRecipeCallback(const std_msgs::String &msg)
{
    json ack = json::parse(msg.data);
    if (ack.contains("recipe"))
    {
        ack = ack["recipe"];
        disabled_class_ids.clear();
        if (ack["nn"].contains("disabled_class_ids"))
        {
            for (auto &c_id : ack["nn"]["disabled_class_ids"])
            {
                disabled_class_ids.push_back(c_id);
            }
        }
        recipe_json.loadJson(ack);
    }
}

/// @brief get the msg from the front that the test modal is open
/// @param msg
void RealWorldMapNode::aiCutMasterModalOpenCallback(const std_msgs::Bool &msg)
{
    is_auto_cut_modal_open = msg.data;
    if (is_auto_cut_modal_open)
    {
        json msg_json;
        msg_json["connection_details"] = cuttingManager.getSystemPLCConfig().dump();

        json connection_status_json = cuttingManager.getPLCConnectionDetails();
        msg_json["connection_status"] = connection_status_json["is_last_cmd_success"] ? "#187318" : "#ff0000";

        std_msgs::String msg;
        msg.data = msg_json.dump();
        ai_cut_master_data_pub.publish(msg);
    }
}

/**
 * @brief callback to update the state of logger
 *
 * @param msg
 */
void RealWorldMapNode::loggerDisconnectedCallback(const std_msgs::Empty &msg)
{
    logger.setState(MySQLClientState::DISCONNECTED_RETRIES_EXCEEDED);
}

void RealWorldMapNode::runningMeterCallback(const std_msgs::String &msg)
{
    running_meter = std::stof(msg.data);
}

bool RealWorldMapNode::setCycleStart(bool state)
{
    bool success = false;
    if (is_cycle_start_electrical)
    {
        success &= plcComm.setCycleStartTo(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        success &= plcComm.setCycleStartTo(false);
    }
    else
    {
        success = cuttingManager.writeCycleStart();
    }
    return success;
}
/**
 * @brief Callback to record the camera position from master node.
 *
 * @param msg - camera position
 */
void RealWorldMapNode::recordCameraPositionCallback(const std_msgs::String &msg)
{
    json cam_config = json::parse(msg.data);
    std::string cam_id = cam_config["id"];
    group_id = cam_config["group_id"];
    if (cam_config.contains("camera_connected") && cam_config["camera_connected"])
    {
        ready_camera_ids.insert(cam_id);
    }

    // Check if the camera config contains all the required parameters
    if (!(cam_config.contains("id") &&
          cam_config.contains("fov_x_start_mm") &&
          cam_config.contains("fov_x_end_mm") &&
          cam_config.contains("is_top_camera") &&
          cam_config.contains("x_px_per_mm") &&
          cam_config.contains("position")))
    {
        std::cout << "\033[1;31m ERROR : \033[0m[RealWorldMapNode] Camera config does not contain all the required parameters";
        return;
    }

    // add the camera config to the map
    cam_config_map[cam_config["id"]] = cam_config;

    // update the total distance
    total_distance_x_mm = 0;
    int counter = 0;

    // update the total distance
    for (auto config : cam_config_map)
    {
        int fov_x_start_mm = config.second["fov_x_start_mm"];
        int fov_x_end_mm = config.second["fov_x_end_mm"];

        // update the total distance
        total_distance_x_mm += (fov_x_end_mm - fov_x_start_mm);
    }

    // update the map constant
    mapDrawer.updateMapSize(total_distance_x_mm, current_cam2stopper_distance_mm);
}

/**
 * @brief Main Function which creates the objects
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RealWorldMapNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    RealWorldMapNode real_world_map_node;
    spinner.stop();
    return 0;
}
