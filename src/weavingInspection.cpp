/**
 * @file weavingInspection.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Weaving Inspection definations for KWIS
 * @version 1.0
 * @date 2022-03-03
 * @updated 2022-08-26
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <weavingInspection.hpp>
#include <intervalTask.hpp>
#include "weaving_inspection/InsertDataLog.h"
#include "weaving_inspection/GetRollLog.h"

/**
 * @brief Construct a new Weaving Inspection:: Weaving Inspection object
 *
 */
WeavingInspection::WeavingInspection() : interval_task_main_q_size_logger(5 * 1000)
{
    ROS_INFO("Loading basic params...");

    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[Inspection Node]Couldn't read all params from launch file. Please check names and types!";
    }

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    if (!readROSParamsAndStoreInDB())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[Inspection Node]Couldn't read all params from launch file and store in DB. Please check names and types!";
    }

    initPubSub();

    initServices();

    initSystem();
}

/**
 * @brief Destroy the Weaving Inspection:: Weaving Inspection object
 *
 */
WeavingInspection::~WeavingInspection()
{
    delete full_image_saver;
}
void WeavingInspection::initServices()
{
    ros::NodeHandle n;
    client = n.serviceClient<weaving_inspection::InsertDataLog>("/report/insert/log");
    get_roll_data_client = n.serviceClient<weaving_inspection::GetRollLog>("/report/get/roll_log");
}

/**
 * @brief It reads the parameters from the ros parameter server and
 * stores them in the class variables.
 *
 * @return true - if parameters are read successfully.
 * @return false - otherwise.
 */
bool WeavingInspection::readROSParams()
{

    return (
        ros::param::get("frame_width", frame_width) &&
        ros::param::get("frame_height", frame_height) &&

        ros::param::get("just_cut_connections_available", just_cut_connections_available) &&

        ros::param::get("pixel_per_mm", pixel_per_mm) &&
        ros::param::get("pulse_per_mm", pulse_per_mm) &&

        ros::param::get("number_of_cameras", number_of_cameras) &&
        ros::param::get("project_id", project_id) &&
        ros::param::get("project_base_path", project_base_path) &&

        ros::param::get("~cam2stopper_distance", original_cam2Stopper_mm));
}

bool WeavingInspection::readROSParamsAndStoreInDB()
{
    // Local variables for the rest
    double save_defect_images_lower_threshold;
    int resize_image_before_prediction_height;
    int resize_image_before_prediction_width;
    bool simulation_mode_on;
    bool simulation_mode_with_camera_connected;
    bool current_body_processed_auto_reset_mode;
    bool debug_mode;
    int num_past_images_to_store;
    int master_camera_id;

    double max_roll_length_m;
    bool enable_trigger_db;
    int db_max_reconnection_attempts;
    bool is_cutter_conn_nc;
    std::string buzzer_at_defect;
    bool half_cut_mode;
    int stop_time_after_defect;
    int crop_margin;
    std::string license_host_url;
    std::string license_product_id;

    std::string report_save_path;
    std::string db_host;
    std::string db_user;
    std::string db_password;
    std::string db_name;
    int db_port;

    double ideal_brightness, brightness_tolerance;
    double min_exposure_value, max_exposure_value;

    std::string reporting_base_path;

    std::string browser_name;
    std::string gui_folder_path;
    std::string ip_address_of_server;
    int port_number;
    std::string command_for_pwa;
    std::string package_name;

    std::string plc_port;
    int plc_baudrate;
    std::string plc_parity;
    int plc_bytesize;
    int plc_stopbit;
    int plc_slave_address;
    double plc_pulse_per_mm;
    int map_width, map_height;
    int realworld_cam2Stopper_mm;
    bool block_punching_for_defect_behind_punch;
    bool spout_control_via_cutting_machine_com;
    bool use_cutter_sensor_instead_machine_ready;
    bool system_installed_in_front_of_dancer;
    double error_popup_threshold_m, error_stop_threshold_m;
    std::string report_image_save_path;
    std::string reporting_folder_name;
    int dancer_end2stopper_distance_mm;
    std::string cutting_mc_port;
    int cutting_mc_baud;
    std::string cutting_mc_parity;
    int cutting_mc_bytesize;
    int cutting_mc_stop_bit;
    int cutting_mc_slave_address;
    int auto_cut_defect_offset_mm;
    bool dancer_control_on_just_cut_mode;
    bool spout_and_cutter_control_on_just_cut_mode;
    bool is_cycle_start_register;
    bool is_cycle_start_electrical;
    bool is_machine_ready_register;
    bool is_spout_status_register;

    std::string cut_length_register, cycle_start, spout_mode, machine_waiting;

    // Read into locals
    ros::param::get("/save_defect_images_lower_threshold", save_defect_images_lower_threshold);
    ros::param::get("/resize_image_before_prediction_height", resize_image_before_prediction_height);
    ros::param::get("/resize_image_before_prediction_width", resize_image_before_prediction_width);
    ros::param::get("/simulation_mode_on", simulation_mode_on);
    ros::param::get("/simulation_mode_with_camera_connected", simulation_mode_with_camera_connected);
    ros::param::get("/current_body_processed_auto_reset_mode", current_body_processed_auto_reset_mode);
    ros::param::get("/debug_mode", debug_mode);
    ros::param::get("/num_past_images_to_store", num_past_images_to_store);
    ros::param::get("/master_camera_id", master_camera_id);

    ros::param::get("/max_roll_length_m", max_roll_length_m);
    ros::param::get("/enable_trigger_db", enable_trigger_db);
    ros::param::get("/db_max_reconnection_attempts", db_max_reconnection_attempts);
    ros::param::get("/is_cutter_conn_nc", is_cutter_conn_nc);
    ros::param::get("/buzzer_at_defect", buzzer_at_defect);
    ros::param::get("/half_cut_mode", half_cut_mode);
    ros::param::get("/stop_time_after_defect", stop_time_after_defect);
    ros::param::get("/crop_margin", crop_margin);
    ros::param::get("/license_host_url", license_host_url);
    ros::param::get("/license_product_id", license_product_id);

    ros::param::get("/report_save_path", report_save_path);
    ros::param::get("/db_host", db_host);
    ros::param::get("/db_user", db_user);
    ros::param::get("/db_password", db_password);
    ros::param::get("/db_name", db_name);
    ros::param::get("/db_port", db_port);

    ros::param::get("/brightness_adjuster_node/ideal_brightness", ideal_brightness);
    ros::param::get("/brightness_adjuster_node/brightness_tolerance", brightness_tolerance);
    ros::param::get("/brightness_adjuster_node/min_exposure_value", min_exposure_value);
    ros::param::get("/brightness_adjuster_node/max_exposure_value", max_exposure_value);

    ros::param::get("/reporting_base_path", reporting_base_path);

    ros::param::get("/pythonServer/browser_name", browser_name);
    ros::param::get("/pythonServer/gui_folder_path", gui_folder_path);
    ros::param::get("/pythonServer/ip_address_of_server", ip_address_of_server);
    ros::param::get("/pythonServer/port_number", port_number);
    ros::param::get("/pythonServer/command_for_pwa", command_for_pwa);
    ros::param::get("/pythonServer/package_name", package_name);

    ros::param::get("/real_world_map/port", plc_port);
    ros::param::get("/real_world_map/baudrate", plc_baudrate);
    ros::param::get("/real_world_map/parity", plc_parity);
    ros::param::get("/real_world_map/bytesize", plc_bytesize);
    ros::param::get("/real_world_map/stopbit", plc_stopbit);
    ros::param::get("/real_world_map/slave_address", plc_slave_address);
    ros::param::get("/real_world_map/plc_pulse_per_mm", plc_pulse_per_mm);
    ros::param::get("/real_world_map/map_width", map_width);
    ros::param::get("/real_world_map/map_height", map_height);
    ros::param::get("/real_world_map/cam2stopper_distance", realworld_cam2Stopper_mm);
    ros::param::get("/real_world_map/block_punching_for_defect_behind_punch", block_punching_for_defect_behind_punch);
    ros::param::get("/real_world_map/spout_control_via_cutting_machine_com", spout_control_via_cutting_machine_com);
    ros::param::get("/real_world_map/use_cutter_sensor_instead_machine_ready", use_cutter_sensor_instead_machine_ready);
    ros::param::get("/real_world_map/system_installed_in_front_of_dancer", system_installed_in_front_of_dancer);
    ros::param::get("/real_world_map/error_popup_threshold_m", error_popup_threshold_m);
    ros::param::get("/real_world_map/error_stop_threshold_m", error_stop_threshold_m);
    ros::param::get("/report_image_save_path", report_image_save_path);
    ros::param::get("/reporting_folder_name", reporting_folder_name);
    ros::param::get("/real_world_map/dancer_end2stopper_distance_mm", dancer_end2stopper_distance_mm);
    ros::param::get("/real_world_map/cutting_mc_port", cutting_mc_port);
    ros::param::get("/real_world_map/cutting_mc_baud", cutting_mc_baud);
    ros::param::get("/real_world_map/cutting_mc_parity", cutting_mc_parity);
    ros::param::get("/real_world_map/cutting_mc_bytesize", cutting_mc_bytesize);
    ros::param::get("/real_world_map/cutting_mc_stop_bit", cutting_mc_stop_bit);
    ros::param::get("/real_world_map/cutting_mc_slave_address", cutting_mc_slave_address);
    ros::param::get("/real_world_map/auto_cut_defect_offset_mm", auto_cut_defect_offset_mm);
    ros::param::get("/real_world_map/dancer_control_on_just_cut_mode", dancer_control_on_just_cut_mode);
    ros::param::get("/real_world_map/spout_and_cutter_control_on_just_cut_mode", spout_and_cutter_control_on_just_cut_mode);
    ros::param::get("/real_world_map/is_cycle_start_register", is_cycle_start_register);
    ros::param::get("/real_world_map/is_cycle_start_electrical", is_cycle_start_electrical);
    ros::param::get("/real_world_map/is_machine_ready_register", is_machine_ready_register);
    ros::param::get("/real_world_map/is_spout_status_register", is_spout_status_register);

    ros::param::get("/cutting_mc_addresses/cut_length_register", cut_length_register);
    ros::param::get("/cutting_mc_addresses/cycle_start", cycle_start);
    ros::param::get("/cutting_mc_addresses/spout_mode", spout_mode);
    ros::param::get("/cutting_mc_addresses/machine_waiting", machine_waiting);

    json config_json;
    // Build JSON
    config_json["frame_width"] = frame_width;
    config_json["frame_height"] = frame_height;
    config_json["just_cut_connections_available"] = just_cut_connections_available;
    {
        pixel_per_mm = std::round(pixel_per_mm * 100.0) / 100.0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << pixel_per_mm;
        std::string formatted_pixel_per_mm = oss.str();
        config_json["pixel_per_mm"] = formatted_pixel_per_mm;
    }
    {
        pulse_per_mm = std::round(pulse_per_mm * 100.0) / 100.0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << pulse_per_mm;
        std::string formatted_pulse_per_mm = oss.str();
        config_json["pulse_per_mm"] = formatted_pulse_per_mm;
    }

    config_json["number_of_cameras"] = number_of_cameras;
    config_json["project_id"] = project_id;
    config_json["project_base_path"] = project_base_path;
    config_json["original_cam2Stopper_mm"] = original_cam2Stopper_mm;
    config_json["save_defect_images_lower_threshold"] = save_defect_images_lower_threshold;
    config_json["resize_image_before_prediction_height"] = resize_image_before_prediction_height;
    config_json["resize_image_before_prediction_width"] = resize_image_before_prediction_width;
    config_json["simulation_mode_on"] = simulation_mode_on;
    config_json["simulation_mode_with_camera_connected"] = simulation_mode_with_camera_connected;
    config_json["current_body_processed_auto_reset_mode"] = current_body_processed_auto_reset_mode;
    config_json["debug_mode"] = debug_mode;
    config_json["num_past_images_to_store"] = num_past_images_to_store;
    config_json["master_camera_id"] = master_camera_id;

    config_json["max_roll_length_m"] = max_roll_length_m;
    config_json["enable_trigger_db"] = enable_trigger_db;
    config_json["db_max_reconnection_attempts"] = db_max_reconnection_attempts;
    config_json["is_cutter_conn_nc"] = is_cutter_conn_nc;
    config_json["buzzer_at_defect"] = buzzer_at_defect;
    config_json["half_cut_mode"] = half_cut_mode;
    config_json["stop_time_after_defect"] = stop_time_after_defect;
    config_json["crop_margin"] = crop_margin;
    config_json["license_host_url"] = license_host_url;
    config_json["license_product_id"] = license_product_id;

    config_json["report_save_path"] = report_save_path;
    config_json["db_host"] = db_host;
    config_json["db_user"] = db_user;
    config_json["db_password"] = db_password;
    config_json["db_name"] = db_name;
    config_json["db_port"] = db_port;

    config_json["ideal_brightness"] = ideal_brightness;
    config_json["brightness_tolerance"] = brightness_tolerance;
    config_json["min_exposure_value"] = min_exposure_value;
    config_json["max_exposure_value"] = max_exposure_value;

    config_json["reporting_base_path"] = reporting_base_path;

    config_json["browser_name"] = browser_name;
    config_json["gui_folder_path"] = gui_folder_path;
    config_json["ip_address_of_server"] = ip_address_of_server;
    config_json["port_number"] = port_number;
    config_json["command_for_pwa"] = command_for_pwa;
    config_json["package_name"] = package_name;

    config_json["plc_port"] = plc_port;
    config_json["plc_baudrate"] = plc_baudrate;
    config_json["plc_parity"] = plc_parity;
    config_json["plc_bytesize"] = plc_bytesize;
    config_json["plc_stopbit"] = plc_stopbit;
    config_json["plc_slave_address"] = plc_slave_address;
    config_json["plc_pulse_per_mm"] = plc_pulse_per_mm;
    config_json["map_width"] = map_width;
    config_json["map_height"] = map_height;
    config_json["realworld_cam2Stopper_mm"] = realworld_cam2Stopper_mm;
    config_json["block_punching_for_defect_behind_punch"] = block_punching_for_defect_behind_punch;
    config_json["spout_control_via_cutting_machine_com"] = spout_control_via_cutting_machine_com;
    config_json["use_cutter_sensor_instead_machine_ready"] = use_cutter_sensor_instead_machine_ready;
    config_json["system_installed_in_front_of_dancer"] = system_installed_in_front_of_dancer;
    config_json["error_popup_threshold_m"] = error_popup_threshold_m;
    config_json["error_stop_threshold_m"] = error_stop_threshold_m;
    config_json["report_image_save_path"] = report_image_save_path;
    config_json["reporting_folder_name"] = reporting_folder_name;
    config_json["dancer_end2stopper_distance_mm"] = dancer_end2stopper_distance_mm;
    config_json["cutting_mc_port"] = cutting_mc_port;
    config_json["cutting_mc_baud"] = cutting_mc_baud;
    config_json["cutting_mc_parity"] = cutting_mc_parity;
    config_json["cutting_mc_bytesize"] = cutting_mc_bytesize;
    config_json["cutting_mc_stop_bit"] = cutting_mc_stop_bit;
    config_json["cutting_mc_slave_address"] = cutting_mc_slave_address;
    config_json["auto_cut_defect_offset_mm"] = auto_cut_defect_offset_mm;
    config_json["dancer_control_on_just_cut_mode"] = dancer_control_on_just_cut_mode;
    config_json["spout_and_cutter_control_on_just_cut_mode"] = spout_and_cutter_control_on_just_cut_mode;
    config_json["is_cycle_start_register"] = is_cycle_start_register;
    config_json["is_cycle_start_electrical"] = is_cycle_start_electrical;
    config_json["is_machine_ready_register"] = is_machine_ready_register;
    config_json["is_spout_status_register"] = is_spout_status_register;

    config_json["cutting_mc_addresses"] = {
        {"cut_length_register", cut_length_register},
        {"cycle_start", cycle_start},
        {"spout_mode", spout_mode},
        {"machine_waiting", machine_waiting}};

    if (logger.add_system_configuration("AppConfig", getAppID(), LogComponent::APP, config_json, project_id))
        return true;
    else
        return false;
}

/**
 * @brief Function to initialize the ros publishers and sub.
 */
void WeavingInspection::initPubSub()
{

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ROS_INFO("Setting Publishers done...");

    /**
     * @brief Publisher to GUI and all other nodes.
     */
    notification_pub = n.advertise<std_msgs::String>("/gui/label/notification", 1);
    system_health_msg_pub = n.advertise<std_msgs::String>("/gui/label/main_node_health_data", 1);
    inspection_category_pub = n.advertise<std_msgs::String>("/gui/label/inspection_category", 1);
    running_meter_pub = n.advertise<std_msgs::String>("/gui/label/running_meter", 1);
    ng_count_pub = n.advertise<std_msgs::String>("/gui/label/no_of_defects", 1);
    system_config_pub = n.advertise<std_msgs::String>("/gui/value/all_values", 1);
    camera_config_sync_pub = n.advertise<std_msgs::Empty>("/cam/config_sync", 10);
    roll_log_pub = n.advertise<std_msgs::String>("/report/roll_log", 10);
    job_log_pub = n.advertise<std_msgs::String>("/report/job_log", 10);
    roll_continue_ack_gui_pub = n.advertise<std_msgs::String>("/gui/ack/roll_continue", 1);
    job_set_ack_pub = n.advertise<std_msgs::String>("/gui/ack/job", 10);
    combine_defects_pub = n.advertise<vision_msgs::Detection2DArray>("/world_map/combine_defects", 1000);
    reset_system_pub = n.advertise<std_msgs::Empty>("/main/roll/reset", 10);
    metadata_pub = n.advertise<std_msgs::String>("/main/roll/metadata", 10);
    cam_metadata_pub = n.advertise<std_msgs::String>("/main/roll/camera_metadata", 10);

    version_pub = n.advertise<std_msgs::Empty>("/gui/button/version", 1);
    /**
     * @brief Image Publishers to GUI and all other nodes.
     *
     */
    inspected_image_pub = it.advertise("/gui/image", 1);
    defected_image_pub = it.advertise("/gui/defected_image", 1);

    /**
     * @brief Subscriber to GUI and all other nodes.

     *
     */
    stop_at_last_defect_toggle_sub = n.subscribe("/gui/checkbox/stop_at_last_defect", 1, &WeavingInspection::stopAtLastDefectToggleCallback, this);
    cam_annotated_image_sub = n.subscribe("/cam/annotated_image", 100, &WeavingInspection::camAnnotatedImageCallback, this);
    defect_sub = n.subscribe("/cam/defect_result", 100, &WeavingInspection::defectDetailsCallback, this);
    material_type_sub = n.subscribe("/gui/value/material_type", 1, &WeavingInspection::materialTypeCallback, this);
    metadata_change_sub = n.subscribe("/gui/onchange/metadata", 1, &WeavingInspection::metadataChangeCallback, this);
    pixel_per_mm_sub = n.subscribe("/gui/value/pixel_per_mm", 1, &WeavingInspection::pixelPerMMCallback, this);
    pulse_per_mm_sub = n.subscribe("/gui/value/pulse_per_mm", 1, &WeavingInspection::pulsePerMMCallback, this);
    reset_system_sub = n.subscribe("/gui/button/reset", 1, &WeavingInspection::resetSystemCallback, this);
    restart_sub = n.subscribe("/gui/button/restart", 1, &WeavingInspection::restartCallback, this);
    system_config_sync_sub = n.subscribe("/gui/sync/configuration", 1, &WeavingInspection::systemConfigSyncCallback, this);
    camera_config_sub = n.subscribe("/cam/config", 100, &WeavingInspection::cameraConfigCallback, this);
    job_set_ack_sub = n.subscribe("/cam/ack/job", 100, &WeavingInspection::setJobAckCallback, this);
    ignore_inspection_sub = n.subscribe("/gui/checkbox/ignore_inspection", 1, &WeavingInspection::ignoreInspectionCallback, this);
    data_collection_sub = n.subscribe("/gui/checkbox/data_collection", 1, &WeavingInspection::dataCollectionCallback, this);
    light_toggle_sub = n.subscribe("/gui/checkbox/light_toggle", 1, &WeavingInspection::lightToggleCallback, this);
    barcode_config_sub = n.subscribe("/gui/set/barcode", 1, &WeavingInspection::setBarcodeCallback, this);
    work_order_config_sub = n.subscribe("/gui/set/workOrder", 1, &WeavingInspection::setWorkOrderCallback, this);
    version_value_sub = n.subscribe("/gui/value/version", 1, &WeavingInspection::versionCallback, this);
    body_counter_sub = n.subscribe("/gui/label/next_stopping", 100, &WeavingInspection::bodyCounterCallback, this);
    roll_continue_ack_sub = n.subscribe("/gui/button/continueAck", 100, &WeavingInspection::rollContinueAckcallback, this);
}

/**
 * @brief Function to initialize the system variables and threads.
 */
void WeavingInspection::initSystem()
{
    // update panel positions
    report_image_save_path = boost::filesystem::path("/images/").string();
    full_image_saver = new RobroImageSaver("/images/", "full_image_", ".jpg");
    store = new ParamStore(project_base_path);
    json stop_at_last_defect = store->getParam("stop_at_last_defect");

    if (stop_at_last_defect != nullptr)
    {
        get_last_defect_in_panel = stop_at_last_defect.get<bool>();
    }
    else
    {
        store->setParam("stop_at_last_defect", get_last_defect_in_panel);
    }
    // calculate mm per frame
    mm_per_frame = frame_height / (float)pixel_per_mm;
    cam2stopper_distance = original_cam2Stopper_mm + offset_cm;

    if (waitUntilCamerasAreReady())
    {
        std::thread defect_processing_thread(&WeavingInspection::processDefectsAndPublishResults, this);
        defect_processing_thread.detach();
    }
    // Save Camera Configs
    if (!cam_config_map.empty())
    {
        json cam_config_agg = json::object();
        json cams = json::object();

        for (auto &entry : cam_config_map)
        {
            const std::string &cam_id = entry.first;
            json &cam_config = entry.second;

            // Check and format x_px_per_mm to 2 decimals
            if (cam_config.contains("x_px_per_mm"))
            {
                double x_px_per_mm = cam_config["x_px_per_mm"].get<double>();
                x_px_per_mm = std::round(x_px_per_mm * 100.0) / 100.0;

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << x_px_per_mm;
                cam_config["x_px_per_mm"] = oss.str();
            }

            cams[cam_id] = cam_config;
            logger.add_system_configuration("cam" + cam_id, getAppID(), LogComponent::CAMERA, cam_config, project_id);
        }
    }

    // Save AppConfig if not exists
    if (!logger.check_system_configuration_exists(getAppID(), "AppConfig"))
    {
        json app_config = json::object();
        app_config["App"] = json::object();
        // Save App configuration
        if (!logger.add_system_configuration("AppConfig", getAppID(), LogComponent::APP, app_config, project_id))
        {
            addSystemLog("INFO", "App Configuration saved failed", "APP024");
        }
    }
}

/**
 * @brief Function will be waiting until all the cameras are ready.
 * it uses camera configuration publisher and subscriber
 * to check the camera status.
 */
bool WeavingInspection::waitUntilCamerasAreReady()
{
    ready_camera_ids.clear();
    int number_of_times_to_check = 120;
    while (ready_camera_ids.size() < number_of_cameras)
    {
        // publish the camera config sync message
        camera_config_sync_pub.publish(std_msgs::Empty());

        // Spin and wait for cam sync subscribers to populate ready cameras
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!number_of_times_to_check)
        {
            sendNotification("Code: CSY008 - [Inspection] Cameras Not ready! Please Restart.", "popup");
            addSystemLog("WARNING", "[waitUntilCamerasAreReady] All Cameras not ready. Ready camera count: " + std::to_string(ready_camera_ids.size()), "CSY008");
            return false;
        }

        number_of_times_to_check--;
    }
    ready_camera_ids.clear();
    return true;
}

void WeavingInspection::waitUntilVersionReceived()
{
    version.clear();
    int number_of_times_to_check = 100;
    while (version.empty())
    {
        version_pub.publish(std_msgs::Empty());
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::string WeavingInspection::getcomponentname(std::string component_name)
{

    return component_name;
}

/**
 * @brief Get the Time in string format of YYYY-MM-DD HH:MM:SS
 *
 * @return std::string - Time in string format of YYYY-MM-DD HH:MM:SS
 */
std::string WeavingInspection::getTimeStr()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    s.resize(std::strlen(s.c_str()));
    return s;
}

/**
 * @brief Function to generate configuration id using current time.
 */
std::string WeavingInspection::generateConfigurationIDString()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm *local_time = std::localtime(&time_t_now);

    // Format the time into the desired roll_id format
    std::ostringstream oss;
    oss << "conf_" << std::put_time(local_time, "%d%m%Y_%H%M%S");

    return oss.str();
}

/**
 * @brief Function to generate work id using current time.
 */
std::string WeavingInspection::generateWorkOrderIDString()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm *local_time = std::localtime(&time_t_now);

    // Format the time into the desired roll_id format
    std::ostringstream oss;
    oss << "WO" << std::put_time(local_time, "%d%m%Y_%H%M%S");

    return oss.str();
}

/**
 * @brief Checks if the images are available in the queue for processing.
 *
 * @return true - if images are available.
 * @return false - otherwise.
 */
bool WeavingInspection::isImagesAvailable()
{
    // Check if we have received the single image in every queue.
    bool images_available = false;
    int seq_no_from_camera = -1;
    try
    {
        // traverse over all the cameras
        for (auto it = all_camera_image_map.begin();
             it != all_camera_image_map.end();
             ++it)
        {
            // Check if we have received the image from this camera
            if (!it->second.empty())
            {
                images_available = true;
            }
            else
            {
                images_available = false;
                break;
            }

            // This is the check if the sequences of both the images in front are also same
            if (seq_no_from_camera == -1)
            {
                // Get the sequence number of the first image in the queue.
                seq_no_from_camera = it->second.front().first;
            }
            else
            {
                if (it->second.front().first != seq_no_from_camera)
                {
                    ROS_ERROR("SEQUENCE NUMBERS OF Cam: %s Seq: %d dont match with %d", it->first.c_str(), it->second.front().first, seq_no_from_camera);
                    sendNotification("Code: CSY002 - Image mismatch. Seq no. Cam " + it->first + " Seq: " + std::to_string(it->second.front().first) +
                                         " & " + std::to_string(seq_no_from_camera) + " PLEASE STOP & RESET",
                                     "popup");
                    addSystemLog("ERROR", "[isImagesAvailable] Seq no. Cam " + it->first + " Seq: " + std::to_string(it->second.front().first) + " dont match with " + std::to_string(seq_no_from_camera), "CSY002");
                }
            }
        }

        return images_available;
    }
    catch (...)
    {
        std::cerr << "[isImagesAvailable] An exception occurred." << std::endl;
        addSystemLog("ERROR", "Code: [isImagesAvailable] An exception occurred.", "APP011");
        return false;
    }
}

/**
 * @brief Checks if the defects are available in the queue for processing.
 *
 * @return true - if defects are available.
 * @return false - otherwise.
 */
bool WeavingInspection::isDetectionsAvailable()
{
    try
    {
        // If we have images from all cameras, do we also have the detections from all cameras.
        bool detection_available = false;

        for (auto it = all_camera_detection_map.begin();
             it != all_camera_detection_map.end();
             it++)
        {
            if (!it->second.empty())
            {
                detection_available = true;
            }
            else
            {
                detection_available = false;
                ROS_INFO_THROTTLE(30, "Don't have Defect data from %s - Not Processing right now!", it->first.c_str());
                break;
            }
            ROS_DEBUG_THROTTLE(30, "[Cam %s] Total Defects Recd: %d", (it->first).c_str(), num_defects_recd_map[it->first]);
        }
        return detection_available;
    }
    catch (...)
    {
        std::cerr << "[isDetectionsAvailable] An exception occurred." << std::endl;
        addSystemLog("ERROR", "[isDetectionsAvailable] An exception occurred.", "APP012");
        return false;
    }
}

/**
 * @brief Prepare combined annotated image from all cameras.
 *
 * @param combined_annotated_image - Combined annotated image from all cameras.
 * @return  true - if the image is prepared successfully.
 * @return false - otherwise.
 */
bool WeavingInspection::prepareCombineAnnotatedImage(cv::Mat &combined_annotated_image)
{
    try
    {
        std::set<int> seq_ids;
        std::vector<int> size_of_queues;
        int min_size = INT_MAX;
        for (auto it = all_camera_image_map.begin();
             it != all_camera_image_map.end();
             it++)
        {
            if (it->second.size())
            {
                seq_ids.insert(it->second.front().first);
            }
            size_of_queues.push_back(it->second.size());
            min_size = std::min(min_size, (int)it->second.size());
            if (combined_annotated_image.empty())
            {
                it->second.front().second.copyTo(combined_annotated_image);
            }
            else
            {
                try
                {
                    cv::hconcat(combined_annotated_image, it->second.front().second, combined_annotated_image);
                }
                catch (const std::exception &e)
                {
                    std::cout << "[ERROR] Can't HConcat:" << e.what() << std::endl;
                    sendNotification("Error in HConcat", "alert");
                    addSystemLog("ERROR", "[prepareCombineAnnotatedImage] Error in HConcat", "APP013");
                }
            }
        }
        bool issue_stopping = false;
        bool found = false;
        std::string size_str = "There is queue size mismatch in the cameras is";
        for (int i = 0; i < size_of_queues.size(); ++i)
        {
            int num = size_of_queues[i] - min_size;
            if (num > 0)
            {
                found = true;
            }
            size_str += " " + std::to_string(size_of_queues[i]);
        }
        if (found)
        {
            ROS_ERROR("%s", size_str.c_str());
        }
        if (seq_ids.size() > 1)
        {
            std::string sequence_data = "";
            for (auto const &id : seq_ids)
            {
                sequence_data += std::to_string(id);
                sequence_data += ',';
            }
            issue_stopping = true;
            sequence_data = "Code: CSY002: ";
            sequence_data += "Frames miss match: " + sequence_data + "Please click on ↺ Reset button to reset the system.";
            addSystemLog("ERROR", sequence_data, "CSY002");
            sendNotification(sequence_data, "popup");
        }
        if (issue_stopping)
        {
            json metadata;
            metadata["type"] = "plc_metadata";
            metadata["is_immediate_stopping_requested"] = true;
            std_msgs::String meta_msg;
            meta_msg.data = metadata.dump();
            metadata_pub.publish(meta_msg);
        }

        return true;
    }
    catch (...)
    {
        std::cerr << "[prepareCombineAnnotatedImage] An exception occurred." << std::endl;
        addSystemLog("ERROR", "[prepareCombineAnnotatedImage] An exception occurred.", "APP013");
        return false;
    }
}

/**
 * @brief Add the defects to the world map. Where they will be processed for stopping and punching.
 *
 * @param total_number_of_defects - Total number of defects to be added to the world map.
 * @return true - if the defects are added successfully.
 * @return false - otherwise.
 */
bool WeavingInspection::addDefectsToWorldMap(int &total_number_of_defects)
{
    try
    {
        total_number_of_defects = 0;

        vision_msgs::Detection2DArray this_frame_defects;
        // update the current frame sequence
        current_frame_seq++;

        // add all the defects in this frame to the this_frame_defects vector
        for (auto it = all_camera_detection_map.begin();
             it != all_camera_detection_map.end();
             it++)
        {
            if (!it->second.empty())
            {
                if (!it->second.front().detections.empty())
                {
                    auto full_frame_detection = it->second.front().detections.back();
                    it->second.front().detections.pop_back();
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(full_frame_detection.source_img, sensor_msgs::image_encodings::BGR8);
                    // Save image
                    std::string full_frame_path = full_image_saver->save(cv_ptr->image.clone());
                    logger.add_dataset_file(dataset_id, full_frame_path);

                    for (auto detection : (it->second.front()).detections)
                    {
                        json j = json::parse(detection.header.frame_id);
                        j["cam_id"] = it->first;
                        j["full_frame_path"] = full_frame_path;
                        detection.header.frame_id = j.dump();
                        this_frame_defects.detections.push_back(detection);
                    }
                }

                total_number_of_defects += (it->second.front()).detections.size();
                it->second.pop();
            }
        }
        json j;
        j["seq"] = current_frame_seq;
        j["roll_info_total_meters"] = roll_info_total_meters;
        this_frame_defects.header.frame_id = j.dump();
        // add the defects to world map
        if (total_number_of_defects > 0)
        {
            combine_defects_pub.publish(this_frame_defects);
        }

        return true;
    }
    catch (...)
    {
        std::cerr << "[addDefectsToWorldMap] An exception occurred." << std::endl;
        addSystemLog("ERROR", "[addDefectsToWorldMap] An exception occurred.", "APP014");
        return false;
    }
}

/**
 * @brief Publish the inspected image and remove from all camera image map.
 */
void WeavingInspection::publishAndRemoveTheImage()
{
    // Publish the single camera inspected image
    try
    {
        sensor_msgs::ImagePtr result_image;
        if (!all_camera_image_map.empty() && all_camera_image_map.begin()->second.size() > 0)
        {
            for (auto it = all_camera_image_map.begin();
                 it != all_camera_image_map.end();
                 it++)
            {
                auto cam_img = it->second.front().second;
                cv::resize(cam_img,
                           cam_img,
                           cv::Size(300, 300));
                json j;
                j["cam_id"] = it->first;
                j["position"] = cam_config_map[it->first]["position"];
                result_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_img).toImageMsg();
                result_image->header.frame_id = j.dump();
                inspected_image_pub.publish(result_image);
            }
        }
        // Remove the processed images from the queue
        for (auto it = all_camera_image_map.begin();
             it != all_camera_image_map.end();
             ++it)
        {
            if (!it->second.empty())
            {
                it->second.pop();
            }
        }
    }
    catch (...)
    {
        std::cerr << "[publishAndRemoveTheImage] An exception occurred." << std::endl;
        addSystemLog("ERROR", "[publishAndRemoveTheImage] An exception occurred.", "APP014");
    }
}

/**
 * @brief The Function process the defects and publish the results. Function runs in a separate thread.
 * It extract the defects from the queue and process them. It also publishes the results to the PLC and the UI.
 */
void WeavingInspection::processDefectsAndPublishResults()
{
    ros::Rate rate(30);
    std_msgs::String current_result_msg;
    int number_of_defects{0};
    IntervalTask interval_task_update_roll_info(5000);
    while (ros::ok())
    {
        try
        {

            // Check if we have received the image and defect data from all cameras atleast once.
            if (all_camera_image_map.size() != number_of_cameras ||
                all_camera_detection_map.size() != number_of_cameras)
            {
                ROS_WARN_THROTTLE(
                    60,
                    "Haven't yet received either image data (map size: %d) or defect data (map size: %d) from all cameras (Expected from: %d cameras) for the first time. Not doing anything...",
                    (int)all_camera_image_map.size(), (int)all_camera_detection_map.size(),
                    number_of_cameras);
                rate.sleep();
                continue;
            }
            if (isImagesAvailable() && isDetectionsAvailable())
            {
                current_result_msg.data = "OK";
                number_of_defects = 0;

                cv::Mat this_defected_image;
                if (prepareCombineAnnotatedImage(this_defected_image) && addDefectsToWorldMap(number_of_defects))
                {
                    if (number_of_defects)
                    {
                        current_result_msg.data = "NG";
                        roll_info_total_defects += number_of_defects;

                        sensor_msgs::ImagePtr defected_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this_defected_image).toImageMsg();
                        defected_image->header.frame_id = std::to_string(current_frame_seq);
                        defected_image_pub.publish(defected_image);
                    }
                    roll_info_total_meters += (mm_per_frame / 1000.0);

                    running_meter_publish_msg.data = std::to_string(roll_info_total_meters);

                    json message;
                    message["main_processed_images"] = current_frame_seq;
                    system_health_msg.data = message.dump();
                    system_health_msg_pub.publish(system_health_msg);

                    inspection_category_pub.publish(current_result_msg);
                    running_meter_pub.publish(running_meter_publish_msg);

                    // publish results to GUI
                    ng_count_publish_msg.data = std::to_string(roll_info_total_defects);
                    ng_count_pub.publish(ng_count_publish_msg);

                    publishAndRemoveTheImage();
                }
                if (is_roll_started && interval_task_update_roll_info.isReady())
                {
                    roll.inspected_length = roll_info_total_meters;
                    roll.total_defects = roll_info_total_defects;
                    std_msgs::String log;
                    log.data = roll.toString();
                    roll_log_pub.publish(log);
                    interval_task_update_roll_info.reset();
                }
            }

            rate.sleep();
        }
        catch (...)
        {
            std::cerr << "[processDefectsAndPublishResults] An exception occurred." << std::endl;
            addSystemLog("ERROR", "[processDefectsAndPublishResults] An exception occurred.", "APP014");
        }
    }
}

/**
 * @brief Send notification to the UI.
 *
 * @param msg - Message to be sent.
 * @param severity - Severity of the message.
 */
void WeavingInspection::sendNotification(std::string msg, std::string severity)
{
    json message;
    message["msg"] = msg;
    message["severity"] = severity;
    notification_msg.data = message.dump();
    notification_pub.publish(notification_msg);
}

/**
 * @brief Function to call when roll starts.The function will log the
 * roll start time and update the roll info.
 * @param metadata - Metadata of the roll.
 */
void WeavingInspection::onRollStart(json metadata)
{

    weaving_inspection::InsertDataLog srv;
    // convert to mm and add the offset. This value we receive is just the offset.
    if (metadata["roll_id"] == "")
    {
        sendNotification("Invalid Roll ID", "warning");
        metadata["roll_id"] = "NA";
    }

    bool create_new_job = false;
    bool is_roll_exists = false;
    bool sync_ui = false;

    get_roll_data_client.waitForExistence();
    weaving_inspection::GetRollLog roll_log_srv;
    roll_log_srv.request.customer_roll_id = metadata["roll_id"];
    roll_log_srv.request.primary_cut_length = metadata["cut_length"];
    if (secondary_cut_lengths.size() == 1)
    {
        roll_log_srv.request.secondary_cut_length = secondary_cut_lengths[0];
        roll_log_srv.request.tertiary_cut_length = 0;
    }
    else if (secondary_cut_lengths.size() == 2)
    {
        roll_log_srv.request.secondary_cut_length = secondary_cut_lengths[0];
        roll_log_srv.request.tertiary_cut_length = secondary_cut_lengths[1];
    }

    if (get_roll_data_client.call(roll_log_srv))
    {
        // json metadata;
        primary_body_count = roll_log_srv.response.primary_body_count;
        secondary_body_count = roll_log_srv.response.secondary_body_count;
        tertiary_body_count = roll_log_srv.response.tertiary_body_count;
        defective_body_count = roll_log_srv.response.defective_body_count;
        total_defective_length = roll_log_srv.response.defective_length;
        last_inserted_job_id = roll_log_srv.response.total_jobs;

        total_bodies = roll_log_srv.response.total_bodies;

        {
            roll = Roll();
            json roll_json = json::parse(roll_log_srv.response.data);
            roll.loadJSON(roll_json);

            roll.loom_number = logger.get_loom_id_for_roll(roll.robro_roll_id);
            roll_info_total_defects = roll.total_defects;

            roll_info_total_meters = roll.inspected_length;
            roll.roll_end_time = "";

            running_meter_publish_msg.data = std::to_string((float)(roll_info_total_meters));
            running_meter_pub.publish(running_meter_publish_msg);

            ng_count_publish_msg.data = std::to_string(roll_info_total_defects);
            ng_count_pub.publish(ng_count_publish_msg);

            sendNotification("Roll ID already exists!", "success");

            running_meter_publish_msg.data = std::to_string((float)(roll_info_total_meters));
            ng_count_publish_msg.data = std::to_string(roll_info_total_defects);

            running_meter_pub.publish(running_meter_publish_msg);
            ng_count_pub.publish(ng_count_publish_msg);

            is_roll_exists = true;
            last_inserted_job_id++;
            addSystemLog("ACTIVITY", "Roll Start Button Pressed and Roll is already present: with robro roll id: " + std::to_string(roll.robro_roll_id), "INFO016");
        }
    }

    // I have the same roll as before
    if (!is_roll_exists)
    {
        roll = Roll();
        roll.customer_roll_id = metadata["roll_id"];
        roll.machine_id = project_id;
        roll.roll_start_time = getTimeStr();
        roll.gsm = metadata["gsm"];
        roll.weight = metadata["roll_weight"];
        roll.width = metadata["fabric_width"];
        roll.width = roll.width * 10;
        roll.roll_end_time = "";
        roll.material_type = metadata["layer_value"];

        roll.loom_number = (metadata.contains("loom_id") && metadata["loom_id"].is_string() && !metadata["loom_id"].get<std::string>().empty())
                               ? metadata["loom_id"].get<std::string>()
                               : "";

        primary_body_count = 0;
        secondary_body_count = 0;
        tertiary_body_count = 0;
        defective_body_count = 0;
        total_defective_length = 0;
        total_bodies = 0;

        if (metadata.contains("roll_length"))
        {
            roll.roll_length = metadata["roll_length"];
        }
        else
        {
            roll.roll_length = roll.weight * 1000 / float(roll.gsm * (roll.width / 1000.0));
        }

        std_msgs::String log;
        log.data = roll.toString();
        // roll_log_pub.publish(log);
        srv.request.table = "roll";
        srv.request.data = log.data;
        if (client.call(srv))
        {
            roll.robro_roll_id = (srv.response.id);
        }
        else
        {
            ROS_ERROR("Failed to call service /report/insert/log");
        }

        create_new_job = true;

        roll_info_total_meters = 0;
        roll_info_total_defects = 0;
        last_inserted_job_id = 1;

        logger.add_roll_manufacturing_info(roll.robro_roll_id, roll.loom_number);

        addSystemLog("ACTIVITY", "Roll Start Button Pressed and Roll is entry created: the robro roll id: " + std::to_string(roll.robro_roll_id), "INFO016");
    }
    else
    {
        std::vector<std::string> conflicts;
        std::string msg, newLoomNumber;

        newLoomNumber = (metadata.contains("loom_id") && metadata["loom_id"].is_string() && !metadata["loom_id"].get<std::string>().empty())
                            ? metadata["loom_id"].get<std::string>()
                            : "";

        if (newLoomNumber != roll.loom_number)
        {
            conflicts.push_back("<div><span style='font-weight:600;'>Loom ID:</span> <b>" +
                                roll.loom_number + "</b></div>");
        }

        if (metadata.contains("roll_length") && metadata["roll_length"] != roll.roll_length)
        {
            conflicts.push_back("<div><span style='font-weight:600;'>Roll Length:</span> <b>" +
                                (static_cast<std::ostringstream &>(std::ostringstream() << std::fixed << std::setprecision(2) << roll.roll_length).str()) +
                                "</b></div>");
        }

        if (metadata.contains("fabric_width"))
        {
            int width = metadata["fabric_width"];
            width *= 10;
            if (width != roll.width)
            {
                conflicts.push_back("<div><span style='font-weight:600;'>Fabric Width:</span> <b>" +
                                    std::to_string(roll.width) + "</b></div>");
            }
        }

        if (metadata.contains("gsm") && metadata["gsm"] != roll.gsm)
        {
            conflicts.push_back("<div><span style='font-weight:600;'>GSM:</span> <b>" +
                                std::to_string(roll.gsm) + "</b></div>");
        }
        if (metadata.contains("layer_value") && metadata["layer_value"].get<std::string>() != roll.material_type)
        {
            conflicts.push_back("<div><span style='font-weight:600;'>Fabric Type:</span> <b>" +
                                (roll.material_type) + "</b></div>");
        }

        if (!conflicts.empty())
        {
            sync_ui = true;
            msg += "<div style='text-align:center;'>";
            msg += "Following roll properties cannot be changed  <br><br>";
            // msg += "[<b>" + roll.customer_roll_id + "</b>]<br><br>";

            for (const std::string &conflict : conflicts)
            {
                msg += conflict;
            }

            msg += "</div>"; // Close center div
        }
        roll_continue_ack = -1;
        std_msgs::String continue_msg;
        continue_msg.data = msg;

        // Roll ID already exists, so publish the "roll_continue" message
        roll_continue_ack_gui_pub.publish(continue_msg);

        // Wait for acknowledgement
        ros::Rate rate(10); // 10 Hz

        while (ros::ok)
        {
            ros::spinOnce();

            if (roll_continue_ack != -1)
                break;

            rate.sleep();
        }

        if (roll_continue_ack == 0)
        {

            return;
        }
    }

    if (cut_length_mm != metadata["cut_length"] ||
        offset_cm != metadata["offset"] ||
        job.recipe != metadata["recipe_name"])
    {
        create_new_job = true;
    }

    cut_length_mm = metadata["cut_length"];
    cut_length_mm *= 10;

    offset_cm = metadata["offset"];
    cam2stopper_distance = original_cam2Stopper_mm + (offset_cm * 10);
    roll_info_punching_on = metadata["punching_status"];
    ignore_inspection_on = metadata["ignore_inspection"];
    cutting_mode = metadata["cutting_mode"];
    secondary_cut_lengths.clear();
    if (metadata.contains("secondary_cut_lengths"))
    {
        secondary_cut_lengths = metadata["secondary_cut_lengths"].get<std::vector<int>>();
    }

    if (cut_length_mm > cam2stopper_distance)
    {
        sendNotification("Long Cut Length. Punch Block Won't work for end of body defects.", "warning");
    }
    if (create_new_job)
    {
        job = Job();
        job.job_id = last_inserted_job_id;
        job.job_start_meter = roll_info_total_meters;
        job.robro_roll_id = roll.robro_roll_id;
        job.cut_length = cut_length_mm;
        job.start_time = getTimeStr();
        job.recipe = metadata["recipe_name"];
        // Convert cm to mm

        if (secondary_cut_lengths.size() == 1)
        {
            job.secondary_cut_length = secondary_cut_lengths[0] * 10;
        }
        else if (secondary_cut_lengths.size() == 2)
        {
            job.secondary_cut_length = secondary_cut_lengths[0] * 10;
            job.tertiary_cut_length = secondary_cut_lengths[1] * 10;
        }
        if (metadata.contains("work_order_id"))
        {
            job.work_order_id = std::stoi(metadata["work_order_id"].get<std::string>());
        }
        if (metadata.contains("batch_count"))
        {
            job.batch_count = metadata["batch_count"];
        }

        std_msgs::String msg;
        msg.data = job.toString();
        job_log_pub.publish(msg); // Publish the job log
    }

    // Reset the paths for the image to save and create directories.
    std::string this_dataset_id = logger.create_dataset_id(metadata["roll_id"].get<std::string>(), metadata["recipe_name"].get<std::string>(), metadata["id"].get<std::string>());
    dataset_id = roll.robro_roll_id;
    boost::filesystem::path base_path(report_image_save_path);

    // Creating a folder in app_id
    boost::filesystem::path project_path = base_path / project_id / std::to_string(dataset_id);
    boost::filesystem::create_directories(project_path);

    // Updating a base path to dataset_path
    base_path = project_path / this_dataset_id;
    std::string current_defected_frame = boost::filesystem::path(base_path / "defected_frames/").string();
    boost::filesystem::create_directories(current_defected_frame);
    full_image_saver->base_path = current_defected_frame;

    is_roll_started = true;
    metadata["roll_id"] = roll.robro_roll_id;
    metadata["customer_roll_id"] = roll.customer_roll_id;
    metadata["job_id"] = job.job_id;
    metadata["type"] = "roll_metadata";
    metadata["total_defects"] = roll_info_total_defects;
    std_msgs::String msg;
    msg.data = metadata.dump();
    metadata_pub.publish(msg);
    json roll_metadata;
    roll_metadata["type"] = "real_world_metadata";
    roll_metadata["stop_at_last_defect"] = get_last_defect_in_panel;
    roll_metadata["primary_body_count"] = primary_body_count;
    roll_metadata["secondary_body_count"] = secondary_body_count;
    roll_metadata["tertiary_body_count"] = tertiary_body_count;
    roll_metadata["defective_body_count"] = defective_body_count;
    roll_metadata["total_bodies"] = total_bodies;
    roll_metadata["total_defective_length"] = total_defective_length;
    std_msgs::String meta_msg;
    meta_msg.data = roll_metadata.dump();
    metadata_pub.publish(meta_msg);

    if (sync_ui)
    {
        // sync configuration
        systemConfigSyncCallback(std_msgs::Empty());
    }
}

/**
 * @brief Function to call when roll ends.The function will log
 * roll end time, update the roll info and reset the system.
 */
void WeavingInspection::onRollEnd()
{

    addSystemLog("ACTIVITY", "Roll End Button Pressed", "INFO017");
    job.end_time = getTimeStr();
    job.primary_body_count = body_counter_vector[0] - primary_body_count;
    if (body_counter_vector.size() > 2)
    {
        job.secondary_body_count = body_counter_vector[1] - secondary_body_count;
        job.tertiary_body_count = body_counter_vector[2] - tertiary_body_count;
        job.defective_body_count = body_counter_vector[3] - defective_body_count;
    }
    else
    {
        job.defective_body_count = body_counter_vector[1] - defective_body_count;
    }
    job.job_end_meter = roll_info_total_meters;
    std_msgs::String job_log;
    job_log.data = job.toString();
    job_log_pub.publish(job_log);

    is_roll_started = false;
    roll.roll_end_time = getTimeStr();
    roll.inspected_length = roll_info_total_meters;
    roll.total_defects = roll_info_total_defects;

    std_msgs::String log;
    log.data = roll.toString();
    roll_log_pub.publish(log);

    json roll_metadata;
    roll_metadata["type"] = "roll_metadata";
    roll_metadata["is_roll_started"] = false;
    roll_metadata["reset_batch_count"] = true;
    std_msgs::String meta_msg;
    meta_msg.data = roll_metadata.dump();
    metadata_pub.publish(meta_msg);
}

/**
 * @brief Add the system log to into system log server.
 *
 * @param severity - Severity of the log.
 * @param msg - Message to be logged.
 */
void WeavingInspection::addSystemLog(std::string severity, std::string msg, std::string msg_code)
{
    logger.add_log(getLogSeverityFromString(severity), getLogCodeFromString(msg_code), msg, LogComponent::APP);
}

/**
 * @brief Callback for the annotated image from the camera.
 *
 * @param msg - annotated image.
 */
void WeavingInspection::camAnnotatedImageCallback(const sensor_msgs::Image &msg)
{
    // convert the image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // get the camera id from the frame_id
    json frame_id_and_seq_json = json::parse(msg.header.frame_id);
    if (frame_id_and_seq_json.contains("cam_id") && frame_id_and_seq_json.contains("seq"))
    {
        std::string cam_id = frame_id_and_seq_json["cam_id"];
        // add the image to the map
        all_camera_image_map[cam_id].push(std::make_pair((int)(frame_id_and_seq_json["seq"]), cv_ptr->image));

        if (all_camera_image_map[cam_id].size() > 15 && interval_task_main_q_size_logger.isReady())
        {

            // create msg with q sizes of all cameras
            std::string error_msg = "Q Size in Main Large: ";
            for (auto it = all_camera_image_map.begin(); it != all_camera_image_map.end(); ++it)
            {
                error_msg += "Cam: " + it->first + " Size: " + std::to_string(it->second.size()) + " ";
            }

            sendNotification(error_msg, "warning");
            addSystemLog("WARNING", "[camAnnotatedImageCallback] " + error_msg, "PERF001");
            interval_task_main_q_size_logger.reset();
        }

        // increment the number of image received for this camera.
        if (num_imgs_recd_map.find(cam_id) == num_imgs_recd_map.end())
        {
            num_imgs_recd_map[cam_id] = 1;
        }
        else
        {
            num_imgs_recd_map[cam_id]++;
        }
    }
    else
    {
        std::cout << "\033[1;31m ERROR : \033[0m[camAnnotatedImageCallback] Json doesn't contain cam_id or seq";
        addSystemLog("ERROR", "[camAnnotatedImageCallback] Json doesnt contain cam_id or seq: " + frame_id_and_seq_json.dump(), "APP009");
    }
}

/**
 * @brief Getting defect details from camera node as a message including
 * (x,y), width, height, frame_id, probability e.t.c.
 *
 * @param msg vision_msgs::Detection2DArray
 */
void WeavingInspection::defectDetailsCallback(const vision_msgs::Detection2DArray &msg)
{
    json frame_id_and_seq_json = json::parse(msg.header.frame_id);
    if (frame_id_and_seq_json.contains("cam_id"))
    {
        std::string cam_id = frame_id_and_seq_json["cam_id"];
        // add the detection to the queue of the respective camera.
        all_camera_detection_map[cam_id].push(msg);
        // increment the number of defects received for this camera.
        if (num_defects_recd_map.find(cam_id) == num_defects_recd_map.end())
        {
            num_defects_recd_map[cam_id] = 1;
        }
        else
        {
            num_defects_recd_map[cam_id]++;
        }
    }
    else
    {
        std::cout << "\033[1;31m ERROR : \033[0m[DefectDetailsCallback] No cam_id in frame_id_and_seq_json";
        addSystemLog("ERROR", "[DefectDetailsCallback] No cam_id in frame_id_and_seq_json" + msg.header.frame_id, "APP009");
    }
}

/**
 * @brief Callback to update material type value (L/UL)
 *
 * @param msg - material type
 */
void WeavingInspection::materialTypeCallback(const std_msgs::String &msg)
{
    roll.material_type = msg.data;
    // roll.roll_end_time = "";

    // std_msgs::String roll_log;
    // if (is_roll_started)
    // {
    //     roll_log.data = roll.toString();
    //     roll_log_pub.publish(roll_log);
    // }
}

/**
 * @brief  Metadata change callback
 *
 * @param msg - metadata change status
 */
void WeavingInspection::metadataChangeCallback(const std_msgs::String &msg)
{
    json metadata = json::parse(msg.data);
    is_job_change_enabled = true;
    json roll_camera_metdata;
    roll_camera_metdata = metadata;
    if (!(metadata.contains("cut_length") &&
          metadata.contains("roll_weight") &&
          metadata.contains("fabric_width") &&
          metadata.contains("gsm") &&
          metadata.contains("roll_id") &&
          metadata.contains("offset") &&
          metadata.contains("punching_status") &&
          metadata.contains("is_roll_started") &&
          metadata.contains("recipe_name") &&
          metadata.contains("ignore_inspection") &&
          metadata.contains("cutting_mode")))
    {
        sendNotification("Invalid metadata", "warning");
        addSystemLog("WARNING", "[metadataChangeCallback] Invalid Metadata.", "APP004");
        return;
    }

    if (metadata["is_roll_started"])
    {
        onRollStart(metadata);
        if (roll_continue_ack == 0)
        {
            roll_continue_ack = -1;
            return;
        }
        resetSystemCallback(std_msgs::Empty());
        sendNotification("Roll Change ON", "info");
    }
    else
    {
        onRollEnd();
        sendNotification("Roll Change OFF", "info");
    }

    if (metadata.contains("fully_ignore_disabled"))
    {
        fully_ignore_disabled_classes = metadata["fully_ignore_disabled"];
    }

    if (metadata.contains("top_bottom_panel_selected"))
    {
        top_bottom_panel_selected = metadata["top_bottom_panel_selected"];
    }

    roll_camera_metdata["robro_roll_id"] = roll.robro_roll_id;
    std_msgs::String camera_meta_msg;
    camera_meta_msg.data = roll_camera_metdata.dump();
    cam_metadata_pub.publish(camera_meta_msg);

    json message;
    message["main_processed_images"] = current_frame_seq;
    system_health_msg.data = message.dump();
    system_health_msg_pub.publish(system_health_msg);
    is_job_change_enabled = false;
}

/**
 * @brief Set the Pixel Per MM as per callback from GUI
 *
 * @param msg - pixel per mm
 */
void WeavingInspection::pixelPerMMCallback(const std_msgs::Float32 &msg)
{
    if (msg.data < 0.0 || msg.data > 100.0)
    {
        sendNotification("Invalid Pixel Per MM: " + std::to_string(msg.data), "alert");
        return;
    }
    pixel_per_mm = msg.data;
    // calculate the mm per frame
    mm_per_frame = (float)frame_height / pixel_per_mm;
    json metadata;
    metadata["type"] = "real_world_metadata";
    metadata["pixel_per_mm"] = pixel_per_mm;
    std_msgs::String meta_msg;
    meta_msg.data = metadata.dump();
    metadata_pub.publish(meta_msg);
    sendNotification("New Pixel Per MM: " + std::to_string(pixel_per_mm), "success");
}

/**
 * @brief Set the Pulse Per MM As per callback from GUI
 *
 * @param msg - pulse per mm
 */
void WeavingInspection::pulsePerMMCallback(const std_msgs::Float32 &msg)
{
    if (msg.data < 0.0 || msg.data > 100.0)
    {
        sendNotification("Invalid Pulse Per MM: " + std::to_string(msg.data), "alert");
        addSystemLog("INFO", "[pulsePerMMCallback] Invalid Pulse per MM" + std::to_string(msg.data), "APP004");
        return;
    }
    pulse_per_mm = msg.data;
    sendNotification("New Pulse Per MM: " + std::to_string(pulse_per_mm), "success");
}

/**
 * @brief Reset the system. Clear all the queues and maps.
 *
 * @param msg - Empty message
 */
void WeavingInspection::resetSystemCallback(const std_msgs::Empty &msg)
{
    std_msgs::String reset_msg;
    json config;

    current_frame_seq = 0;
    /**
     * @todo
     * 1. how to handle the good and bad body counts on reset
     */
    // Clear all the maps and queues
    all_camera_image_map.clear();
    all_camera_detection_map.clear();

    num_defects_recd_map.clear();
    num_imgs_recd_map.clear();

    // Publish the reset message
    reset_msg.data = "NA";
    // inspection_category_pub.publish(reset_msg);
    reset_msg.data = "0.0";
    reset_system_pub.publish(std_msgs::Empty());
    running_meter_pub.publish(reset_msg);
    ng_count_pub.publish(reset_msg);

    // Publish the system health message
    config["main_processed_images"] = current_frame_seq;

    system_health_msg.data = config.dump();
    system_health_msg_pub.publish(system_health_msg);
    addSystemLog("INFO", "[Main][resetCallback] Done", "APP001");
}

/**
 * @brief callback to restart the system
 *
 * @param msg - empty message
 */
void WeavingInspection::restartCallback(const std_msgs::Empty &msg)
{
    /**
     * @todo close the application and restart the system.
     */
    system("/sbin/reboot");
}

/**
 * @brief Sync the system config with the GUI.
 *
 * @param msg - Empty message
 */
void WeavingInspection::systemConfigSyncCallback(const std_msgs::Empty &msg)
{
    json config;
    // Generate the config for the GUI
    config["main_processed_images"] = current_frame_seq;
    config["roll_id"] = roll.customer_roll_id;
    config["gsm"] = roll.gsm;
    config["offset"] = offset_cm;
    config["punch_on"] = roll_info_punching_on;
    config["material"] = roll.material_type;
    config["roll_weight"] = roll.weight;
    config["fabric_width"] = roll.width / 10.0;
    config["pixel_per_mm"] = pixel_per_mm;
    config["pulse_per_mm"] = pulse_per_mm;
    config["cut_length"] = (int)(cut_length_mm / 10.0); // in front ppl enter in cm
    config["ignore_inspection"] = ignore_inspection_on;
    config["data_collection"] = data_collection_enabled;
    config["total_meters_run"] = roll_info_total_meters;
    config["fully_ignore_disabled"] = fully_ignore_disabled_classes;
    config["top_bottom_panel_selected"] = top_bottom_panel_selected;
    config["total_num_defects"] = roll_info_total_defects;
    config["is_roll_started"] = is_roll_started.load();
    config["is_light_on"] = !is_light_off;
    config["stop_at_last_defect"] = get_last_defect_in_panel;
    config["cutting_mode"] = cutting_mode;
    config["secondary_cut_lengths"] = json(secondary_cut_lengths).dump();
    config["cam2stopper_distance_mm"] = cam2stopper_distance;

    // This is being set by the Real World Map node who knows if the cutting machine plc is connected
    // Might need a refresh if the UI is loaded before this variable is set.
    ros::NodeHandle nh;
    bool cutting_machine_plc_connected = false;
    if (nh.getParam("cutting_machine_plc_connected", cutting_machine_plc_connected))
    {
        ROS_INFO("Cutting Machine PLC connection status: %s", cutting_machine_plc_connected ? "Connected" : "Not Connected");
    }
    config["cutting_machine_plc_connected"] = cutting_machine_plc_connected;
    config["just_cut_connections_available"] = just_cut_connections_available;
    config["Barcode"] = getBarcodeConfig().dump();
    config["WorkOrder"] = logger.get_pending_work_orders().dump();
    config["BatchCount"] = job.batch_count;
    config["layer_value"] = roll.material_type;
    config["roll_length"] = roll.roll_length;
    config["loom_number"] = roll.loom_number;
    config["work_order_id"] = job.work_order_id;

    // publish config.
    std_msgs::String send_msg;
    send_msg.data = config.dump();
    system_config_pub.publish(send_msg);
}

void WeavingInspection::bodyCounterCallback(const std_msgs::String &msg)
{
    try
    {
        json parsed_data = json::parse(msg.data);

        // Check if "cutting_info" exists and is an object
        if (!parsed_data.contains("cutting_info") || !parsed_data["cutting_info"].is_object())
        {
            ROS_WARN("[Main][bodyCounterCallback] Missing or invalid 'cutting_info': %s", msg.data.c_str());
            return;
        }

        const auto &cutting_info = parsed_data["cutting_info"];

        // Check if "body_counters" exists and is an array
        if (!cutting_info.contains("body_counters") || !cutting_info["body_counters"].is_array())
        {
            ROS_WARN("[Main][bodyCounterCallback] 'body_counters' key missing or not an array: %s", msg.data.c_str());
            return;
        }

        // Clear and update the vector
        body_counter_vector.clear();
        for (const auto &count : cutting_info["body_counters"])
        {
            if (count.is_number_integer())
            {
                body_counter_vector.push_back(count.get<int>());
            }
            else
            {
                ROS_WARN("[Main][bodyCounterCallback] Non-integer value in body_counters: %s", count.dump().c_str());
            }
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Main][bodyCounterCallback] JSON parse error: %s", e.what());
    }
}

/**
 * @brief Camera config callback to get the camera configuration from the different cameras.
 *
 * @param msg - Camera info message
 */
void WeavingInspection::cameraConfigCallback(const std_msgs::String &msg)
{
    json config = json::parse(msg.data);
    std::string cam_id = config["id"];
    ready_camera_ids.insert(cam_id);
    cam_config_map[cam_id] = config;
}

/**
 * @brief On Job Set/Reset Acknowledgement callback.
 *
 * @param msg - Acknowledgement message
 */
void WeavingInspection::setJobAckCallback(const std_msgs::String &msg)
{
    json ack = json::parse(msg.data);
    if (ack.contains("cam_id") && ack.contains("is_roll_started") && ack.contains("recipe"))
    {
        std::string cam_id = ack["cam_id"];
        if (!ready_camera_ids.size())
        {
            camera_job_set_ack_state = ack["is_roll_started"];
        }
        else
        {
            bool state = ack["is_roll_started"];
            camera_job_set_ack_state &= state;
        }

        ready_camera_ids.insert(cam_id);

        if (ready_camera_ids.size() >= number_of_cameras)
        {
            while (is_job_change_enabled)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            ready_camera_ids.clear();
            ack["is_roll_started"] = camera_job_set_ack_state & is_roll_started;
            std_msgs::String send_msg;
            send_msg.data = ack.dump();
            job_set_ack_pub.publish(send_msg);
        }
    }
    else
    {
        ROS_WARN("[Main][setJobAckCallback] Invalid Ack: %s", msg.data.c_str());
    }
}

/**
 * @brief Callback to update the that we have to consider inspection or not.
 *
 * @param msg - ignore inspection status
 */
void WeavingInspection::ignoreInspectionCallback(const std_msgs::Bool &msg)
{
    ignore_inspection_on = msg.data;
    json metadata;
    metadata["type"] = "real_world_metadata";
    metadata["ignore_inspection_on"] = ignore_inspection_on;
    std_msgs::String meta_msg;
    meta_msg.data = metadata.dump();
    metadata_pub.publish(meta_msg);
    addSystemLog("INFO", ignore_inspection_on ? "Ignore Inspection ON" : "Ignore Inspection OFF", "APP001");
}

/**
 * @brief Callback to update that we have to collect the data or not.
 *
 * @param msg - data collection status
 */
void WeavingInspection::dataCollectionCallback(const std_msgs::Bool &msg)
{
    data_collection_enabled = msg.data;
    sendNotification(data_collection_enabled ? "Data Collection is ON" : "Data Collection is OFF", "info");
    addSystemLog("INFO", data_collection_enabled ? "Data Collection is ON" : "Data Collection is OFF", "APP001");
}

/**
 * @brief Callback to update to stop at last defect or every defect.
 *
 * @param msg - stop at last defect status
 */
void WeavingInspection::stopAtLastDefectToggleCallback(const std_msgs::Bool &msg)
{
    get_last_defect_in_panel = msg.data;
    store->setParam("stop_at_last_defect", get_last_defect_in_panel);
    json metadata;
    metadata["type"] = "real_world_metadata";
    metadata["stop_at_last_defect"] = get_last_defect_in_panel;
    std_msgs::String meta_msg;
    meta_msg.data = metadata.dump();
    metadata_pub.publish(meta_msg);
}

/**
 * @brief Light toggle callback
 *
 * @param msg - light status
 */
void WeavingInspection::lightToggleCallback(const std_msgs::Bool &msg)
{
    is_light_off = !msg.data;
    json metadata;
    metadata["type"] = "plc_metadata";
    metadata["is_light_off"] = is_light_off;
    std_msgs::String meta_msg;
    meta_msg.data = metadata.dump();
    metadata_pub.publish(meta_msg);
}

void WeavingInspection::setBarcodeCallback(const std_msgs::String &msg)
{

    // check configuration table exist
    if (!logger.check_configuration_table_exists())
    {
        sendNotification("Failed configuration table does not exist.", "alert");
        return;
    }

    // Save Barcode configuration

    json app_config = json::object();
    app_config["App"] = {
        {"Barcode", json::parse(msg.data)}};

    if (logger.add_system_configuration("AppConfig", getAppID(), LogComponent::APP, app_config, project_id))
    {
        sendNotification("Barcode Configuration Saved!", "success");
        addSystemLog("ACTIVITY", "Barcode Configuration Saved: " + msg.data, "INFO018");
    }
    else
    {
        sendNotification("Saving Failed!", "alert");
    }

    // sync configuration
    systemConfigSyncCallback(std_msgs::Empty());
}

void WeavingInspection::setWorkOrderCallback(const std_msgs::String &msg)
{

    try
    {
        json workOrder = json::parse(msg.data);
        if (workOrder.contains("action"))
        {
            if (workOrder["action"] == "create")
            {
                // check configuration table exist
                if (!logger.check_kwis_work_order_table_exists())
                {
                    sendNotification("Failed: Work order table does not exist.", "alert");
                    return;
                }

                // Convert cut length to mm
                json data = workOrder["data"];
                int cut_length = data.at("cut_length").get<int>() * 10;
                std::string name = data["name"];

                // Check if work order with same name and cut length already exists
                if (logger.check_work_order_exists(name, cut_length))
                {
                    std::string msg = "Work Order with name '" + name + "' and cut length " + std::to_string(cut_length) + "mm already exists!";
                    sendNotification(msg, "alert");
                    return;
                }

                // Insert new work order
                if (logger.insert_work_order_log(name, cut_length, data["target_pcs"], 0, 0, 0, 0))

                {
                    sendNotification("Work Order Configuration Saved!", "success");
                    addSystemLog("ACTIVITY", "Work Order Configuration Saved: workOrder name: " + name, "APP001");
                }
                else
                {
                    sendNotification("Saving Failed!", "alert");
                }
            }
            else if (workOrder["action"] == "complete")
            {
                json data = workOrder["data"];
                int64_t id = std::stoll(data["work_order_id"].get<std::string>());

                if (logger.update_work_order_status(id, 1))
                {
                    sendNotification("Work Order ID " + std::to_string(id) + " marked as complete.", "success");
                }
                else
                {
                    sendNotification("Failed to update Work Order status for ID " + id, "alert");
                }
            }
        }

        // sync configuration
        systemConfigSyncCallback(std_msgs::Empty());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught while handling work order message: " << e.what() << "\n";
        sendNotification("Invalid work order data!", "alert");
    }
}

void WeavingInspection::versionCallback(const std_msgs::String &msg)
{
    json message = json::parse(msg.data);
    version = message["version"];
}

void WeavingInspection::rollContinueAckcallback(const std_msgs::Int8 &msg)
{
    roll_continue_ack = msg.data;
}
/**
 * @brief Retrieves the Barcode Configuration from the system configuration.
 *
 * First checks if "AppConfig" exists and contains a "Barcode" entry under "App".
 * If not found, falls back to "BarcodeConfig" directly.
 *
 * @return JSON object of Barcode configuration or empty if not found.
 */
json WeavingInspection::getBarcodeConfig()
{
    json app_config = logger.get_system_configuration(getAppID(), "AppConfig");

    if (!app_config.empty() && app_config.contains("App"))
    {
        json app_section = app_config["App"];
        if (app_section.contains("Barcode"))
        {
            return app_section["Barcode"];
        }
    }

    // Fallback: Check if "BarcodeConfig" is directly stored
    json barcode_config = logger.get_system_configuration(getAppID(), "BarcodeConfig");
    if (!barcode_config.empty())
    {
        return barcode_config;
    }

    std::cerr << "[WARN] Barcode configuration not found in AppConfig or BarcodeConfig.\n";
    return json::object(); // empty JSON
}
