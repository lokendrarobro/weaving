/**
 * @file realWorldMap.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Map all panel positions and defects with the world map.
 * @version 1.0
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 */

#ifndef REALWORLDMAP_HPP
#define REALWORLDMAP_HPP

#include <mutex>
#include <cmath>
#include <thread>
#include <json.hpp>
#include <ros/ros.h>
#include <systemState.hpp>

#include <queue>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/Detection2D.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <robroImageSaver.h>

#include <body.hpp>
#include <defect.hpp>
#include <plcComm.hpp>
#include <bodyTowerLight.hpp>
#include <mysqlClient.hpp>
#include <worldMapDrawer.hpp>
#include <worldMapDefect.hpp>
#include <executionTimer.hpp>
#include <inductiveSensor.hpp>
#include <dancerSensor.hpp>
#include <cutterSensor.hpp>
#include <cuttingManager.hpp>
#include <buzzerControl.hpp>
#include <intervalTask.hpp>
#include <errorTracker.hpp>
#include <workOrder.hpp>
#include <batchCount.hpp>
#include <bodyMeter.hpp>
#include <componentStateTrackerLogger.hpp>
#include <defectsProcessor.hpp>
#include <recipeJson.hpp>
#include <speedCalculator.hpp>

#include <weaving_inspection/GetPulseDistance.h>
#include <weaving_inspection/GetCurrentPulses.h>
#include <weaving_inspection/StoppingDefectImages.h>
#include <weaving_inspection/InsertDataLog.h>

using json = nlohmann::json;

/**
 * @brief class to map all panel positions and defects with the real world map.
 */
class RealWorldMapNode
{

    /**************************************************/
    /****CUSTOMER MACHINE RELATED PARAMETER*****/
    /**************************************************/

    /**************************************************/
    /****KWIS MECHANICALS/ELECTRICALS RELATED PARAMETER*****/
    /**************************************************/



    bool system_installed_in_front_of_dancer{false}; // KWIS doesn't need to recalculate cam2stopper if installed in front of dancer

    int distance_from_bottom_to_top_camera_mm{0}; // Offset between bottom and top camera position

    int frame_height{4096}; // height of frame the camera captures
    int frame_width{1024};  // width of frame the camera captures

    int body_log_checker{0}; // cheking for body log

    int defected_body_log_checker{0}; // checking for defected body log

    int is_body_planed_checker{0};

    int64_t last_defect_body_id{0};

    int64_t last_inserted_body_id{0};

    int64_t last_inserted_system_id{0};

    int dancer_end2stopper_distance_mm{0}; // after what pos we stop handling dancer effect in defect distance calc.

    int const_original_cam2stopper_mm{0}; // camera to cutter distance without offset. Fixed.

    int total_distance_x_mm{0}; // distance covered by all cameras in x

    int number_of_cameras; // total number of cameras

    int group_id{0};

    float y_px_per_mm; // Resolution in Y Direction (along roll movement). Should be same for all cameras.

    float pulse_per_mm{0.0f}; // Pulse per mm in the camera. Should be same for all cameras.

    float running_meter{0.0f}; // fabric running meter

    double machine_speed_m_per_min{0.0}; // machine speed computed from cutter encoder
    SpeedCalculator speed_calculator;

    std::string buzzer_at_defect; //start:if you want buzzer at start of defect or end:if you want buzzer at the end of the defect

    int stop_time_after_defect{2}; //for how many seconds you want to stop the machine after defect

    std::map<std::string, json> cam_config_map; // Cameras, their positions, FOVs and IDs
    bool dancer_control_on_just_cut_mode{true};
    bool spout_and_cutter_control_on_just_cut_mode{false};

    /**************************************************/
    /****KWIS SETTING PARAMS*****/
    /**************************************************/

    std::atomic<bool> is_light_toggle_requested{false}; // UI : User asks to toggle light state

    std::atomic<bool> is_immediate_stopping_requested{false}; // For Error/Alert: Request to Stop Immediately

    std::atomic<bool> is_cut_length_updated{false}; // Have we received a new cut-length value?

    std::set<std::string> ready_camera_ids; // Are all cameras ready?

    std::atomic<bool> is_reset_requested{false}; // Store to reset the system or not requested by the main node of the system.

    bool is_light_off{true}; // Store if light is on or not by  the user.

    /**************************************************/
    /****KWIS GENERAL RUNNING PARAMS*****/
    /**************************************************/

    bool check_for_stopping{false};           // when we write a command to stop, we check if we've stopped.
    std::atomic<bool> is_stopping_done{true}; // now we know, we've stopped.

    bool defect_present_in_last_body{false}; // For tracking if last body contain defect or not in semi auto and just cut mode
    bool defect_present_within_cut_length{false};      // For Punch block of full body
    bool defect_present_within_half_cut_length{false}; // For punch block of half body

    bool is_defect_available_for_next_stopping{false}; // Is there a defect available to stop for?

    bool is_mode_top_bottom_and_defect_passed{false}; // Is there a defect available to stop for?

    bool punching_block_state{false}; // Is KWIS Blocking the punch currently

    std::atomic<bool> is_roll_started{false}; // Reflection of UI selection of "Start Roll"

    bool is_stopping_window_open{false}; // UI: Modal in front shown during stopping is open or not

    std::atomic<int> stopping_defect_frame_id{-1};        // Which frame ID did we stop for
    uint64_t total_processed_body_mm_after_last_stopping; // how much mm we moved since we stopped (for closing modal)

    int last_loop_close_fabric_mm_diff{0}; // What was the last loop close value

    uint64_t previous_body_processed_mm{0}; // body processed in last run of control loop

    uint64_t previous_body_processed_before_reset{0};

    bool defect_present_within_cut_length_before_reset{false};

    uint64_t current_cam2stopper_distance_mm{0}; // Dynamic Cam2Stopper

    uint64_t cam_last_loop_close_pulse{0}; // Camera Pulses When last loop close event happened

    uint64_t prev_fabric_moved_under_camera_mm{0}; // fabric under camera value at last run of control loop
    uint64_t prev_fabric_move_under_cutter{0};     // fabric under cutter value at last run of control loop

    std::vector<WorldMapDefect> new_master_defects; // all defects stored

    std::vector<int> panel_positions; // Dynamic positions of the panels that'll be cut.

    std::queue<vision_msgs::Detection2DArray> combine_defects; // combined defects of all cameras coming from weaving main node

    WorldMapDefect defect_for_next_stopping; // What defect are we gonna stop for next?

    /**************************************************/

    /**************************************************/
    /****CUTTING MODE RELATED PARAMS*****/
    /**************************************************/

    int auto_cut_defect_offset_mm{0}; // safety bracket around defect in auto-cutting mode

    bool auto_cut_debug_in_semi_auto_mode{false}; // Show auto-cut stuff on map even in semi-auto mode.

    std::string cutting_mode{"semiauto"};

    bool is_auto_cut_modal_open{false};

    /**************************************************/
    /****USER PREFERENCES RELATED PARAMETER (FIXED / LONG TERM)*****/
    /**************************************************/

    bool get_last_defect_in_panel{false}; // if true, we look for the last defect in panel for stopping

    bool block_punching_for_defect_behind_punch{true}; // punch block to be done for half body or full body.

    bool spout_control_via_cutting_machine_com{false}; // set the spout status once based on user preference in roll start.

    bool use_cutter_sensor_instead_machine_ready{false}; // use the cutter sensor instead of machine ready signal.


    bool is_cycle_start_electrical{false}; // use the electrical instead of register for cycle start


    bool bypass_cutter_sensor_once{false}; // bypass the cutter sensor once in cutter based control mode.

    /**************************************************/

    /**************************************************/
    /****CURRENT JOB RELATED PARAMETER (TEMP / FOR THIS JOB)*****/
    /**************************************************/

    int cut_length_mm{2400};         // Current cut-length
    int updated_cut_length_mm{2400}; // If we've received an updated cut-length

    bool batchCountNotificationSend{false};

    std::atomic<bool> ignore_inspection_on{false}; // Are we not stopping & ignoring defects? All other stuff still works.

    bool roll_info_punching_on{true}; // Are we doing punching in this roll?

    bool top_bottom_panel_selected{false};

    int dancer_end2stopper_distance_mm_with_offset{0}; // add offset to dancer_end2stopper_distance_mm

    int original_cam2stopper_mm{0}; // May change with offset , need to check.

    int offset_cm{0}; // How much extra fabric in the current fabric flow. Fabric offset value for this job.

    int goodfabric{0};
    int defectedfabric{0};

    int64_t roll_id{0}; // This roll ID

    int64_t job_id{0}; // This Job ID

    int64_t body_id{0};

    int64_t work_order_id{0};

    std::string project_id{""}; // Project ID specified in the launch file

    float fabric_gsm{0}; // This roll gsm
    float fabric_width{0}; // This roll fabric width

    /**************************************************/

    /**************************************************/
    /****RECIPE RELATED PARAMS*****/
    /**************************************************/

    json current_recipe_json; // all stuff about the current recipe

    std::vector<int> disabled_class_ids; // It stores the class_ids of disabled classes

    std::string this_dataset_id;

    int64_t dataset_id{0}; // name of the dataset for this roll

    /**************************************************/
    /****REPORTING RELATED PARAMS*****/
    /**************************************************/

    bool is_prev_punch_blocked{false}; // did we block punch for previous body? need to log.

    long running_meter_at_cutter{0}; // what's the running meter at cutter? for logging at body cut.

    std::string report_image_save_path; // Full path (generally "weaving/gui/images") where cropped defect images are stored.

    std::string reporting_folder_name; // name of folder (generally "/images") where cropped defect images are stored.

    RobroImageSaver *report_image_saver; // Saver object for storing cropped defect images.

    RobroImageSaver *full_image_saver; // Saver object for storing defect map images.

    Body body_log; // Body object to store the body information and publish the body log.
    BodyMeter body_meter_tracker; // meter object to store the meter;

    WorkOrder current_work_order; // Object to store the current work order selected;

    BatchCount current_batch_count; // Object to store the current batch count selected;

    IntervalTask interval_task_pulse_error_logger;

    DefectsProcessor dp;   //for logging the sensitivity in database 


    /**
     * @brief Inspection Node following messages to publish to the GUI.
     */
    std_msgs::String goodfabric_publish_msg
    ,defectedfabric_publish_msg;

    /**************************************************/

    /**************************************************/
    /****PUBLISHERS*****/
    /**************************************************/
    ros::Publisher notification_pub,
        goodfabric_pub,
        defectedfabric_pub,
        system_config_pub,
        stopped_defect_image_pub,
        defect_log_pub,
        next_stopping_pub,
        body_log_pub,
        stopping_window_close_pub,
        camera_config_sync_pub,
        ai_cut_master_data_pub,
        loading_screen_data_and_status_pub,
        ai_cut_master_modal_connection_status_pub,
        ai_cut_master_modal_machine_ready_pub,
        dancer_sensor_on_pub,
        system_sync_pub;

    image_transport::Publisher defect_map_image_pub;

    /**************************************************/
    /****SUBSCRIBERS*****/
    /**************************************************/

    ros::Subscriber camera_config_sub,
        job_set_ack_sub,
        metadata_change_sub,
        reset_system_sub,
        gui_sync_sub,
        cycle_start_button_sub,
        show_last_popub_sub,
        write_cut_len_sub,
        ai_cut_master_modal_open_sub,
        logger_disconnected_state_sub, // Added
        running_meter_sub,             // Added
        combine_defects_sub;

    /**************************************************/
    /****SERVICES*****/
    /**************************************************/

    ros::ServiceClient cam_pulse_client,  // for getting distance from current camera pulse
         get_last_id_client, // for getting the last id of the body
        cam_last_loop_close_pulse_client; // for getting last loop close pulse value

    // for getting the defect images of the stopping panel
    weaving_inspection::StoppingDefectImages defect_pop_msg;

    /**************************************************/
    /****INTERFACES*****/
    /**************************************************/

    PLCComm plcComm; // PLCComm object to communicate with the PLC, and call the PLC services.

    BodyTowerLight  *body_tower_light;

    WorldMapDrawer mapDrawer; // DrawWorldMap object to draw the world map and update the world map.

    CuttingManager cuttingManager; // object to manage and plan the cutting action.

    DancerSensor dancer_sensor;

    CutterSensor cutter_sensor;

    BuzzerControl buzzerControl;

    /**************************************************/
    /****DIAGNOSTICS*****/
    /**************************************************/

    /**
     * @brief system logger object. Which is used to log the messages.
     */
    MySQLClient &logger = MySQLClient::getInstance();

    std::string component_name{"RealWorldMap"}; // Component name for the system state tracker.
    componentStateTrackerLogger *sst;

    /**************************************************/
    /****ERROR TRACKER PARAMS *****/
    /**************************************************/
    double error_popup_threshold_m{5.0}; // meters
    double error_stop_threshold_m{20.0}; // meters

    /**
     * @brief Checks if the logger is disconnected due to retry limits being exceeded.
     *
     * This function checks if the logger's state indicates that it has been disconnected
     * after exceeding the maximum number of retries. It returns true if the state is
     * 'DISCONNECTED_RETRIES_EXCEEDED', otherwise returns false.
     *
     * @return true if the logger is disconnected, false otherwise.
     **/
    inline bool isLoggerDisconnected() const
    {
        return logger.getState() == MySQLClientState::DISCONNECTED_RETRIES_EXCEEDED;
    }

public:
    /**
     * @brief Construct a new Real World Map object
     */
    RealWorldMapNode();

    /**
     * @brief Destroy the Real World Map object
     */
    ~RealWorldMapNode();

    /**
     * @brief Function to read the ROS parameters.
     *
     * @return true - If the parameters are read successfully.
     * @return false - If the parameters are not read successfully.
     */
    bool readROSParams();

    /**
     * @brief Initialize the camera node parameters.
     */
    void initParams();

    /**
     * @brief Function to initialize the ros publishers and subscribers.
     */
    void initPubSub();

    /**
     * @brief Function to initialize the ros client.
     */
    void initClient();

    /**
     * @brief Function will be waiting until all the cameras are ready.
     * it uses camera configuration publisher and subscriber
     * to check the camera status.
     */
    bool waitUntilCamerasAreReady();

    /**
     * @brief: Update the positions of the panel and add a new one if needed
     *
     * @param panel_move_forward_mm - The distance the panel has moved forward
     */
    void updatePanelPositions(int panel_move_forward_mm);

    /**
     * @brief Update the world map with the new defects and camera positions and calculate the
     * punch block state, and defect for next stopping.
     *
     * @param move_forward_mm - The distance the panel has moved forward
     */
    void updateRealWorld(int move_forward_mm);

    /**
     * @brief Record the defects on the world map as well as add the defects to the list of defects.
     *
     * @param detections - list of detections
     */
    void recordDefects(vision_msgs::Detection2DArray &detections);

    /**
     * @brief Record the cutter action. This will update the cut panel defects and
     * panel at cutter defects in sorted order. As well as update the panel positions.
     */
    void recordCutterAction(bool stopping_is_underway);

    /**
     * @brief Record the stopping action. This will update the cut panel defects and
     * panel at cutter defects in sorted order. As well as update the panel positions.
     */
    void recordStoppingAction(int stopped_for_seq_id);

    /**
     * @brief update the panel positions.
     */
    void resetPanelsPos();

    /**
     * @brief Reset the world map. This will release all the vectors and maps.
     */
    void resetWorldMap();

    /**
     * @brief Get the Process Body Map visualization image
     *
     * @return cv::Mat - process body map
     */
    cv::Mat getProcessedBodyMap(int stopped_for_seq_id);

    /**
     * @brief Get the Last Defect In Processing panel.
     *
     * @param defect - last defect in processing panel
     * @return true - if there is a defect in processing panel
     * @return false - if there is no defect in processing panel
     */
    bool getLastDefectInProcessPanel(vision_msgs::Detection2D &defect);

    /**
     * @brief Get the First Defect In Process Panel.
     *
     * @param defect - first defect in processing panel.
     * @return true - if there is a defect in processing panel
     * @return false - if there is no defect in processing panel
     */
    bool getFirstDefectInProcessPanel(vision_msgs::Detection2D &defect);

    /**
     * @brief Get stopping panel defects images from the processing panel.
     *
     * @param stopped_for_seq_id - stopped for sequence id
     * @return std::vector<sensor_msgs::Image> - list of images of defects in processing panel.
     */
    std::vector<sensor_msgs::Image> getStoppingPanelDefectsImages(int stopped_for_seq_id);

    /**
     * @brief Get stopping panel defects images from the processing panel.
     *
     * @param next_cut_len
     * @return std::vector<sensor_msgs::Image> - list of images of defects in processing panel.
     */
    std::vector<sensor_msgs::Image> getAutoCutNextBodyDefectImages(int next_cut_len);

    /**
     * @brief Get the world map camera names
     *
     * @return json - list of camera names
     */
    json getWorldMapCamNames();

    /**
     * @brief Send notification to the UI.
     *
     * @param msg - Message to be sent.
     * @param severity - Severity of the message.
     */
    void sendNotification(std::string msg, std::string severity);

    /**
     * @brief Log body info to reporting node using publisher.
     */
    void logBodyInfo();

    /**
     * @brief Process the cutting action and update the cutter status.
     *
     */
    void resetValuesAtCuttingAction();

    /**
     * @brief Get the Distance Move By Current Panel in mm in Real World.
     *
     * @param current_body_processed_mm - current body processed in mm.
     * @return int64_t - distance moved by current panel.
     */
    int64_t getMoveForwardMM(uint64_t current_body_processed_mm);

    /**
     * @brief Estimate the camera to stopper distance in real world.
     *
     * @param current_fabric_moved_under_cutter_mm - current fabric moved under cutter in mm.
     * @param current_fabric_moved_under_camera_mm - current fabric moved under camera in mm.
     *
     */
    void updateCurrentCam2StopperDistance(uint64_t current_fabric_moved_under_cutter_mm, uint64_t current_fabric_moved_under_camera_mm);

    /**
     * @brief Process the defects and update the world map.This function is contains infinite loop which will
     * be running in main thread and do all service calling from PLC and update the world map.
     */
    void processAndUpdateWorldMap();

    /**
     * @brief Get the Time in string format of YYYY-MM-DD HH:MM:SS
     *
     * @return std::string - Time in string format of YYYY-MM-DD HH:MM:SS
     */
    std::string getTimeStr();

    /**
     * @brief gives the time difference in milliseconds between the given time and current time.
     *
     * @param point - time point to compare with current time.
     * @return int - time difference in milliseconds.
     */
    int timeDiffFromCurrentTime(std::chrono::high_resolution_clock::time_point point);

    /**
     * @brief Returns the distance between given defect and the fabric currently under camera
     * in mm.
     *
     * @param current_frame_seq - current frame sequence number.
     * @return double - distance between given defect and the fabric currently under camera in mm
     */
    double getDistanceFromCameraMM(int current_frame_seq);

    /**
     * @brief Gets the difference of pulses in Primary Camera Counter since last time
     * loop was closed (dancer sensor was turned on).
     * If values are negative (due to small backlash) - just send 0.
     * @param uint64_t 0 if backlash, else actual value
     *
     * @return Returns `true` on success, otherwise `false`.
     */
    bool getCameraPulsesFromLastLoopClose(uint64_t &value);

    /**
     * @brief Add the system log to into system log server.
     *
     * @param severity - Severity of the log.
     * @param msg - Message to be logged.
     */
    void addSystemLog(std::string severity, std::string msg, std::string msg_code);

    float goodfabriclength{0.0};
    float defectedfabriclength{0.0};


    RecipeJSON::Recipe recipe_json; // Recipe JSON object to store the current recipe information.
    json current_defect_frame_json; // Current defect JSON object to store the current defect information.

    /**
     * @brief  Function to subscribe to the topics and work as a callback function are as follows.
     * 1. metadataChangeCallback - To subscribe the metadata change from main node.
     * 2. combineDefectsCallback - To subscribe the combine defects from main node.
     * 3. resetWorldMapCallback - To subscribe the reset system from main node.
     * 4. recordCameraPositionCallback - To subscribe the camera config from camera node.
     * 5. updateCurrentRecipeCallback - To subscribe to update the current recipe from camera node.
     * 6. showLastPopupCallback - To subscribe to show the last popup from GUI and publish the last popup info.
     */
    void metadataChangeCallback(const std_msgs::String &msg);
    void combineDefectsCallback(const vision_msgs::Detection2DArray &msg);
    void resetWorldMapCallback(const std_msgs::Empty &msg);
    void showLastPopupCallback(const std_msgs::Empty &msg);
    void recordCameraPositionCallback(const std_msgs::String &msg);
    void updateCurrentRecipeCallback(const std_msgs::String &msg);
    void guiSyncCallback(const std_msgs::Empty &msg);
    void cycleStartCallback(const std_msgs::Empty &msg);
    void writeCutLengthCallback(const std_msgs::Int16 &msg);
    void aiCutMasterModalOpenCallback(const std_msgs::Bool &msg);
    void loggerDisconnectedCallback(const std_msgs::Empty &msg); // Added
    void runningMeterCallback(const std_msgs::String &msg);      // Added

    // Function to set cycle start
    bool setCycleStart(bool state);

};

#endif // WORLDMAP_HPP