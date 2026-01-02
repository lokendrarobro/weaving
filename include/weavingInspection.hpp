/**
 * @file weavingInspection.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Weaving Inspection Blue print for KWIS
 * @version 1.0
 * @date 2022-03-03
 * @updated 2022-08-26
 *
 * @copyright Copyright (c) 2022
 *
 */

// Including Necessary Libraries.
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <json.hpp>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <thread>
#include <sstream>
#include <mutex>
#include <string>
#include <queue>
#include <paramStore.hpp>
#include <fstream>
#include <boost/filesystem.hpp>

// Custom Class
#include <mysqlClient.hpp>
#include <roll.hpp>
#include <job.hpp>
#include <intervalTask.hpp>
#include <logMessage.hpp>
#include <robroImageSaver.h>
/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Application core class for KWIS. Which is used to process the defects
 * and publish the results. It Also process the stopping and punching.
 */
class WeavingInspection
{
    ros::ServiceClient client,get_roll_data_client; 

    /**
     * @brief It indicates that light is on or off.
     */
    bool is_light_off{false};

    /**
     * @brief Indicates if the system is running or not.
     */
    std::atomic<bool> is_roll_started{false};

    /**
     * @brief Indicates if punching is enabled or not for the current roll by the operator.
     */
    bool roll_info_punching_on{true};

    bool top_bottom_panel_selected{false};

    /**
     * @brief It Indicates if inspection needes to be ignored or not.
     */
    bool ignore_inspection_on{false};

    /**
     * @brief It Indicates if the data collection is enabled for image or not.
     */
    bool data_collection_enabled{false};

    /**
     * @brief Use to combine the all cameras acknowledged state into one.
     */
    bool camera_job_set_ack_state{false};

    /**
     * @brief Use for acknowledgement purpose during job change point.
     */
    bool is_job_change_enabled{false};

    /**
     * @brief It indicates whether system requires the last defect in panel or first defect in panel
     * for stopping action.
     */
    bool get_last_defect_in_panel{false};

    /**
     * @brief Store the height of the single frame captured by the camera.
     * It is used to calculate the distance between defects.
     */
    int frame_height{4096};

    /**
     * @brief Store the width of the single frame captured by the camera.
     */
    int frame_width{1024};

    /**
     * @brief The number of cameras connected to the system.
     * Used to wait for data from all cameras.
     */
    int number_of_cameras;

    /** Indicate the ack received from GUI for roll continue */
    int roll_continue_ack{-1};

    /**
     * @brief The distance between stopper/cutter & camera when dancer sensor is on.
     * used to calculate the distance between defects.
     */
    int original_cam2Stopper_mm;

    /**
     * @brief The distance between stopper/cutter & camera when dancer sensor is on.
     * used to calculate the distance between defects. It is camera to stopper distance with offset.
     */
    int cam2stopper_distance;

    /**
     * @brief GSM value of the current roll.
     *
     */
    int gsm{0};

    int64_t last_inserted_roll_id{0};
    int64_t last_inserted_job_id{0};
    std::vector<int> body_counter_vector;

    /**
     * @brief Total number of defects in the current roll.
     * It is used to calculate the percentage of defects in the roll.
     */
    int roll_info_total_defects{0};

    /**
     * @brief Offset in cm. It is used to calculate the camera to stopper distance
     * with given offset.
     */
    int offset_cm{0};

    /**
     * @brief Current frame sequence number. Which indicates the current frame
     * which processed by the system for defect processing.
     */
    int current_frame_seq{0};

    /**
     * @brief It store the cut length of the roll in mm.
     * It is used to calculate length of panel cut in current roll.
     */
    int cut_length_mm{2400};
    std::vector<int> secondary_cut_lengths;
    /**
     * @brief Number of ng in the current roll. which is similar to @p roll_info_total_defects.
     * but used publish the ng count to the GUI.
     */
    int ng_count = 0;

    /**
     * @brief Pixel per mm for the camera. Which is used to cover co-ordinates
     * from image world to real world.
     */
    float pixel_per_mm{12.2f};

    /**
     * @brief Pulse per mm for the camera. Which is used to covert the distance
     * from camera encoder pulses to real world.
     */
    float pulse_per_mm{0.0f};

    /**
     * @brief Running meter of the roll. Which is used to publish the running
     * meter to the GUI.
     */
    float running_meter = 0.0;

    /**
     * @brief Stores the mm per frame. Which is used to calculate the distance
     * for roll info.
     */
    float mm_per_frame;

    /**
     * @brief total meters of the roll. Which use to log the roll data.
     */
    float roll_info_total_meters{0.0f};

    std::string cutting_mode{"semiauto"};
    /**
    /**
     * @brief Stores the material type of roll. Which is used to generate the report.
     */
    std::string material_type{"L"};

    /**
     * @brief Project Id used to maintain the unique identifier for
     * the system.
     */
    std::string project_id;

    /**
     * @brief Project base path. Which is used to store the project related
     * data.
     */
    std::string project_base_path;

    /**
     *  @brief Version of application
     */
    std::string version{""};

    /**
     * @brief All camera detection map. This is used to store the detections
     * from all cameras and process them.It stores the camera id as key and
     * the queue of detections as value.
     */
    std::map<std::string, std::queue<vision_msgs::Detection2DArray>> all_camera_detection_map;

    /**
     * @brief This is used to store number of defects received from all cameras.
     * It stores the camera id as key and the number of defects as value.
     */
    std::map<std::string, int> num_defects_recd_map;

    /**
     * @brief Stores the camera config info. This is used to draw the camera positions on the world map.
     * The key is the camera_id and the value is the json containing the camera config info.
     */
    std::map<std::string, json> cam_config_map;

    /**
     * @brief It stores the camera id as key and the queue of images and seq no.
     * as value. This is used to store the images from all cameras and process.
     */
    std::map<std::string, std::queue<std::pair<int, cv::Mat>>> all_camera_image_map;

    /**
     * @brief Number of images received from all cameras. It stores the camera id as key
     * and the number of images as value.
     */
    std::map<std::string, int> num_imgs_recd_map;

    /**
     * @brief camera ids. Which is used to store the camera ids. It is use to track
     * the number of cameras are ready.Until all cameras are ready, the system will
     * not start and call for camera config.
     */
    std::set<std::string> ready_camera_ids;

    /**
     * @brief system logger object. Which is used to log the messages.
     */
    MySQLClient &logger = MySQLClient::getInstance();

    /**
     * @brief Roll object. Which is used to store the roll info as well as used to
     * publish the roll info to reporting.
     */
    Roll roll;

    /**
     * @brief Job object. Which is used to store the job info as well as used to
     * publish the job info to reporting.
     */
    Job job;

    /**
     * @brief Param store object. Which is used to store the parameters.
     */
    ParamStore *store;

    /**
     * @brief Inspection Node following messages to publish to the GUI.
     * 1. system_health_msg - To publish the system health message.
     * 2. notification_msg - To publish the notification message.
     * 3. running_meter_publish_msg - To publish the running meter.
     * 4. ng_count_publish_msg - To publish the ng count.
     */
    std_msgs::String system_health_msg,
        notification_msg,
        running_meter_publish_msg,
        ng_count_publish_msg;

    /**
     * @brief Inspection Node publisher to publish following topics.
     * 1. notification - To publish the notification message.
     * 2. system_health_msg - To publish the system health message.
     * 3. inspection_category - To publish the inspection category.
     * 4. running_meter - To publish the running meter.
     * 5. ng_count - To publish the ng count.
     * 6. system_config - To publish the system config.
     * 7. camera_config_sync - To publish the camera config sync.
     * 8. roll_log - To publish the roll log.
     * 9. job_log - To publish the job log.
     * 10. job_set_ack - To publish the job set ack.
     * 11. combine_defects - To publish the combine defects.
     * 12. reset_system - To publish the reset system.
     * 13. metadata - To publish the metadata to other nodes.
     * 14. version_pub - To publish the version
     */
    ros::Publisher notification_pub,
        system_health_msg_pub,
        inspection_category_pub,
        running_meter_pub,
        ng_count_pub,
        system_config_pub,
        camera_config_sync_pub,
        roll_log_pub,
        job_log_pub,
        roll_continue_ack_gui_pub,
        job_set_ack_pub,
        combine_defects_pub,
        reset_system_pub,
        version_pub,
        metadata_pub,
        cam_metadata_pub;

    /**
     * @brief Inspection Node publisher to publish following Image topics.
     * 1. inspected_image - To publish the inspected image.
     * 2. defected_image - To publish the defected image.
     */
    image_transport::Publisher inspected_image_pub,
        defected_image_pub;

    /**
     * @brief Subscriber to subscribe following topics.
     * 1. stop_at_last_defect_toggle - To subscribe the stop at last defect toggle.
     * 2. cam_annotated_image - To subscribe the cam annotated image.
     * 3. defect - To subscribe the defect from the camera.
     * 4. material_type - To subscribe the material type.
     * 5. metadata_change - To subscribe the metadata change.
     * 6. pixel_per_mm - To subscribe the pixel per mm.
     * 7. pulse_per_mm - To subscribe the pulse per mm.
     * 8. reset_system - To subscribe the reset system.
     * 9. restart - To subscribe the restart.
     * 10. system_config_sync - To subscribe the system config sync.
     * 11. camera_config - To subscribe the camera config.
     * 12. ignore_inspection - To subscribe the ignore inspection toggle.
     * 13. light_toggle - To subscribe the light toggle.
     * 14. data_collection - To subscribe the data collection.
     * 15. job_set_ack - To subscribe the job set ack.
     */
    ros::Subscriber stop_at_last_defect_toggle_sub,
        cam_annotated_image_sub,
        defect_sub,
        material_type_sub,
        metadata_change_sub,
        pixel_per_mm_sub,
        pulse_per_mm_sub,
        reset_system_sub,
        restart_sub,
        system_config_sync_sub,
        camera_config_sub,
        ignore_inspection_sub,
        light_toggle_sub,
        data_collection_sub,
        job_set_ack_sub,
        barcode_config_sub, // Added
        work_order_config_sub,     // Added
        version_value_sub,  // Added
        body_counter_sub,
        roll_continue_ack_sub ;

    bool fully_ignore_disabled_classes{false};

    IntervalTask interval_task_main_q_size_logger;

    bool just_cut_connections_available{false};

    /**
     * @brief Function to generate configuration id using current time.
     */
    std::string generateConfigurationIDString();

    /**
     * @brief Function to generate Work Order ID using current time.
     */
    std::string generateWorkOrderIDString();

    std::string report_image_save_path; // Full path (generally "weaving/gui/images") where cropped defect images are stored.

public:
    /**
     * @brief Construct a new Weaving Inspection object
     */
    WeavingInspection();

    /**
     * @brief Destroy the Weaving Inspection object
     *
     */
    ~WeavingInspection();

    /**
     * @brief It reads the parameters from the ros parameter server and
     * stores them in the class variables.
     *
     * @return true - if parameters are read successfully.
     * @return false - otherwise.
     */
    bool readROSParams();

    bool readROSParamsAndStoreInDB();

    /**
     * @brief Function to initialize the ros publishers and subscribers.
     */
    void initPubSub();

    /**
     * @brief Function to initialize the ros services.
     */
    void initServices();

    /**
     * @brief Function to initialize the system variables and threads.
     */
    void initSystem();

    /**
     * @brief Function will be waiting until all the cameras are ready.
     * it uses camera configuration publisher and subscriber
     * to check the camera status.
     */
    bool waitUntilCamerasAreReady();

        /**
     * @brief RobroImageSaver object to save the defect images.
     */
    RobroImageSaver *full_image_saver;
    

    /**
     * @brief store the dataset id of the roll
     */
    int64_t dataset_id{0};

    /**
     * @brief The function will wait until the version value is received.
     */
    void waitUntilVersionReceived();

    /**
     * @brief Get the Time in string format of YYYY-MM-DD HH:MM:SS
     *
     * @return std::string - Time in string format of YYYY-MM-DD HH:MM:SS
     */
    std::string getTimeStr();

    /**
     * @brief Checks if the images are available in the queue for processing.
     *
     * @return true - if images are available.
     * @return false - otherwise.
     */
    bool isImagesAvailable();

    /**
     * @brief Checks if the defects are available in the queue for processing.
     *
     * @return true - if defects are available.
     * @return false - otherwise.
     */
    bool isDetectionsAvailable();

    /**
     * @brief Add the system log to into system log server.
     *
     * @param severity - Severity of the log.
     * @param msg - Message to be logged.
     */
    void addSystemLog(std::string severity, std::string msg, std::string message_code);

    /**
     * @brief Prepare combined annotated image from all cameras.
     *
     * @param combined_annotated_image - Combined annotated image from all cameras.
     * @return  true - if the image is prepared successfully.
     * @return false - otherwise.
     */
    bool prepareCombineAnnotatedImage(cv::Mat &defected_image);

    /**
     * @brief Add the defects to the world map. Where they will be processed for stopping and punching.
     *
     * @param total_number_of_defects - Total number of defects to be added to the world map.
     * @return true - if the defects are added successfully.
     * @return false - otherwise.
     */
    bool addDefectsToWorldMap(int &total_number_of_defects);

    /**
     * @brief Publish the inspected image and remove from all camera image map.
     */
    void publishAndRemoveTheImage();

    /**
     * @brief The Function process the defects and publish the results. Function runs in a separate thread.
     * It extract the defects from the queue and process them. It also publishes the results to the PLC and the UI.
     */
    void processDefectsAndPublishResults();

    /**
     * @brief Send notification to the UI.
     *
     * @param msg - Message to be sent.
     * @param severity - Severity of the message.
     */
    void sendNotification(std::string msg, std::string severity);

    /**
     * @brief Function to call when roll starts.The function will log the
     * roll start time and update the roll info.
     * @param metadata - Roll metadata.
     */
    void onRollStart(json metadata);
    /**
     * @brief Function to call when roll starts.The function will log the
     * roll start time and update the roll info.
     * @param metadata - Roll metadata.
     */
    int primary_body_count{0};
    int secondary_body_count{0};
    int tertiary_body_count{0};
    int defective_body_count{0};
    float total_defective_length{0.0f};
    int total_bodies{0};

    /**
     * @brief Function to call when roll ends.The function will log the
     * roll end time, update the roll info and reset the system.
     */
    void onRollEnd();

    /**
     * @brief Function to generate configuration id using current time.
     */
    //std::string getAppID();

    std::string getcomponentname(std::string component_name);

    /**
     * @brief  Function to subscribe to the topics and work as a callback function are as follows.
     * 2. camAnnotatedImageCallback - To subscribe the cam annotated images.
     * 3. defectDetailsCallback - To subscribe the defects from the camera.
     * 4. materialTypeCallback - To subscribe the material type.
     * 5. metadataChangeCallback - To subscribe the metadata change.
     * 6. pixelPerMMCallback - To subscribe the pixel per mm.
     * 7. pulsePerMMCallback - To subscribe the pulse per mm.
     * 8. resetSystemCallback - To subscribe the reset system.
     * 9. shutdownCallback - To subscribe the shutdown.
     * 10. restartCallback - To subscribe the restart.
     * 11. systemConfigSyncCallback - To subscribe the system config sync.
     * 12. cameraConfigCallback - To subscribe the camera config.
     * 13. ignoreInspectionCallback - To subscribe the ignore inspection toggle.
     * 14. dataCollectionCallback - To subscribe the data collection.
     * 15. stopAtLastDefectToggleCallback - To subscribe the stop at last defect toggle.
     * 16. setJobAckCallback - To subscribe the set job ack.
     * 17. stoppingDefectCallback - To subscribe the stopping defect.
     * 18. lightToggleCallback - To subscribe the light toggle.
     */
    void camAnnotatedImageCallback(const sensor_msgs::Image &msg);
    void defectDetailsCallback(const vision_msgs::Detection2DArray &msg);
    void materialTypeCallback(const std_msgs::String &msg);
    void metadataChangeCallback(const std_msgs::String &msg);
    void pixelPerMMCallback(const std_msgs::Float32 &msg);
    void pulsePerMMCallback(const std_msgs::Float32 &msg);
    void resetSystemCallback(const std_msgs::Empty &msg);
    void shutdownCallback(const std_msgs::Empty &msg);
    void restartCallback(const std_msgs::Empty &msg);
    void systemConfigSyncCallback(const std_msgs::Empty &msg);
    void cameraConfigCallback(const std_msgs::String &msg);
    void ignoreInspectionCallback(const std_msgs::Bool &msg);
    void dataCollectionCallback(const std_msgs::Bool &msg);
    void stopAtLastDefectToggleCallback(const std_msgs::Bool &msg);
    void setJobAckCallback(const std_msgs::String &msg);
    void stoppingDefectCallback(const vision_msgs::Detection2D &msg);
    void lightToggleCallback(const std_msgs::Bool &msg);
    void setBarcodeCallback(const std_msgs::String &msg);   // Added
    void versionCallback(const std_msgs::String &msg);      // Added
    void setWorkOrderCallback(const std_msgs::String &msg); // Added
    void bodyCounterCallback(const std_msgs::String &msg) ; // Added
    void rollContinueAckcallback(const std_msgs::Int8 &msg);


        /**
     * @brief Retrieves the Barcode Configuration from the system configuration.
     *
     * First checks if "AppConfig" exists and contains a "Barcode" entry under "App".
     * If not found, falls back to "BarcodeConfig" directly.
     *
     * @return JSON object of Barcode configuration or empty if not found.
     */
    json getBarcodeConfig();

};
