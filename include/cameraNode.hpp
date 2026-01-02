/**
 * @file cameraNode.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Blue print of camera node in weaving inspection.
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) 2023
 *
 */
// System Includes
#include <ctime>
#include <queue>
#include <chrono>
#include <json.hpp>
#include <iostream>
#include <mutex>
#include <fstream>
#include <boost/filesystem.hpp>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/Detection2D.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>

// Custom Msgs and Services
#include <weaving_inspection/GetPulseDistance.h>
#include <weaving_inspection/GetCurrentPulses.h>

// Robro Systems Includes
#include <robroFloatingLicense.h>
#include <robroImageSaver.h>
#include <cameraSimulation.h>

// Custom Class
#include <notifier.hpp>
#include <mysqlClient.hpp>
#include <cameraManager.hpp>
#include <networkManager.hpp>
#include <cameraIndicator.hpp>
#include <defectsProcessor.hpp>
#include <jobAndRecipeManager.hpp>
#include <intervalTask.hpp>
#include <logMessage.hpp>
#include <paramStore.hpp>
/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Main Class to handle all interactions with the camera.
 * Grabs images from camera, queues them up, predicts on them based
 * on a recipe and publishes the results & annotated image.
 */
class CameraNode
{

public:
    /**
     * @brief It indicates the debug mode of the camera node.
     * If true, it will update the ros logger level to debug.
     * If false, it will update the ros logger level to info.
     */
    bool debug_mode{false};

    /**
     * @brief It indicates that do we need to publish the original image or not.
     *
     */
    bool publish_original_image{false};

    /**
     * @brief It indicates that do we need to save the images or not when
     * defect is detected.
     */
    bool save_all_defect_images{false};

    /**
     * @brief It indicates whether the cryptlex license is expired or not.
     *
     */
    bool is_license_expired{false};

    /**
     * @brief It indicates whether the roll is started or not.
     *
     */
    bool is_roll_started{false};

    // TODO: What is the need of this variable not used anyware.
    /**
     * @brief Path to save the raw images corresponding to the past processed images.
     */
    std::string past_images_save_path;

    // TODO What is the need of this variable not used anyware.
    /**
     * @brief Path to save the defected images detected by the camera node.
     */
    std::string defect_images_save_path;

    // TODO: What is the need of this variable not used anyware.
    /**
     * @brief Path to save the false positive images detected by the camera node.
     */
    std::string false_positive_images_save_path;

    /**
     * @brief Default path to save the images.
     */
    std::string default_images_save_path;

    /**
     * @brief Path to project base directory.
     */
    std::string project_base_path;

    /**
     * @brief Project ID of the current project.
     */
    std::string project_id;

    int crop_margin;

    /**
     * @brief Notifier prefix to send the notification to the notifier.
     */
    std::string notifier_prefix;

    /**
     * @brief Stores the cryptlex product Id given on dashboard
     * Used to check the license validity.
     */
    std::string license_product_id;

    /**
     * @brief Stores the cryptlex license key given by dashboard
     * Used to check the license validity.
     */
    std::string license_host_url;

    /**
     * @brief Stores the cryptlex license expiry message.
     * Which includes the license expiry date and message.
     */
    std::string license_expiry_msg{""};

    /**
     * @brief Stores the cryptlex license exipry data.
     * which use to show on front.
     */
    std::string license_expiry_date{""};

    /**
     * @brief Indicate whether license expiry verification sent to gui or not.
     */
    bool license_warning_sent{false};

    /**
     * @brief Name of the current roll dataset
     */
    std::string dataset_name{""};

    /**
     * @brief the dataset id of the roll
     */
      int64_t dataset_id{0};

    /**
     * @brief Queue to store the images grabbed from the camera and
     * to be processed by the network.
     */
    std::queue<cv::Mat> acquired_image_queue;

    /**
     * @brief Vector to store the images grabbed from the camera with
     * their corresponding frame id and defect detection status.
     */
    std::vector<std::pair<std::pair<int, bool>, cv::Mat>> past_images_queue;

    /**
     * @brief Map to store the frame id and the corresponding camera counter.
     */
    std::map<int, int64_t> frame_id_and_pulses_map;

    /**
     * @brief Maintains last loop close pulse value.When we close the loop the next time,
     * we'll use this to understand how much fabric has moved under Camera in that time.
     */
    int64_t last_loop_close_pulse_value{0};

    /**
     * @brief Number of images published by the camera node after processing.
     */
    int num_imgs_published{0};

    /**
     * @brief Number of images processed by the camera node.
     */
    int number_of_image_processed{0};

    /**
     * @brief Number of pulses to be processed camera to get one image.
     */
    int pulses_per_frame;

    /**
     * @brief Number of past image to be stored in the past_images_queue.
     * Since we are using a vector to store the past images, we need to protect the RAM usage.
     */
    int num_past_images_to_store{20};

    /**
     * @brief Stores the offset needed to crop the image from left side.
     */
    int left_crop_offset{0};

    /**
     * @brief Stores the offset needed to crop the image from right side.
     *
     */
    int right_crop_offset{0};

    /**
     * @brief Number of original images to be published by the camera node.
     *
     */
    int number_of_original_image_published{0};

    /**
     * @brief Maximum number of original images to be published by the camera node.
     *
     */
    int max_num_of_original_image_to_published{50};

    /**
     * @brief Number of pulse to be processed by the camera node to get one pixel.
     */
    double pulse_per_pixel{0};

    /**
     * @brief Number of pixels available in the camera image per mm.
     */
    double pixel_per_mm;

    /**
     * @brief Number of pulses available in the encoder per mm.
     *
     */
    double pulse_per_mm;

    /**
     * @brief Max Roll length meter.
     *
     */
    double max_roll_length_m;

    /**
     * @brief Lower threshold to store the images.
     */
    float save_defect_images_lower_threshold;

    /**
     * @brief The Mutex use to synchronize the reset of weaving node.
     */
    std::mutex reset_mtx;

    /**
     * @brief Camera Node publisher to publish following topics.
     * 1. job_set_ack: To publish the job set ack.
     * 2. camera_health_msg: To publish the camera health message.
     * 3. defect: To publish the defect message.
     * 4. recipes_list: To publish the recipes list.
     * 5. cam_gui_config_pub: To publish the camera configuration for GUI.
     * 6. camera_config: To publish the camera config.
     * 7. edit_recipe_json: To publish the edit recipe json.
     */
    ros::Publisher job_set_ack_pub,
        camera_health_msg_pub,
        defect_pub,
        recipes_list_pub,
        cam_gui_config_pub,
        camera_config_pub,
        all_nn_files_pub,
        edit_recipe_json_pub;

    /**
     * @brief Camera Node publisher to publish the following topics.
     * 1. annotated_image: To publish the annotated image.
     * 2. original_image: To publish the original image.
     */
    image_transport::Publisher annotated_image_pub,
        original_image_pub;

    /**
     * @brief Camera Node subscriber to subscribe to the following topics.
     * 1. save_image_flag: To set save the images flag.
     * 2. save_past_images: To save the past images.
     * 3. reset: To reset the processing.
     * 4. save_false_positive_image: To save the false positive images.
     * 5. dancer_sensor_on: To turn on the dancer sensor.
     * 6. metadata: To change the metadata.
     * 7. load_recipe: To load the recipe.
     * 8. camera_exposure: To change the camera exposure.
     * 9. camera_config_sync: To sync the camera configuration.
     * 10. threshold_value: To change the threshold value.
     */
    ros::Subscriber save_image_flag_sub,
        save_past_images_sub,
        reset_sub,
        save_false_positive_image_sub,
        dancer_sensor_on_sub,
        metadata_sub,
        load_recipes_sub,
        camera_exposure_sub,
        camera_config_sync_sub,
        threshold_value_sub,
        load_recipe_json_sub,
        save_group_recipes_sub,
        get_all_nn_files_sub,
        save_recipe_json_sub;

    /**
     * @brief Camera Node service server to provide the following services.
     * 1. pulse: To get the current pulse of the camera.
     * 2. last_loop_close: To get the last loop close pulse value.
     */
    ros::ServiceServer pulse_service,
        last_loop_close_service;

    /**
     * @brief RobroImageSaver object to save the raw images corresponding to
     * the past processed images.
     */
    RobroImageSaver *past_images_saver;

    /**
     * @brief RobroImageSaver object to save the false positive images.
     */
    RobroImageSaver *false_positive_images_saver;

    /**
     * @brief RobroImageSaver object to save the defect images.
     */
    RobroImageSaver *defect_image_saver;

    /**
     * @brief CameraManager object to manage the camera.
     */
    CameraManager cameraManager;

    /**
     * @brief CameraParams object to store the camera parameters.
     */
    CameraParams cameraParams;

    /**
     * @brief NetworkManager object to manage the network.
     */
    NetworkManager networkManager;

    /**
     * @brief DefectsProcessor object to process the defects.
     */
    DefectsProcessor defectsProcessor;

    /**
     * @brief JobAndRecipeManager object to manage the jobs and recipes.
     */
    JobAndRecipeManager jobAndRecipeManager;

    /**
     * @brief Recipe object to store the current recipe.
     */
    Recipe recipe;

        /**
     * @brief Param store object. Which is used to store the parameters.
     */
    ParamStore *store;

    /**
     * @brief system logger object. Which is used to log the messages.
     */
    MySQLClient &logger = MySQLClient::getInstance();

    /**
     * @brief CameraIndicator object to indicate the camera status.
     */
    CameraIndicator camera_indicator;

    IntervalTask interval_task_overflow_logger;

    IntervalTask interval_task_q_filling_logger;

    /**
     * @brief Construct a new Camera Node object
     */
    CameraNode();

    /**
     * @brief Destroy the Camera Node object
     */
    ~CameraNode();

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
     * @brief Function to initialize the ros services.
     */
    void initServices();

    /**
     * @brief Grab the images from the camera and push them into the acquired_image_queue.
     */
    void startLooping();

    /**
     * @brief Function to process the images and publish the results.
     * It will run in thread and process the images from the acquired_image_queue and publish the results.
     */
    void processAndPublishImage();

    /**
     * @brief Function to push the image into the acquired_image_queue.
     *
     * @param img - Image to be pushed into the acquired_image_queue.
     */
    void pushIntoAcquiredImageQueue(cv::Mat img);

    /**
     * @brief Function gives the difference between the current date and given date.
     *
     * @param expiry_date - expiry date of the liciense.
     */

    int dayDifferenceWithCurrentDate(std::string expiry_date);

    /**
     * @brief Function to check for the validity of the license.
     * If the license is not valid, it will stop the camera node.
     * If the license is valid, it will process the images.
     */
    void validateLicense();

    /**
     * @brief Add the system log to into system log server.
     *
     * @param severity - Severity of the log.
     * @param msg - Message to be logged.
     */
    void addSystemLog(std::string severity, std::string msg, std::string msg_code);

    /**
     * @brief Notify the user through notifications.
     *
     * @param msg - Message to be notified.
     * @param severity - Severity of the notification.
     */
    void notify(std::string msg, std::string severity);

    /**
     * @brief Notify the general info for user through notifications.
     *
     * @param msg - Message to be notified.
     * @param severity - Severity of the notification.
     */
    void notifyGeneralInfo(std::string msg, std::string severity);

    /**
     * @brief Update the system with the recipe.
     *
     * @param recipe_name - Name of the recipe.
     */
    void updateSystemWithRecipe(std::string recipe_name);

    /**
     * @brief Sync the camera configuration with the GUI.
     */
    void guiConfigSync();

    /**
     * @brief for updating the recipe.
     *
     * @param new_recipe_name - Name of the new recipe.
     * @return true - If the recipe is updated successfully.
     * @return false - If the recipe is not updated successfully.
     */
    bool updateRecipe(std::string new_recipe_name);

    /**
     * @brief  Function to subscribe to the topics and work as a callback function are as follows.
     * 1. save_image_flag: To set save the images flag.
     * 2. save_past_images: To save the past images.
     * 3. reset: To reset the processing.
     * 4. save_false_positive_image: To save the false positive images.
     * 5. dancer_sensor_on: To turn on the dancer sensor.
     * 6. metadata: To change the metadata.
     * 7. load_recipe: To load the recipe.
     * 8. camera_exposure: To change the camera exposure.
     * 9. camera_config_sync: To sync the camera config.
     * 10. threshold_value: To change the threshold value.
     */
    void saveImageFlagCallback(const std_msgs::Bool &msg);
    void savePastImagesCallback(const std_msgs::Empty &msg);
    void resetCallback(const std_msgs::Empty &msg);
    void saveFalsePositiveImageCallback(const std_msgs::Int16 &msg);
    void recordLoopCloseValueCallback(const std_msgs::Empty &msg);
    void metadataCallback(const std_msgs::String &msg);
    void loadRecipesCallback(const std_msgs::Empty &msg);
    void cameraExposureCallback(const std_msgs::Int16 &msg);
    void cameraConfigSyncCallback(const std_msgs::Empty &msg);
    void thresholdCallback(const std_msgs::Float32 &msg);
    void recipeJSONSaveCallback(const std_msgs::String &msg);
    void recipeGroupsSaveCallback(const std_msgs::String &msg);
    void recipeJSONLoadCallback(const std_msgs::Empty &msg);
    void getAllNNFilesCallback(const std_msgs::Empty &msg);

    /**
     * @brief Function to provide the following services.
     * 1. pulse: To get the current pulse of the camera.
     * 2. last_loop_close: To get the last loop close pulse value.
     */

    /**
     * @brief Gets the Pulse difference of a given frameID with the current reading of counter pulses in the camera.
     *
     * @param req -  request of type GetPulseDistance.srv
     * @param res  - response of type GetPulseDistance.srv
     * @return true - if request is completed successfully
     * @return false - if request is not completed successfully
     */
    bool getPulseDistance(weaving_inspection::GetPulseDistance::Request &req,
                          weaving_inspection::GetPulseDistance::Response &res);

    /**
     * @brief Gets the difference between camera counter values from the last time dancer sensor was on
     *
     * @param req - request of type GetCurrentPulses.srv
     * @param res - response of type GetCurrentPulses.srv
     * @return true - if request is completed successfully
     * @return false - if request is not completed successfully
     */
    bool getCameraPulsesFromLastLoopClose(weaving_inspection::GetCurrentPulses::Request &req,
                                          weaving_inspection::GetCurrentPulses::Response &res);

    // Tile images
    std::vector<cv::Mat> cropImageIntoTiles(const cv::Mat &, int tileSize);

         /**
     * @brief Saves the JSON data in the same directory as the image.
     *
     * @param image_path The path of the image file.
     * @param data The JSON data to be saved.
     * @return true if the JSON file is saved successfully, false otherwise.
     */
    inline bool saveDetectionJson(const std::string &image_path, const json &data)
    {
        try
        {
            boost::filesystem::path full_path(image_path);
            std::string json_file_name = full_path.stem().string() + ".json";
            std::string json_save_path = (full_path.parent_path() / json_file_name).string();

            std::ofstream json_file(json_save_path);
            if (json_file.is_open())
            {
                json_file << data.dump(4); // pretty print
                json_file.close();
                // ROS_INFO_STREAM("[saveJsonInSameDirectoryAsImage] Saved JSON to: " << json_save_path);
                return true;
            }
            else
            {
                ROS_ERROR_STREAM("[saveDetectionJson] Failed to open file: " << json_save_path);
                return false;
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("[saveDetectionJson] Exception: " << e.what());
            return false;
        }
    }
};
