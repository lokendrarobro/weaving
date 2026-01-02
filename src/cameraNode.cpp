/**
 * @file cameraNode.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Camera Node for KWIS for FIBC
 * @version 1.2
 * @date 2023-04-09
 *
 * @copyright Copyright (c) 2022-2029
 *
 */
#include <cameraNode.hpp>

/**
 * @brief Construct a new KWIS Line-scan Camera Node Application.
 *
 */
CameraNode::CameraNode() : interval_task_overflow_logger(10 * 1000), interval_task_q_filling_logger(10 * 1000)
{
    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[camera node] Couldn't read some params from the launch file. Check their names and types!";
    }

    validateLicense();

    initPubSub();

    initParams();

    license_warning_sent = false; // false to again check license expiry days remaining

    // If the camera is initialized, go ahead.
    if (cameraParams.is_initialized)
    {
        // If the camera is the master, go ahead and initialize the services.
        if (cameraParams.is_master)
        {
            cameraManager.resetCounterValue();
            last_loop_close_pulse_value = 0; // reset here
            initServices();
        }
        // Start processing the images in a separate thread.
        std::thread processing_thread(&CameraNode::processAndPublishImage, this);
        processing_thread.detach();
    }

    startLooping();
}

/**
 * @brief Destroy the Camera Node:: Camera Node object
 *
 */
CameraNode::~CameraNode()
{
    delete past_images_saver;
    delete false_positive_images_saver;
    delete defect_image_saver;
}

/**
 * @brief Function to read the ROS parameters.
 *
 * @return true - If the parameters are read successfully.
 * @return false - If the parameters are not read successfully.
 */
bool CameraNode::readROSParams()
{

    // Read parameters with default value assigned
    ROS_INFO("Loading basic params cam node...");

    return (
        // private node parameters
        ros::param::get("~pulses_per_frame", pulses_per_frame) &&

        // public node parameters
        ros::param::get("project_base_path", project_base_path) &&
        ros::param::get("project_id", project_id) &&

        ros::param::get("crop_margin", crop_margin) &&

        ros::param::get("pixel_per_mm", pixel_per_mm) &&
        ros::param::get("pulse_per_mm", pulse_per_mm) &&
        ros::param::get("max_roll_length_m", max_roll_length_m) &&

        ros::param::get("debug_mode", debug_mode) &&

        ros::param::get("~crop_left", left_crop_offset) &&
        ros::param::get("~crop_right", right_crop_offset) &&

        ros::param::get("num_past_images_to_store", num_past_images_to_store) &&
        ros::param::get("save_defect_images_lower_threshold", save_defect_images_lower_threshold) &&

        ros::param::get("license_product_id", license_product_id) &&
        ros::param::get("license_host_url", license_host_url));
}

/**
 * @brief Initialize the camera node parameters.
 */
void CameraNode::initParams()
{
    store = new ParamStore(project_base_path);
    json save_defect_images = store->getParam("save_all_defect_images");

    if (save_defect_images != nullptr)
    {
        save_all_defect_images = save_defect_images.get<bool>();
    }
    else
    {
        store->setParam("save_all_defect_images", save_all_defect_images);
    }
    // Get the camera parameters
    cameraParams = cameraManager.getCameraParams();
    // Set notifier prefix
    notifier_prefix = "[Cam " + cameraParams.id + "] ";
    // Calculate the constant.
    pulse_per_pixel = pulse_per_mm / pixel_per_mm;

    // Initialize the Folder paths for saving images.
    boost::filesystem::path base_path(project_base_path);

    default_images_save_path = boost::filesystem::path("/images/").string();

    // Initialize the image savers.
    defect_image_saver = new RobroImageSaver(default_images_save_path, "image_", ".jpg");
    past_images_saver = new RobroImageSaver(default_images_save_path, "image_", ".jpg");
    false_positive_images_saver = new RobroImageSaver(default_images_save_path, "image_", ".jpg");

    // update cropping margin parameter
    jobAndRecipeManager.updateCroppingMarging(crop_margin); // update cropping

    // Initialize the job and recipe manager with first recipe.
    jobAndRecipeManager.loadRecipes(project_base_path, project_id);
    recipe = jobAndRecipeManager.getCurrentRecipe();
    // Load the recipe
    networkManager.loadNetwork(recipe);
    defectsProcessor.updateNames(recipe.names_file_path);
    float x_px_per_mm = cameraParams.frame_width / (float)(cameraParams.fov_x_end_mm - cameraParams.fov_x_start_mm);
    defectsProcessor.updatePxPerMM(x_px_per_mm, (float)pixel_per_mm);
    // Update the camera parameters.
    cameraManager.setExposure(recipe.exposure);

    if (save_defect_images_lower_threshold < recipe.threshold)
    {
        networkManager.setThreshold(save_defect_images_lower_threshold);
    }

    // Sync the camera configuration with the GUI.
    guiConfigSync();

    // Set the logger level.
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, debug_mode ? ros::console::levels::Debug : ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
}

/**
 * @brief Function to initialize the ros publishers and subscribers.
 */
void CameraNode::initPubSub()
{
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    /**
     * @brief Camera Node Publishers.
     */
    annotated_image_pub = it.advertise("/cam/annotated_image", 100);
    original_image_pub = it.advertise("/brightness_checker/image", 1);
    camera_health_msg_pub = n.advertise<std_msgs::String>("/gui/label/cam_node_health_data", 1);
    defect_pub = n.advertise<vision_msgs::Detection2DArray>("/cam/defect_result", 100);
    recipes_list_pub = n.advertise<std_msgs::String>("/gui/value/recipe_list", 1);
    cam_gui_config_pub = n.advertise<std_msgs::String>("/gui/cam_node/config", 1);
    camera_config_pub = n.advertise<std_msgs::String>("/cam/config", 100);
    edit_recipe_json_pub = n.advertise<std_msgs::String>("/gui/value/config", 10);
    job_set_ack_pub = n.advertise<std_msgs::String>("/cam/ack/job", 1);
    all_nn_files_pub = n.advertise<std_msgs::String>("/gui/sync/all_nn_files", 1);

    /**
     * @brief Camera Node Subscribers.
     */
    save_image_flag_sub = n.subscribe("/gui/checkbox/save_image", 1, &CameraNode::saveImageFlagCallback, this);
    save_past_images_sub = n.subscribe("/gui/button/btn_save_past_images", 1, &CameraNode::savePastImagesCallback, this);
    save_false_positive_image_sub = n.subscribe("/gui/button/btn_false_positive", 1, &CameraNode::saveFalsePositiveImageCallback, this);
    reset_sub = n.subscribe("/gui/button/reset", 1, &CameraNode::resetCallback, this);
    dancer_sensor_on_sub = n.subscribe("/cam/record_loop_close", 1, &CameraNode::recordLoopCloseValueCallback, this);
    metadata_sub = n.subscribe("/main/roll/camera_metadata", 1, &CameraNode::metadataCallback, this);
    load_recipes_sub = n.subscribe("/gui/sync/configuration", 1, &CameraNode::loadRecipesCallback, this);
    camera_exposure_sub = n.subscribe("/gui/value/exposure", 1, &CameraNode::cameraExposureCallback, this);
    camera_config_sync_sub = n.subscribe("/cam/config_sync", 10, &CameraNode::cameraConfigSyncCallback, this);
    threshold_value_sub = n.subscribe("/gui/value/threshold", 1, &CameraNode::thresholdCallback, this);
    save_recipe_json_sub = n.subscribe("/gui/button/save_json", 1, &CameraNode::recipeJSONSaveCallback, this);
    save_group_recipes_sub = n.subscribe("/gui/onchange/recipedata", 1, &CameraNode::recipeJSONSaveCallback, this);
    load_recipe_json_sub = n.subscribe("/gui/button/json_editor", 1, &CameraNode::recipeJSONLoadCallback, this);
    get_all_nn_files_sub = n.subscribe("/gui/list/all_nn_files", 1, &CameraNode::getAllNNFilesCallback, this);
}

/**
 * @brief Function to initialize the ros services.
 */
void CameraNode::initServices()
{
    ros::NodeHandle n;
    /**
     * @brief Services
     */
    pulse_service = n.advertiseService("get_pulse_distance", &CameraNode::getPulseDistance, this);
    last_loop_close_service = n.advertiseService("get_camera_pulses_from_last_loop_close", &CameraNode::getCameraPulsesFromLastLoopClose, this);
}

/**
 * @brief Grab the images from the camera and push them into the acquired_image_queue.
 */
void CameraNode::startLooping()
{
    cv::Mat image;
    int grabbing_failed_counter = 0;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IntervalTask log_at_interval(10 * 1000); // Once Every 10 seconds

    while (ros::ok())
    {
        // Check if the license is expired.
        if (!cameraParams.is_simulation && !cameraParams.is_simulation_with_cam_connected && is_license_expired)
        {
            notify(license_expiry_msg + "\nPlease contact Robro Systems PVT LTD.", "alert");
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }

        if (!cameraParams.is_initialized)
        {
            notify("Camera Not Initialized: Cam " + cameraParams.id, "alert");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            continue;
        }

        try
        {
            // Grab the image from the camera.
            cameraParams = cameraManager.grabImage(image);

            if (!is_roll_started)
            {
                continue; // Skip the loop if the roll is not started.
            }

            // Check if the image is valid.
            if (cameraParams.is_valid_img)
            {
                if (recipe.is_available)
                {

                    number_of_image_processed++;

                    // Save the frame id and pulses in a map.
                    frame_id_and_pulses_map[number_of_image_processed] = cameraManager.getCounterValue(number_of_image_processed);
                    pushIntoAcquiredImageQueue(image.clone());
                }
            }
            else
            {
                ROS_ERROR_THROTTLE(10, "Grabbing Failed\n");
                grabbing_failed_counter++;

                if (log_at_interval.isReady())
                {
                    log_at_interval.reset();

                    // WHAT THINGS WE WANT TO KNOW WHEN GRABBING FAILS - CURRENT FPS, INF TIME, CPU
                    // LAST RESET TIME, SPEEDS?

                    notify("Grabbing Failed", "alert");
                    addSystemLog("ERROR", "[StartLooping] Grabbing Failed: Grab Fail Counter: " + std::to_string(grabbing_failed_counter), "CAM011");
                }
            }
        }
        catch (std::exception &e)
        {
            std::string msg = e.what();
            addSystemLog("ERROR", "[StartLooping]" + msg, "CAM012");
            ROS_ERROR("Exception Caught! %s", e.what());
        }
    }

    addSystemLog("INFO", "[startLooping] ros::ok() finished. Exiting... Camera Loop End.", "INFO002");

    spinner.stop();
}

/**
 * @brief Function to process the images and publish the results.
 * It will run in thread and process the images from the acquired_image_queue and publish the results.
 */
void CameraNode::processAndPublishImage()
{
    ros::Rate rate(30);
    cv::Mat img;
    cv::Mat annotated_image;
    sensor_msgs::ImagePtr annotated_image_msg;
    vision_msgs::Detection2DArray current_detections;
    json frame_id_and_seq_json;
    std_msgs::String camera_health_msg;
    sensor_msgs::ImagePtr original_image_msg;
    json message;

    // Check if the camera is initialized.
    if (!cameraParams.is_initialized)
    {
        ROS_ERROR_THROTTLE(10, "Camera Not Initialized! Returning.");
        notify("Cam not initialized", "alert");
        addSystemLog("ERROR", "[processAndPublishImage] Failed to initialize cam", "CAM014");
        return;
    }

    while (ros::ok())
    {
        if (!acquired_image_queue.empty() && recipe.is_available && networkManager.isNetworkLoaded())
        {
            {
                std::lock_guard<std::mutex> lock(reset_mtx);
                img = acquired_image_queue.front();
                acquired_image_queue.pop();
                // Increment the number of images published.
                num_imgs_published++;
            }

            // Crop the right or left edges if requested.
            try
            {
                if (left_crop_offset > 0 || right_crop_offset > 0)
                {
                    cv::Rect rect(left_crop_offset, 0, img.size().width - left_crop_offset - right_crop_offset, img.size().height);
                    img = img(rect);
                }
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Unable to crop image: %s", e.what());
                addSystemLog("ERROR", "[processAndPublishImage] Unable to crop image. Cam: " + cameraParams.id + " Left Crop: " + std::to_string(left_crop_offset) + " Right Crop: " + std::to_string(right_crop_offset), "APP016");
            }

            // Change the image to 3 channels if it is 1 channel.
            if (img.channels() == 1)
            {
                cv::cvtColor(img, img, CV_GRAY2BGR);
            }

            if (publish_original_image && cameraParams.is_master)
            {
                original_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                // Store camera exposure in frame_id for brightness adjustment.
                original_image_msg->header.frame_id = std::to_string(cameraParams.exposure_us);
                // if (!(cameraParams.is_simulation || cameraParams.is_simulation_with_cam_connected))
                {
                    original_image_pub.publish(original_image_msg);
                }
                number_of_original_image_published++;
                if (number_of_original_image_published > max_num_of_original_image_to_published)
                {
                    publish_original_image = false;
                }
            }

            // Get the predictions from the network.
            auto preds = networkManager.doPrediction(img, annotated_image);

            // Get the defect regions and Draw the defects on the image.
            bool is_recipe_style_v2_groups = true;

            if (!is_recipe_style_v2_groups)
            {
                current_detections = defectsProcessor.getDefectRegion(preds, annotated_image,
                                                                      recipe.number_of_classes,
                                                                      recipe.threshold,
                                                                      recipe.min_defect_area_px, recipe.cropping_margin);
            }
            else
            {

                current_detections = defectsProcessor.getDefectRegion_v2(preds, annotated_image,
                                                                         recipe);
            }

            if (!current_detections.detections.empty())
            {
                // Add a full image entry to the detection array.
                vision_msgs::Detection2D full_image_detection;
                sensor_msgs::ImagePtr full_image_msg;

                full_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img.clone()).toImageMsg();
                full_image_detection.source_img = *full_image_msg;

                // Add the full image entry to the detection array
                current_detections.detections.push_back(full_image_detection);
            }

            // Post Process the annotated image.
            // cv::putText(annotated_image, cameraParams.position.c_str(),
            //             cv::Point(250, 15 * annotated_image.size().height / 16),
            //             cv::FONT_HERSHEY_TRIPLEX, 5, cv::Scalar(0, 200, 200), 3, cv::LINE_AA);

            // Prepare the current detections message.
            frame_id_and_seq_json["cam_id"] = cameraParams.id;
            frame_id_and_seq_json["seq"] = num_imgs_published;

            // converting image into sensor message
            try
            {
                cv::resize(annotated_image, annotated_image,
                           cv::Size(annotated_image.size().width / 4,
                                    annotated_image.size().height / 4),
                           cv::INTER_LINEAR);

                annotated_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", annotated_image).toImageMsg();
                annotated_image_msg->header.frame_id = frame_id_and_seq_json.dump();
                annotated_image_msg->header.stamp = ros::Time::now();
            }
            catch (const std::exception &e)
            {
                std::string msg = e.what();
                addSystemLog("ERROR", "[processAndPublishImage] During resizing of annotated image: " + msg, "APP016");
                ROS_ERROR("During resizing of annotated image: %s", e.what());
            }

            // Save image and detections to the past_images_queue.
            past_images_queue.push_back(std::make_pair(
                std::make_pair(num_imgs_published,
                               !current_detections.detections.empty()),
                img.clone()));

            if (past_images_queue.size() > num_past_images_to_store)
            {
                past_images_queue.erase(past_images_queue.begin());
            }

            // Save the defect images if requested.
            if (save_all_defect_images && !preds.empty())
            {
                json current_defect_image_json;
                //current_defect_image_json["completely_empty"] = false;

                current_defect_image_json["model_info"] = json::object();
                boost::filesystem::path engine_full_path(recipe.engine_file_path);
                std::string engine_file_name = engine_full_path.stem().string();
                current_defect_image_json["model_info"]["model_name"] = engine_file_name;
                current_defect_image_json["model_info"]["threshold"] = std::round(recipe.threshold * 1000.0) / 1000.0;
                current_defect_image_json["model_info"]["nms"] = std::round(recipe.threshold * 1000.0) / 1000.0;
                current_defect_image_json["predictions"] = json::array();
                current_defect_image_json["annotations"] = json::array();

                std::vector<cv::Mat> tiles = cropImageIntoTiles(img.clone(), 1024);
                auto annotation_by_tiles = defectsProcessor.groupDefectsByTile(1024, img.rows, img.cols);
                for (size_t i = 0; i < tiles.size(); ++i)
                {

                        // Clear previous annotations
                    current_defect_image_json["annotations"].clear();
                    auto annotation_in_this_tile = annotation_by_tiles[i];
                    current_defect_image_json["completely_empty"] = annotation_in_this_tile.empty();
                    std::string defect_image_path = defect_image_saver->save(tiles[i], "", cameraParams.id + std::to_string(i));
                    if (!annotation_in_this_tile.empty())
                    {
                        for (const auto &annotation : annotation_in_this_tile)
                        {
                            // Add the annotation to the JSON.
                            current_defect_image_json["annotations"].push_back(annotation.getJson());
                        }
                    }
                    current_defect_image_json["timestamp"] = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    logger.add_dataset_file(dataset_id, defect_image_path);
                    saveDetectionJson(defect_image_path, current_defect_image_json);
                }
            }

            // Prepare detection message.
            current_detections.header.frame_id = frame_id_and_seq_json.dump();

            int last_pred_time = networkManager.getLastPredictionTime();
            camera_indicator.UpdateStatus(cameraParams.is_valid_img,
                                          (last_pred_time > 0 && networkManager.isNetworkLoaded()),
                                          last_pred_time,
                                          acquired_image_queue.size(),
                                          true,  // need to update camera lib for counter status
                                          true); // need to update camera lib for exposure status

            // Prepare the camera health message.
            message["cam_id"] = cameraParams.id;
            message["num_imgs_published"] = num_imgs_published;
            message["camera_indicator_color"] = camera_indicator.GetColor();
            message["inference_time"] = last_pred_time;
            message["avg_inference_time"] = camera_indicator.GetAverageInferenceTime();
            message["error_msg"] = camera_indicator.GetLastErrorMessage();
            camera_health_msg.data = message.dump();

            annotated_image_pub.publish(annotated_image_msg);
            defect_pub.publish(current_detections);
            camera_health_msg_pub.publish(camera_health_msg);
        }
        else
        {
            rate.sleep();
        }
    }
}

/**
 * @brief Function to push the image into the acquired_image_queue.
 *
 * @param image - Image to be pushed into the acquired_image_queue.
 */
void CameraNode::pushIntoAcquiredImageQueue(cv::Mat image)
{
    acquired_image_queue.push(image);

    /**
     * @brief If the queue size is more than 15, we start dropping frames.
     * Since we are not processing as fast as we are getting, memory will explode at some point.
     */
    if (acquired_image_queue.size() > 30 && interval_task_q_filling_logger.isReady())
    {
        std::string msg;
        msg = "Cam-Node not processing fast enough - Q filling up!";
        msg += " Last Pred Time: " + std::to_string(networkManager.getLastPredictionTime());
        msg += " Q Size: " + std::to_string(acquired_image_queue.size());
        notify(msg, "warning");
        addSystemLog("WARNING", "[pushIntoAcquiredImageQueue] Cam-Node not processing fast enough - Q filling up!", "PERF007");
        interval_task_q_filling_logger.reset();
    }
}

int CameraNode::dayDifferenceWithCurrentDate(std::string expiry_date)
{
    std::tm given_tm = {};
    std::istringstream ss(expiry_date);

    ss >> given_tm.tm_year;
    ss.ignore(); // ignore the '-'
    ss >> given_tm.tm_mon;
    ss.ignore(); // ignore the '-'
    ss >> given_tm.tm_mday;

    ss.ignore();
    ss >> given_tm.tm_hour;
    ss.ignore();
    ss >> given_tm.tm_min;
    ss.ignore();
    ss >> given_tm.tm_sec;

    given_tm.tm_mon -= 1;
    given_tm.tm_year -= 1900;

    // Get current time
    std::time_t t = std::time(nullptr);
    std::tm *now = std::localtime(&t);

    // Convert to time_t
    std::time_t given_time = std::mktime(&given_tm);
    std::time_t current_time = std::mktime(now);

    double difference = std::difftime(given_time, current_time) / (60 * 60 * 24);
    return static_cast<int>(difference);
}
/**
 * @brief Function to check for the validity of the license.
 * If the license is not valid, it will stop the camera node.
 * If the license is valid, it will process the images.
 */
void CameraNode::validateLicense()
{
    // Create Robro License object
    RobroFloatingLicense floating_license(license_product_id, license_host_url);

    // Initialize the Product.
    if (floating_license.initializeProduct())
    {
        // Check if License is Valid
        is_license_expired = !floating_license.isLicenseValid();

        if (is_license_expired)
        {
            license_expiry_msg = "License Expiry Date: " + floating_license.getLicenseExpiryDate();
            license_expiry_msg += "License Status: " + floating_license.getLicenseStatMessage();
            addSystemLog("WARNING", license_expiry_msg, "KVP003");
        }
        license_expiry_date = floating_license.getLicenseExpiryDate();
    }
    else
    {
        notify("License Initialization Failed", "warning");
        addSystemLog("ERROR", "[validateLicense] License Initialization Failed", "KVP003");
        is_license_expired = true;
    }
}

/**
 * @brief Add the system log to into system log server.
 *
 * @param severity - Severity of the log.
 * @param msg - Message to be logged.
 */
void CameraNode::addSystemLog(std::string severity, std::string msg, std::string msg_code)
{
    // System log needs a json with severity and msg.
    msg = "[Cam " + cameraParams.id + "]" + msg;
    logger.add_log(getLogSeverityFromString(severity), getLogCodeFromString(msg_code), msg, LogComponent::CAMERA);
}

/**
 * @brief Notify the user through notifications.
 *
 * @param msg - Message to be notified.
 * @param severity - Severity of the notification.
 */
void CameraNode::notify(std::string msg, std::string severity)
{
    Notifier::getInstance(notifier_prefix + msg, severity);
}
/**
 * @brief Notify the user through notifications.
 *
 * @param msg - Message to be notified.
 * @param severity - Severity of the notification.
 */
void CameraNode::notifyGeneralInfo(std::string msg, std::string severity)
{
    Notifier::getInstance(msg, severity);
}
/**
 * @brief Update the system with the recipe.
 *
 * @param recipe_name - Name of the recipe.
 */
void CameraNode::updateSystemWithRecipe(std::string recipe_name)
{
    // Update the recipe.
    if (jobAndRecipeManager.isRecipeDifferent(recipe_name))
    {
        if (jobAndRecipeManager.setCurrentRecipe(recipe_name))
        {
            recipe = jobAndRecipeManager.getCurrentRecipe();
            networkManager.loadNetwork(recipe);
            defectsProcessor.updateNames(recipe.names_file_path);
            addSystemLog("INFO", "[updateSystemWithRecipe] LoadNetwork : " + recipe_name, "INFO003");
        }
    }
    else
    {
        networkManager.setThreshold(recipe.threshold);
    }

    // Update the camera parameters.
    cameraManager.setExposure(recipe.exposure);
    if (save_defect_images_lower_threshold < recipe.threshold)
    {
        networkManager.setThreshold(save_defect_images_lower_threshold);
        // log only from master camera
        if (cameraParams.is_master)
        {
            addSystemLog("ACTIVITY", "[updateSystemWithRecipe] Threshold changed to " + std::to_string(save_defect_images_lower_threshold), "INFO003");
        }
    }
    guiConfigSync();
    cameraParams = cameraManager.getCameraParams();
}

/**
 * @brief Sync the camera configuration with the GUI.
 */
void CameraNode::guiConfigSync()
{
    if (cameraParams.is_master)
    {
        // Create the json object for the threshold.
        json gui_config;
        gui_config["threshold"] = recipe.threshold;
        gui_config["exposure"] = cameraParams.exposure_us;
        gui_config["license_expiry_date"] = license_expiry_date;
        gui_config["save_all_defect_images"] = save_all_defect_images;

        std_msgs::String msg;
        msg.data = gui_config.dump();
        cam_gui_config_pub.publish(msg);

        if (!license_warning_sent)
        {
            int remainingDays = dayDifferenceWithCurrentDate(license_expiry_date);

            if (remainingDays < 10 && remainingDays > 0)
            {
                notifyGeneralInfo("License is about to expire in :\n" + std::to_string(remainingDays) + " days please renew it", "popup");
            }
            license_warning_sent = true;
        }
    }
}

/**
 * @brief for updating the recipe.
 *
 * @param new_recipe_name - Name of the new recipe.
 * @return true - If the recipe is updated successfully.
 * @return false - If the recipe is not updated successfully.
 */
bool CameraNode::updateRecipe(std::string new_recipe_name)
{
    if (!jobAndRecipeManager.checkRecipeAndFiles(new_recipe_name))
    {
        notify("Recipe not in Recipes JSON.", "alert");
        addSystemLog("ERROR", "[updateRecipe] Recipe not in Recipes JSON : " + new_recipe_name, "APP015");
        return false;
    }

    // Update the recipe.
    updateSystemWithRecipe(new_recipe_name);
    guiConfigSync();
    return true;
}

/**
 * @brief Subscribe to the save_image_flag topic to turn on/off the defect image saving.
 *
 * @param msg - Bool message.
 */
void CameraNode::saveImageFlagCallback(const std_msgs::Bool &msg)
{
    save_all_defect_images = msg.data;
    std::string is_on = msg.data ? "On" : "Off";
    notify("Defect Image Saving Turned " + is_on, "info");
    addSystemLog("ACTIVITY", "[saveImageFlagCallback] Defect Image Saving Turned:" + is_on, "INFO003");
}

// Tile images
std::vector<cv::Mat> CameraNode::cropImageIntoTiles(const cv::Mat &image, int tileSize)
{
    std::vector<cv::Mat> tiles;

    int rows = image.rows;
    int cols = image.cols;

    // Iterate through the image and extract 1024x1024 tiles
    for (int y = 0; y < rows; y += tileSize)
    {
        for (int x = 0; x < cols; x += tileSize)
        {
            // Calculate the bottom right corner of the current tile
            int tileHeight = std::min(tileSize, rows - y);
            int tileWidth = std::min(tileSize, cols - x);

            // Create the tile
            cv::Rect tileRect(x, y, tileWidth, tileHeight);
            cv::Mat tile = image(tileRect).clone(); // Clone the tile to create a new Mat
            tiles.push_back(tile);
        }
    }

    return tiles;
}

/**
 * @brief Subscribe to the save_past_images topic to save the past images.
 *
 * @todo This is a highly unoptimized way because it makes it difficult to
 * search for exact image that is false negative - need to explore a better way
 *
 * @param msg - Empty message.
 */
void CameraNode::savePastImagesCallback(const std_msgs::Empty &msg)
{
    while (!past_images_queue.empty())
    {
        std::vector<cv::Mat> tiles = cropImageIntoTiles(past_images_queue.front().second, 1024);
        for (size_t i = 0; i < tiles.size(); ++i)
        {
            std::string past_image_file_path = past_images_saver->save(tiles[i], "", cameraParams.id + std::to_string(i));
            logger.add_dataset_file(dataset_id, past_image_file_path);
        }
        past_images_queue.erase(past_images_queue.begin());
    }

    notify("[savePastImagesCallback] Saved Past Images", "success");
}

/**
 * @brief Subscribe to the reset topic to reset the camera node.
 *
 * @todo need a best way to reset. Need some softer reset options as well that
 * don't delete all the good info about defects and imgs that we have.
 *
 * @param msg - Empty message.
 */
void CameraNode::resetCallback(const std_msgs::Empty &msg)
{
    // Empty the past images queue.
    std::vector<std::pair<std::pair<int, bool>, cv::Mat>> empty_queue;
    std::queue<cv::Mat> empty_img_queue;
    past_images_queue.swap(empty_queue);
    {
        std::lock_guard<std::mutex> lock(reset_mtx);
        std::swap(acquired_image_queue, empty_img_queue);
        num_imgs_published = 0;
    }
    // Empty the frame_id_and_pulses_map.
    frame_id_and_pulses_map.clear();

    number_of_image_processed = 0;

    json message;
    std_msgs::String camera_health_msg;
    message["cam_id"] = cameraParams.id;
    message["num_imgs_published"] = 0;
    message["inference_time"] = 0;
    camera_health_msg.data = message.dump();
    camera_health_msg_pub.publish(camera_health_msg);

    // Reseting last loop close value
    // last_loop_close_pulse_value = 0;
    if (cameraParams.is_master)
    {

        addSystemLog("INFO", "[resetCallback] Done", "INFO007");
    }
}

/**
 * @brief Load JSON from file and publish it to front end.
 *
 * @param msg - Empty message
 */
void CameraNode::recipeJSONLoadCallback(const std_msgs::Empty &msg)
{
    if (cameraParams.is_master)
    {
        std_msgs::String recipe_data_msg;
        json all_recipes_data = jobAndRecipeManager.getAllRecipesJSON();
        if (!all_recipes_data.empty())
        {
            recipe_data_msg.data = all_recipes_data.dump();
            edit_recipe_json_pub.publish(recipe_data_msg);
            notify("JSON Loaded.", "success");
        }
        else
        {
            addSystemLog("ERROR", "[recipeJSONLoadCallback] Recipes data is empty!", "APP015");
            notify("Recipes data is empty!", "alert");
        }
    }
}

/**
 * @brief get all files nn
 *
 * @param msg - Empty message
 */
void CameraNode::getAllNNFilesCallback(const std_msgs::Empty &msg)
{
    if (cameraParams.is_master)
    {
        std_msgs::String all_files_msg;
        all_files_msg.data = jobAndRecipeManager.getAllAvailableNamesAndEngineFiles(project_base_path);

        all_nn_files_pub.publish(all_files_msg);
    }
}

void CameraNode::recipeGroupsSaveCallback(const std_msgs::String &msg)
{
    std::cout << "New Recipe JSON Recd:  " << msg.data << std::endl;
}

/**
 * @brief Save JSON from front end to file.
 *
 * @param msg - JSON string message
 */
void CameraNode::recipeJSONSaveCallback(const std_msgs::String &msg)
{
    try
    {
        // we are extracting 2 char of json to check that the json callback is for delete ,modify or create recipe
        auto data = msg.data;
        std::string s = data.substr(data.size() - 2, data.size()); // Extract the last 2 characters
        data.erase(data.size() - 2, 2);                            // Remove the last 2 characters

        json config = json::parse(data);
        bool load_recipe = false;
        // Write to file if it is master
        if (cameraParams.is_master)
        {
            if (jobAndRecipeManager.saveRecipesJSON(config, true))
            {
                load_recipe = true;
                if (s == "<D")
                {
                    notify("Recipe Deleted Successfully!", "success");
                    addSystemLog("ACTIVITY", "[recipeJSONSaveCallback] Recipe deleted Successful!", "INFO011");
                }
                if (s == "<M")
                {
                    notify("Recipe Modified Successfully!", "success");
                    addSystemLog("ACTIVITY", "[recipeJSONSaveCallback] Recipe Modified Successful!", "INFO012");
                }
                if (s == "<C")

                {
                    notify("Recipe Created Successfully!", "success");
                    addSystemLog("ACTIVITY", "[recipeJSONSaveCallback] Recipe created Successful!", "INFO013");
                }
            }
            else
            {
                if (s == "<D")
                    notify("Recipe Deletion failed!", "success");
                if (s == "<M")
                    notify("Recipe Modification failed", "success");
                if (s == "<C")
                    notify("Recipe Creation failed!", "success");
                addSystemLog("WARNING", "[recipeJSONSaveCallback] JSON Not Saved!", "APP015");
            }
        }
        else
        {
            if (jobAndRecipeManager.saveRecipesJSON(config, false))
            {
                load_recipe = true;
            }
        }
        if (load_recipe)
        {
            // Update the recipe.
            recipe = jobAndRecipeManager.getCurrentRecipe();
            networkManager.loadNetwork(recipe);
            defectsProcessor.updateNames(recipe.names_file_path);
            if (save_defect_images_lower_threshold < recipe.threshold)
            {
                networkManager.setThreshold(save_defect_images_lower_threshold);
            }
        }
        guiConfigSync();
    }
    catch (const std::exception &e)
    {
        notify("JSON Not Saved!", "alert");
        addSystemLog("WARNING", "[recipeJSONSaveCallback] JSON Not Saved!", "APP015");
    }
}

/**
 * @brief Subscribe to the save_false_positive_image topic to save the false positive images.
 * Search for the ID and check if this was the camera that detected the defect in the image.
 *
 * @param msg - Int16 message.
 */
void CameraNode::saveFalsePositiveImageCallback(const std_msgs::Int16 &msg)
{
    bool found_id = false;
    for (auto itr = past_images_queue.begin(); itr != past_images_queue.end(); ++itr)
    {
        if ((msg.data == itr->first.first))
        {
            if (itr->first.second)
            {
                std::vector<cv::Mat> tiles = cropImageIntoTiles(itr->second, 1024);
                for (size_t i = 0; i < tiles.size(); ++i)
                {
                    std::string false_positive_file_name = false_positive_images_saver->save(tiles[i], "", cameraParams.id + std::to_string(i));
                    logger.add_dataset_file(dataset_id, false_positive_file_name);
                }
                notify("Saved False + " + std::to_string(msg.data), "success");
            }
            else
            {
                notify("No Defect in ID " + std::to_string(msg.data), "warning");
            }
            found_id = true;
            break;
        }
    }
    if (!found_id)
    {
        notify(" ID Not in Q", "warning");
    }
}

/**
 * @brief Subscribe to loop_close topic to store the value of the counter pulses.
 *
 * @param msg - Empty message.
 */
void CameraNode::recordLoopCloseValueCallback(const std_msgs::Empty &msg)
{
    if (cameraParams.is_master)
    {
        last_loop_close_pulse_value = cameraManager.getCounterValue();
        // std::cout << "[Dancer turn on recordLoopCloseValueCallback]last_loop_close_pulse_value:" << last_loop_close_pulse_value << std::endl;
    }
}

/**
 * @brief Callback for storing metadata with false positive, past queue and defect images.
 *
 * @param msg - metadata in json form dumped in string.
 */
void CameraNode::metadataCallback(const std_msgs::String &msg)
{
    // parse metadata from the message.

    try
    {
        json metadata = json::parse(msg.data);
        json acknowledge;
        bool is_recipe_loaded = false;
        // Update the recipe.
        if (metadata.contains("recipe_name") && metadata["recipe_name"] != "")
        {
            if (updateRecipe(metadata["recipe_name"].get<std::string>()))
            {
                acknowledge["recipe"] = jobAndRecipeManager.getCurrentRecipeJson();
                is_recipe_loaded = true;
            }
        }

        resetCallback(std_msgs::Empty());

        acknowledge["is_roll_started"] = metadata.contains("is_roll_started") && metadata["is_roll_started"];
        acknowledge["cam_id"] = cameraParams.id;
        std_msgs::String ack_msg;
        ack_msg.data = acknowledge.dump();
        job_set_ack_pub.publish(ack_msg);

        if (metadata.contains("is_roll_started") && metadata["is_roll_started"]) // ROll Start pressed
        {
            cameraManager.resetCounterValue();
            last_loop_close_pulse_value = 0; // reset here
        }

        if (metadata.contains("is_roll_started") && !metadata["is_roll_started"]) // ROll Change turned off from UI
        {
            if (cameraParams.is_master)
            {
                // Publish the original image flag.
                publish_original_image = true;
                number_of_original_image_published = 0;
                cameraManager.resetCounterValue();
                last_loop_close_pulse_value = 0; // reset here
                logger.update_dataset(dataset_id, dataset_name);
            }
            is_roll_started = false;

            // Reset the camera node.
            return;
        }

        if (!is_recipe_loaded)
        {
            return;
        }

        // check if roll type is mentioned or not.
        if (!metadata.contains("id") || !metadata.contains("roll_id") || metadata["id"] == "")
        {
            notify("Unable to save the metadata", "alert");
            addSystemLog("WARNING", "[metadataCallback] Unable to save the metadata.", "APP004");
            return;
        }

        // Get the roll type.
        if (metadata.contains("fully_ignore_disabled"))
        {
            defectsProcessor.setFullyIgnoreDisabledClasses(metadata["fully_ignore_disabled"]);
        }

        if (!metadata.contains("recipe_name") || metadata["recipe_name"] == "")
        {
            metadata["recipe_name"] = recipe.name;
        }

        dataset_name = logger.create_dataset_id(metadata["roll_id"].get<std::string>(), recipe.name, metadata["id"].get<std::string>());
        dataset_id = metadata["robro_roll_id"];

        // Reset the paths for the image to save and create directories.
        boost::filesystem::path base_path(default_images_save_path);

        // Create the folder in the name of app_id
        boost::filesystem::path project_path = base_path / project_id / std::to_string(dataset_id);
        boost::filesystem::create_directories(project_path);

        // Joinging a dataset name
        base_path = project_path / dataset_name;

        std::string new_past_images_save_path, new_false_positive_images_save_path, new_defect_images_save_path;

        new_past_images_save_path = boost::filesystem::path(base_path / "past_images/").string();
        new_false_positive_images_save_path = boost::filesystem::path(base_path / "false_positive/").string();
        new_defect_images_save_path = boost::filesystem::path(base_path / "defect_images/").string();

        // Set the base path for the image saver.
        past_images_saver->base_path = new_past_images_save_path;
        false_positive_images_saver->base_path = new_false_positive_images_save_path;
        defect_image_saver->base_path = new_defect_images_save_path;

        // Create the directories if they don't exist.
        boost::filesystem::create_directories(new_past_images_save_path);
        boost::filesystem::create_directories(new_false_positive_images_save_path);
        boost::filesystem::create_directories(new_defect_images_save_path);

        // TODO: Move this insie master camera condition.
        // Write the metadata.
        std::string metadata_file_path = boost::filesystem::path(base_path / "metadata.json").string();
        std::ofstream file(metadata_file_path);
        file << std::setw(4) << metadata;
        file.close();

        // send notification of metadata successfully write.
        addSystemLog("INFO", "[metadataCallback] Saved Metadata.", "INFO003");

        // Reset the counter value if roll change is true.
        is_roll_started = metadata.contains("is_roll_started") && metadata["is_roll_started"];

        // TODO: What is the need for doing this? can we remove.
        // reset the paths.
        past_images_save_path = new_past_images_save_path;
        false_positive_images_save_path = new_false_positive_images_save_path;
        defect_images_save_path = new_defect_images_save_path;

        // Add a entry of dataset to dataset table
        if (is_roll_started && cameraParams.is_master)
        {
            logger.add_dataset(dataset_id, dataset_name, getAppID());
            logger.add_dataset_file(dataset_id, metadata_file_path);
        }
    }
    catch (...)
    {
        notify("Can't save metadata", "alert");
        addSystemLog("WARNING", "[metadataCallback]Cant save metadata: " + msg.data, "APP004");

        // TODO: What is the need for doing this? can we remove.
        // reset the paths.
        past_images_saver->base_path = past_images_save_path;
        false_positive_images_saver->base_path = false_positive_images_save_path;
        defect_image_saver->base_path = defect_images_save_path;
    }
}

/**
 * @brief Page has loaded in the GUI. Resend values.
 *
 * @param msg
 */
void CameraNode::loadRecipesCallback(const std_msgs::Empty &msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // only send from master camera.
    if (cameraParams.is_master)
    {
        // Get the current recipe.
        recipe = jobAndRecipeManager.getCurrentRecipe();

        // Create the json object for the threshold.

        std::string recipe_list = jobAndRecipeManager.getRecipesListInJSONStr();
        std_msgs::String list_msg;

        list_msg.data = recipe_list;

        // Publish the recipe list and threshold.
        recipes_list_pub.publish(list_msg);
        guiConfigSync();

        // Publish the original image flag.
        publish_original_image = true;
        number_of_original_image_published = 0;
    }

    json message;
    std_msgs::String camera_health_msg;
    message["cam_id"] = cameraParams.id;
    message["num_imgs_published"] = num_imgs_published;
    message["inference_time"] = networkManager.getLastPredictionTime();
    camera_health_msg.data = message.dump();
    camera_health_msg_pub.publish(camera_health_msg);

    // Update the recipe.
    updateSystemWithRecipe(recipe.name);
}

/**
 * @brief Callback for updating the camera exposure.
 *
 * @param msg - exposure value.
 */
void CameraNode::cameraExposureCallback(const std_msgs::Int16 &msg)
{
    int exposure = msg.data;

    // Set the exposure.
    if (cameraManager.setExposure(exposure))
    {
        cameraParams = cameraManager.getCameraParams();
        addSystemLog("INFO", "[cameraExposureCallback] Exposure set to " + std::to_string(cameraParams.exposure_us), "INFO003");
        guiConfigSync();
    }
}

/**
 * @brief Callback to synchronize the camera configuration.
 *
 * @param msg - empty message.
 */
void CameraNode::cameraConfigSyncCallback(const std_msgs::Empty &msg)
{
    json config;
    std_msgs::String config_msg;
    config["id"] = cameraParams.id;
    config["fov_x_start_mm"] = cameraParams.fov_x_start_mm;
    config["fov_x_end_mm"] = cameraParams.fov_x_end_mm;
    config["is_top_camera"] = cameraParams.is_top_camera;
    config["x_px_per_mm"] = cameraParams.frame_width / (float)(cameraParams.fov_x_end_mm - cameraParams.fov_x_start_mm);
    config["position"] = cameraParams.position;
    config["camera_connected"] = cameraParams.is_initialized;
    config["group_id"] = cameraParams.group_id;

    config_msg.data = config.dump();
    camera_config_pub.publish(config_msg);
}

/**
 * @brief Change the current threshold value of network.
 *
 * @param msg -
 */
void CameraNode::thresholdCallback(const std_msgs::Float32 &msg)
{
    recipe.threshold = msg.data;
    notify("Threshold: " + std::to_string(msg.data), "success");
    // log only from master camera
    if (cameraParams.is_master)
    {
        addSystemLog("ACTIVITY", "[thresholdCallback] threshold changed to " + std::to_string(msg.data), "INFO003");
    }
}

/**
 * @brief Gets the Pulse difference of a given frameID with the current reading of counter pulses in the camera.
 *
 * @param req -  request of type GetPulseDistance.srv
 * @param res  - response of type GetPulseDistance.srv
 * @return true - if request is completed successfully
 * @return false - if request is not completed successfully
 */
bool CameraNode::getPulseDistance(weaving_inspection::GetPulseDistance::Request &req,
                                  weaving_inspection::GetPulseDistance::Response &res)
{

    int start_frame_id = req.req_frame_id;
    // If the frame id is not in the map, return false.
    if (!frame_id_and_pulses_map.count(start_frame_id))
    {
        res.pulse_difference = 0;
        return false;
    }

    // If simulation mode, return the difference between the frame id and the start frame id.
    if (cameraParams.is_simulation)
    {
        res.pulse_difference = frame_id_and_pulses_map[number_of_image_processed] - frame_id_and_pulses_map[start_frame_id];
        return true;
    }

    // Get the current pulse value.
    res.pulse_difference = cameraManager.getCounterValue() - frame_id_and_pulses_map[start_frame_id];

    return true;
}

/**
 * @brief Gets the difference between camera counter values from the last time dancer sensor was on
 *
 * @param req - request of type GetCurrentPulses.srv
 * @param res - response of type GetCurrentPulses.srv
 * @return true - if request is completed successfully
 * @return false - if request is not completed successfully
 */
bool CameraNode::getCameraPulsesFromLastLoopClose(weaving_inspection::GetCurrentPulses::Request &req,
                                                  weaving_inspection::GetCurrentPulses::Response &res)
{
    int64_t current_pulses = cameraManager.getCounterValue();

    // Handle negative counter value, maximum pulse count based on (max_roll_length (meters))
    current_pulses = (current_pulses > max_roll_length_m * 1000 * pulse_per_mm) ? 0 : current_pulses;

    if (last_loop_close_pulse_value > current_pulses)
    {
        if (last_loop_close_pulse_value - current_pulses > 1000)
        {
            if (interval_task_overflow_logger.isReady())
            {
                interval_task_overflow_logger.reset();
                addSystemLog("WARNING", "Reverse or Overflow in Camera Pulses! Current: " + std::to_string(current_pulses) + " LastLoopClose: " + std::to_string(last_loop_close_pulse_value) + "Difference: " + std::to_string(current_pulses - last_loop_close_pulse_value), "CAM013");
            }
        }
        res.pulses = 0;
        return false;
    }
    res.pulses = (current_pulses - last_loop_close_pulse_value);
    return true;
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
    ros::init(argc, argv, "Camera Node");
    CameraNode camera_node;
    return 0;
}
