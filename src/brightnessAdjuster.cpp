/**
 * @file brightnessAdjuster.cpp
 * @author Mohandass (mohan@robrosystems.com)
 * @brief Definations of brightness adjustment of camera.
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <brightnessAdjuster.hpp>

/**
 * @brief Construct a new Brightness Checker:: Brightness Checker object
 *
 */
BrightnessAdjuster::BrightnessAdjuster()
{
    // Read the parameters from the launch file.
    ros::NodeHandle n;

    if (!(ros::param::get("~ideal_brightness", ideal_brightness) &&
          ros::param::get("~brightness_tolerance", brightness_tolerance) &&
          ros::param::get("~min_exposure_value", min_exposure_limit) &&
          ros::param::get("~max_exposure_value", max_exposure_limit)))
    {
        std::cout << "\033[1;31m ERROR : \033[0m[Brightness Adjuster] Couldn't read some params from the launch file. Check their names and types!";
    }

    // Initialize the publisher and subscriber.
    exposure_value_pub = n.advertise<std_msgs::Int16>("/gui/value/exposure", 1);
    display_values_pub = n.advertise<std_msgs::String>("/gui/value/brightness_data", 1);
    metadata_change_sub = n.subscribe("/gui/onchange/metadata", 1, &BrightnessAdjuster::metaDataChangeCallback, this);
    input_image_sub = n.subscribe("/brightness_checker/image", 1, &BrightnessAdjuster::inputImageCallback, this);
}

/**
 * @brief Calculate brightness in HSV space.
 *
 * @param image - input image
 * @return float - HSV brightness value
 */
float BrightnessAdjuster::hsvBrightness(cv::Mat &image)
{
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);
    cv::Mat v_channel = channels[2];
    cv::Scalar mean = cv::mean(v_channel);
    return mean.val[0];
}

/**
 * @brief Calculate brightness in LAB space.
 *
 * @param image - input image
 * @return float- LAB brightness value
 */
float BrightnessAdjuster::labBrightness(cv::Mat &image)
{
    cv::Mat labImage;
    cv::cvtColor(image, labImage, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> channels;
    cv::split(labImage, channels);
    cv::Mat l_channel = channels[0];
    cv::Scalar mean = cv::mean(l_channel);
    return mean.val[0];
}

/**
 * @brief Calculate the brightness in HSV and LAB and return average
 *
 * @param image - input image
 * @return float - average brightness value
 */
float BrightnessAdjuster::calculateBrightness(cv::Mat image)
{
    cv::Mat input_img;
    if (image.type() == CV_8UC1)
    {
        cv::cvtColor(image, input_img, cv::COLOR_GRAY2BGR);
    }
    else
    {
        input_img = image;
    }

    float h_brightness = hsvBrightness(input_img);
    float l_brightness = labBrightness(input_img);

    float avg_brightness = 0.5 * h_brightness + 0.5 * l_brightness;
    return avg_brightness / 255.0; // for percentage
}

/**
 * @brief Get the ROI from the input image.Which is the fabric region from the input image.
 *
 * @param gray - input image
 * @param roi - output ROI
 * @return true - if ROI is found
 * @return false - if ROI is not found
 */
bool BrightnessAdjuster::getROI(cv::Mat gray, cv::Rect &roi)
{
    bool roi_found = false;
    int min_dimension = std::min(gray.cols, gray.rows);

    // Contours to store the detected contours.
    std::vector<std::vector<cv::Point>> contours;

    // Change color to gray scale.
    if (gray.channels() > 1)
    {
        cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    }

    // Remove noise using blur filter
    cv::medianBlur(gray, gray, 5);

    // Apply adaptive threshold to enhance the vertical edges of slot conveyor.
    cv::adaptiveThreshold(gray, gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2);

    // calculate the structure.
    cv::Mat verticalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 50));

    // Remove more noise.
    cv::dilate(gray, gray, verticalStructure);
    cv::erode(gray, gray, verticalStructure);

    // find the contours.
    cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // choose the contours which are bigger in size 450px.
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Rect rect = cv::boundingRect(contours[i]);

        if (rect.height > min_dimension / 2 & rect.width > min_dimension / 2)
        {
            roi = rect;
            roi_found = true;
            break;
        }
    }

    return roi_found;
}

/**
 * @brief Callback function for the input image.
 *
 * @param msg - input image
 */
void BrightnessAdjuster::inputImageCallback(const sensor_msgs::Image &msg)
{
    execution_timer.start();

    // if we've already adjusted the brightness, then don't do anything.
    if (brightness_adjusted)
    {
        return;
    }
    // get the current exposure value.
    int current_exposure_value = std::stoi(msg.header.frame_id);
    // increase the brightness adjustment counter.
    ++brightness_adjustment_counter;

    // we've tried multiple times and still we're trying, so let's give up! Ask user to do it manually.
    if (brightness_adjustment_counter > 20 && !send_pop_once)
    {
        brightness_adjusted = true;
        send_pop_once = true;
        Notifier::getInstance("Code APP017: Auto-Brightness Failed.", "popup");
        addSystemLog("WARNING", "Auto-Brightness Failed. Please set manually.", "APP017");
    }

    // convert the image to opencv format.
    // ROS_INFO("Image Received in Brightness Checker!");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // get the ROI from the image.
    cv::Rect fabric_region;
    float adjusted_exposure_value{0.0f};

    bool get_roi_success = getROI(cv_ptr->image, fabric_region);
    float brightness = 0.0f;

    if (get_roi_success)
    {
        // calculate the brightness of the ROI.
        brightness = calculateBrightness(cv_ptr->image(fabric_region));
        // calculate the difference between the ideal brightness and the current brightness.
        float deviation_from_ideal = ideal_brightness - brightness;
        // A negative deviation_from_ideal means img is too bright, exposure needs to be reduced
        if (abs(deviation_from_ideal) > brightness_tolerance)
        {
            adjusted_exposure_value = (deviation_from_ideal * (max_exposure_limit - min_exposure_limit)) + min_exposure_limit;

            // publish the adjusted exposure value.
            exposure_value_msg.data = (int)adjusted_exposure_value + current_exposure_value;

            if (exposure_value_msg.data > max_exposure_limit)
            {
                exposure_value_msg.data = max_exposure_limit;
            }

            if (exposure_value_msg.data < min_exposure_limit)
            {
                exposure_value_msg.data = min_exposure_limit;
            }

            exposure_value_pub.publish(exposure_value_msg);
        }
        else
        {
            brightness_adjusted = true;
        }
    }
    else
    {
        ROS_ERROR("Not able to find the fabric for brightness adjustment!");
        addSystemLog("WARNING", "Not able to find the fabric for brightness adjustment!", "APP017");
    }

    execution_timer.stop();

    std_msgs::String display_values_msg;
    json brightness_data;

    brightness_data["brightness_adjusted"] = (brightness_adjusted && !send_pop_once);
    brightness_data["roi_success"] = get_roi_success;
    brightness_data["brightness_adjustment_counter"] = brightness_adjustment_counter;
    brightness_data["requested_exposure"] = exposure_value_msg.data;
    brightness_data["current_brightness"] = brightness;
    brightness_data["ideal_brightness"] = ideal_brightness;
    brightness_data["brightness_tolerance"] = brightness_tolerance;
    brightness_data["execution_time"] = execution_timer.getLastDurationInMilliseconds();
    brightness_data["max_exposure"] = max_exposure_limit;
    brightness_data["min_exposure"] = min_exposure_limit;

    display_values_msg.data = brightness_data.dump();
    display_values_pub.publish(display_values_msg);

    execution_timer.reset();
}

/**
 * @brief Subscribe to the metadata topic to reset the brightness adjustment.
 *
 * @param msg - metadata message.
 */
void BrightnessAdjuster::metaDataChangeCallback(const std_msgs::String &msg)
{
    json metadata = json::parse(msg.data);
    if (metadata.contains("is_roll_started") && metadata["is_roll_started"])
    {
        brightness_adjusted = false;
        brightness_adjustment_counter = 0;
        send_pop_once = false;
        Notifier::getInstance("Adjusting brightness.Please Wait.....", "warning");
    }
}

/**
 * @brief Add the system log to into system log server.
 *
 * @param severity - Severity of the log.
 * @param msg - Message to be logged.
 */
void BrightnessAdjuster::addSystemLog(std::string severity, std::string msg, std::string msg_code)
{
    // System log needs a json with severity and msg.
    logger.add_log(getLogSeverityFromString(severity), getLogCodeFromString(msg_code), msg, LogComponent::APP);
}

/**
 * @brief Main function
 *
 * @param argc - number of arguments
 * @param argv - arguments
 * @return int - 0
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Brightness Adjuster");
    BrightnessAdjuster adjust;
    ros::spin();
    return 0;
}