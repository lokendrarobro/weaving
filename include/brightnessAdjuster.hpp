/**
 * @file brightnessAdjuster.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Blue print of brightness adjustment of camera.
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) 2023
 *
 */

// Opencv library imports.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// Custom Includes
#include <notifier.hpp>
#include <mysqlClient.hpp>
#include <executionTimer.hpp>
#include <logMessage.hpp>

/**
 * @brief Class to adjust the camera exposure for proper image brightness.
 *
 */
class BrightnessAdjuster
{

    /**
     * @brief Indicates if the camera exposure has been adjusted for proper image brightness.
     */
    bool brightness_adjusted{false};

    /**
     * @brief Use to send the pop up message once. in case of brightness adjustment failure.
     */
    bool send_pop_once{false};

    /**
     * @brief Stores the number of times the camera exposure has been adjusted.
     */
    int brightness_adjustment_counter{0};

    /**
     * @brief Stores the ideal brightness value. We want the image brightness
     * to be around this value.
     */
    float ideal_brightness{0.5f};

    /**
     * @brief Stores the tolerance value for the brightness. The brightness
     * should be within this range of the ideal brightness.
     */
    float brightness_tolerance{0.1f};

    /**
     * @brief Stores the minimum exposure value for the camera.
     */
    float min_exposure_limit{20.0f};

    /**
     * @brief Stores the maximum exposure value for the camera.
     */
    float max_exposure_limit{300.0f};

    /**
     * @brief Brightness Adjuster Node publisher to publish following topics.
     * 1. exposure_value: To publish the exposure value.
     */
    ros::Publisher exposure_value_pub, display_values_pub;

    /**
     * @brief Brightness Adjuster Node subscriber to subscribe following topics.
     * 1. input_image: To subscribe the input image.
     * 2. metadata_change: To subscribe the roll change topic.
     */
    ros::Subscriber input_image_sub, metadata_change_sub;

    /**
     * @brief Message to publish the exposure value.
     */
    std_msgs::Int16 exposure_value_msg;

    /**
     * @brief system logger object. Which is used to log the messages.
     */
    MySQLClient& logger = MySQLClient::getInstance();
    
    /**
     * @brief Execution Timer object to calculate the execution time.
     */
    ExecutionTimer execution_timer;

public:
    /**
     * @brief Construct a new Brightness Checker object
     *
     */
    BrightnessAdjuster();

    /**
     * @brief Calculate brightness in HSV space.
     *
     * @param image - input image
     * @return float - HSV brightness value
     */
    float hsvBrightness(cv::Mat &image);

    /**
     * @brief Calculate brightness in LAB space.
     *
     * @param image - input image
     * @return float- LAB brightness value
     */
    float labBrightness(cv::Mat &image);

    /**
     * @brief Calculate the brightness in HSV and LAB and return average
     *
     * @param image - input image
     * @return float - average brightness value
     */
    float calculateBrightness(cv::Mat image);

    /**
     * @brief Get the ROI from the input image.Which is the fabric region from the input image.
     *
     * @param gray - input image
     * @param roi - output ROI
     * @return true - if ROI is found
     * @return false - if ROI is not found
     */
    bool getROI(cv::Mat gray, cv::Rect &roi);

    /**
     * @brief  Function to subscribe to the topics and work as a callback function are as follows.
     * 1. inputImageCallback: To subscribe the input image.
     * 2. metaDataChangeCallback: To subscribe the metadata change topic.
     */
    void inputImageCallback(const sensor_msgs::Image &msg);

    /**
     * @brief meta data callback
     *
     * @param msg
     */
    void metaDataChangeCallback(const std_msgs::String &msg);

    /**
     * @brief Add the system log to into system log server.
     *
     * @param severity - Severity of the log.
     * @param msg - Message to be logged.
     */
    void addSystemLog(std::string severity, std::string msg, std::string msg_code);
};
