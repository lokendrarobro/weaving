/**
 * @file cameraHighSpeedSimulator.hpp
 * @author mohandass (mohan@robrosystems.com)
 * @brief maintaines the image queue by reading the image using camera simulator
 *  and publish the image on service request.
 * @version 0.1
 * @date 2023-04-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <queue>

#include <cameraSimulation.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>
#include <weaving_inspection/GetSimulationImage.h>

/**
 * @brief class to grab images and maintain the queue
 *
 */
class CameraHighSpeedSimulator
{
public:
    /**
     * @brief simulation images variables
     *
     */
    std::string simulation_images_path, image_path_prefix, image_path_suffix, camera_node_number;

    /**
     * @brief Service Server
     *
     */
    ros::ServiceServer simulation_image_service;

    /**
     * @brief Read images using camerasimulation and fill in this queue
     *
     */
    std::queue<cv::Mat> acquired_image_queue;

    /**
     * @brief camera object
     *
     */
    RobroCameraAbstractClass *camera;

    /**
     * @brief Construct a new Camera High Speed Simulator object
     *
     */
    CameraHighSpeedSimulator();

    /**
     * @brief Destroy the Camera High Speed Simulator object
     *
     */
    ~CameraHighSpeedSimulator();

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    /**
     * @brief Get the image from the queue and publish
     *
     * @param req Empty
     * @param res Image
     * @return true when success.
     * @return false if the image queue is empty.
     */
    bool getImageServiceCallback(weaving_inspection::GetSimulationImage::Request &req, weaving_inspection::GetSimulationImage::Response &res);

};