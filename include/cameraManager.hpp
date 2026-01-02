/**
 * @file cameraManager.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Class to interact with all camera related things in KWIS
 * @version 1.0
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023 Robro Systems Private Limited
 *
 */
// System Includes

#ifndef CAMERAMANAGER_HPP
#define CAMERAMANAGER_HPP

#include <robroCameraAbstractClass.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <weaving_inspection/GetSimulationImage.h>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Structure to store the camera parameters. It has all the information about the camera.
 */
struct CameraParams
{
    /**
     * @brief It indicates whether the system is running in simulation mode or not.
     * If it is set to true, the system is in simulation mode, otherwise it is in real mode.
     */
    bool is_simulation{false};

    /**
     * @brief It indicates whether or not the camera node is designated as the master camera,
     * which is the only camera that can maintain camera counters.
     */
    bool is_master{false};

    /**
     * @brief It indicates whether the image captured by the camera is valid or not.
     */
    bool is_valid_img{false};

    /**
     * @brief It indicates whether the camera is initialized or not.
     */
    bool is_initialized{false};

    /**
     * @brief It indicates whether the simulation mode is active with a camera connected.
     * If its value is true, then the simulation is running in a mode where a camera is connected
     * and its counter is being processed along with the simulated environment.
     */
    bool is_simulation_with_cam_connected{false};

    /**
     * @brief It indicates whether the camera is a top camera or not. Which means it is a camera
     * that is mounted on the top of the fabric.otherwise it is a bottom camera.
     */
    bool is_top_camera{true};

    /**
     * @brief It indicates the camera Id of the camera in the system that uniquely identifies it.
     */
    std::string id;
    
    /**
     * @brief It indicates the camera Id of the camera in the system that uniquely identifies it.
     */
    int group_id;

    /**
     * @brief It indicates the brand of the camera in the system
     * Which can be either Dalsa or HikRobot.
     */
    std::string brand;

    /**
     * @brief It indicate the specific model of the dalsa camera.
     * Which can be either Linea or Linea_Lite.
     */
    std::string dalsa_model;

    /**
     * @brief It indicates the serial number of the camera.
     */
    std::string serial_number;

    /**
     * @brief It indicates the IP address of the camera.
     * Which is unique for each camera in the system network.
     */
    std::string ip_address;

    /**
     * @brief It stores the camera position in the system.
     * Which can be either Left, Right, Top or Bottom.
     */
    std::string position; // camera position

    /**
     * @brief It store the number of images captured by the camera as a frame counter.
     */
    int frame_counter{0};

    /**
     * @brief It store the frame height of the image captured by the camera.
     */
    int frame_height{4096};

    /**
     * @brief It store the frame width of the image captured by the camera.
     */
    int frame_width{4096};

    /**
     * @brief It store the frame width of the image captured by the camera.
     */
    int exposure_us;

    /**
     * @brief It store the x start position of camera's field of view w.r.t the real world
     * in mm.
     */
    int fov_x_start_mm;

    /**
     * @brief It store the x end position of camera's field of view w.r.t the real world.
     * in mm.
     */
    int fov_x_end_mm;
};

/**
 * @brief A class that interacts with the camera and provides camera-related functionality.
 *
 */
class CameraManager
{
private:
    /**
     * @brief To Send the real world map flipped or same as generated.
     */
    bool is_flip_on{false};

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
     * @brief Topic on which images are published when running in camera_simulation
     */
    std::string simulation_image_service_name{""};

    /**
     * @brief It store the images suffix in the simulation mode.
     * which is used during the simulation for the image path.
     */
    std::string image_path_suffix;

    /**
     * @brief It store the images prefix in the simulation mode.
     * which is used during the simulation for the image path.
     */
    std::string image_path_prefix;

    /**
     * @brief It stores absolute path of the simulation images.
     */
    std::string simulation_images_path;

    /**
     * @brief It stores the camera parameters. It has all the information about the camera.
     */
    CameraParams params;

    /**
     * @brief Service Client to Call the Service to get images when in camera simulation w/ camera connected
     */
    ros::ServiceClient simulation_image_client;

    /**
     * @brief CV Bridge Pointer
     */
    cv_bridge::CvImagePtr cv_ptr;

    /**
     * Service Call client for the simulation image
     */
    weaving_inspection::GetSimulationImage simulation_img_srv;

    /**
     * @brief It points to the abstract camera class which is used to interact with the camera.
     * Which can be either Dalsa or HikRobot or camera simulation class.
     */
    RobroCameraAbstractClass *camera{nullptr};

public:
    /**
     * @brief Construct a new Camera Manager object.
     */
    CameraManager();

    /**
     * @brief Destroy the Camera Manager object.
     */
    ~CameraManager();

    /**
     * @brief It reads the parameters from the ros parameter server and
     * stores them in the class variables.
     *
     * @return true - if parameters are read successfully.
     * @return false - otherwise.
     */
    bool readROSParams();

    /**
     * @brief Resets the camera parameters to the default values.
     *
     * @return true - if the camera parameters are reset successfully.
     * @return false - otherwise.
     */
    bool resetCameraParams();

    /**
     * @brief Get the Camera Params object
     *
     * @return CameraParams - the camera parameters.
     */
    CameraParams getCameraParams();

    /**
     * @brief Get the Counter Value from the camera.
     * frame number is used to calculate the counter value in simulation mode.
     *
     * @param frame_number - the frame number.
     * @return int64_t - the counter value.
     */
    int64_t getCounterValue(int frame_number = 0);

    /**
     * @brief Reset the counter value of the camera.
     *
     * @return true - if the counter value is reset successfully.
     * @return false - otherwise.
     */
    bool resetCounterValue();

    /**
     * @brief Set the Exposure of the camera.
     *
     * @param exposure_us - the exposure value in microseconds (us).
     * @return true - if the exposure is set successfully.
     * @return false - otherwise.
     */
    bool setExposure(int exposure_us);

    /**
     * @brief It initializes the camera and returns if the camera is initialized or not.
     *
     * @return true - if the camera is initialized.
     * @return false - otherwise.
     */
    bool initCamera();

    /**
     * @brief It grabs the image from the camera and returns the camera parameters
     * along with the image.
     *
     * @param image - the image captured by the camera.
     * @return CameraParams - the camera parameters.
     */
    CameraParams grabImage(cv::Mat &image);
};

#endif // CAMERAMANAGER_HPP