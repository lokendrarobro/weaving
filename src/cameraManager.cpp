/**
 * @file cameraManager.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Class to interact with all camera related things in KWIS
 * @version 1.0
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023 Robro Systems Private Limited
 *
 */
// System Includes

#ifndef CAMERAMANAGER_CPP
#define CAMERAMANAGER_CPP

#include <cameraManager.hpp>

#include <dalsaCamera.h>
#include <hikRobotCamera.h>
#include <cameraSimulation.h>

/**
 * @brief Construct a new Camera Manager:: Camera Manager object
 *
 */
CameraManager::CameraManager()
{

    // Read all params from roslaunch file
    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[CameraManager] Could not read all params from ros launch file. Exiting.";
    }

    ROS_WARN("I'm Camera ID: %s and my Master Camera Status is: %d", params.id.c_str(), params.is_master);

    // Initialize Camera
    if (initCamera())
    {
        // Check if camera initialized or not.
        params.is_initialized = camera->isInitialized();
    }
}

/**
 * @brief Destroy the Camera Manager:: Camera Manager object
 *
 */
CameraManager::~CameraManager()
{
    // Free the memory allocated to the camera object.
    if (camera)
    {
        delete camera;
    }
}

/**
 * @brief It reads the parameters from the ros parameter server and
 * stores them in the class variables.
 *
 * @return true - if parameters are read successfully.
 * @return false - otherwise.
 */
bool CameraManager::readROSParams()
{
    ros::NodeHandle n;
    int cam_id_int;
    int master_cam_id_int;
    int group_id_int;
    if (!(
            // private members
            ros::param::get("~simulation_images_path", simulation_images_path) &&
            ros::param::get("~simulation_images_path_prefix", image_path_prefix) &&
            ros::param::get("~simulation_images_path_suffix", image_path_suffix) &&
            ros::param::get("~cam_ip_address", params.ip_address) &&
            ros::param::get("~cam_serial_number", params.serial_number) &&
            ros::param::get("~camera_brand", params.brand) &&
            ros::param::get("~this_camera_position", params.position) &&
            ros::param::get("~this_camera_id", cam_id_int) &&
            ros::param::get("~fov_x_start_mm", params.fov_x_start_mm) &&
            ros::param::get("~group_id",group_id_int)&&
            ros::param::get("~fov_x_end_mm", params.fov_x_end_mm) &&
            ros::param::get("~is_top_camera", params.is_top_camera) &&

            // non private members
            ros::param::get("simulation_mode_on", params.is_simulation) &&
            ros::param::get("simulation_mode_with_camera_connected", params.is_simulation_with_cam_connected) &&
            ros::param::get("master_camera_id", master_cam_id_int) &&
            ros::param::get("frame_width", params.frame_width) &&
            ros::param::get("frame_height", params.frame_height) &&
            ros::param::get("pixel_per_mm", pixel_per_mm) &&
            ros::param::get("pulse_per_mm", pulse_per_mm)))
    {
        return false;
    }

    if (!ros::param::get("~is_flip_on", is_flip_on))
    {
        is_flip_on = false;
    }

    if (params.is_simulation_with_cam_connected)
    {
        if (!ros::param::get("~simulation_image_service_name", simulation_image_service_name))
        {
            ROS_ERROR("We're in Simulation With Cam Connected Mode, but Simulation Image Service Name Not in launch file.");
        }
        else
        {
            simulation_image_client = n.serviceClient<weaving_inspection::GetSimulationImage>(simulation_image_service_name);
        }
    }

    if (params.brand == "dalsa")
    {
        if (!ros::param::get("~dalsa_camera_model", params.dalsa_model))
        {
            ROS_ERROR("Can't read Dalsa Camera Model from Launch File");
            return false;
        }
    }

    // Initialize Parameters.
    params.id = std::to_string(cam_id_int);
    params.group_id= group_id_int;
    params.is_master = (cam_id_int == master_cam_id_int);
    pulse_per_pixel = pulse_per_mm / pixel_per_mm;
    return true;
}

/**
 * @brief Resets the camera parameters to the default values.
 *
 * @return true - if the camera parameters are reset successfully.
 * @return false - otherwise.
 */
bool CameraManager::resetCameraParams()
{
    params.frame_counter = 0;
    return true;
}


/**
 * @brief Get the Camera Params object
 *
 * @return CameraParams - the camera parameters.
 */
CameraParams CameraManager::getCameraParams()
{
    return params;
}

/**
 * @brief Get the Counter Value from the camera.
 * frame number is used to calculate the counter value in simulation mode.
 *
 * @param frame_number - the frame number.
 * @return int64_t - the counter value.
 */
int64_t CameraManager::getCounterValue(int frame_number)
{
    if (!params.is_simulation || params.is_simulation_with_cam_connected)
    {
        return camera->readCounter1Value();
    }

    return params.frame_height * pulse_per_pixel * frame_number;
}

/**
 * @brief Reset the counter value of the camera.
 *
 * @return true - if the counter value is reset successfully.
 * @return false - otherwise.
 */
bool CameraManager::resetCounterValue()
{
    if (!params.is_simulation || params.is_simulation_with_cam_connected)
    {
        camera->resetCounter1Value();
        return true;
    }

    return true;
}

/**
 * @brief Set the Exposure of the camera.
 *
 * @param exposure_us - the exposure value in microseconds (us).
 * @return true - if the exposure is set successfully.
 * @return false - otherwise.
 */
bool CameraManager::setExposure(int exposure_us)
{
    if (exposure_us < 0)
    {
        ROS_ERROR("Exposure value cannot be negative");
        return false;
    }

    params.exposure_us = exposure_us;

    if (!params.is_simulation)
    {
        camera->setExposure_us(params.exposure_us);
    }
    return true;
}

/**
 * @brief It initializes the camera and returns if the camera is initialized or not.
 *
 * @return true - if the camera is initialized.
 * @return false - otherwise.
 */
bool CameraManager::initCamera()
{
    if (params.is_simulation && !params.is_simulation_with_cam_connected)
    {
        // Simulation camera
        camera = new RobroCameraSimulation(simulation_images_path, image_path_prefix, image_path_suffix, true);
        return true;
    }
    if (params.brand == "hikrobot")
    {
        camera = new HikRobotCamera(params.serial_number, "MONO8");
        return true;
    }
    if (params.brand == "dalsa")
    {
        if (params.dalsa_model == "linea_lite")
        {
            camera = new DalsaCamera(params.frame_width, params.frame_height, params.ip_address, "LINEA_LITE");
            return true;
        }
        if (params.dalsa_model == "linea")
        {
            camera = new DalsaCamera(params.frame_width, params.frame_height, params.ip_address, "LINEA");
            return true;
        }
        ROS_ERROR("Unidentified Dalsa Camera Model: %s", params.dalsa_model.c_str());
        return false;
    }

    ROS_ERROR("Camera Not Recognised (%s)", params.brand.c_str());
    return false;
}

/**
 * @brief It grabs the image from the camera and returns the camera parameters
 * along with the image.
 *
 * @param image - the image captured by the camera.
 * @return CameraParams - the camera parameters.
 */
CameraParams CameraManager::grabImage(cv::Mat &image)
{

    // get the image from the camera.
    params.is_valid_img = camera->grabNextImage(image);

    if (params.is_simulation_with_cam_connected)
    {
        // Call the service to get an image in sim mode with cam connected
        params.is_valid_img = simulation_image_client.call(simulation_img_srv);
        if (params.is_valid_img)
        {
            cv_ptr = cv_bridge::toCvCopy(simulation_img_srv.response.img, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image.clone();
        }
    }

    if (is_flip_on)
    {
        cv::flip(image, image, 1); // flip the image horizontally
    }

    // update the frame counter value.
    params.frame_counter++;

    // sleep for 3 seconds in simulation mode.
    if (params.is_valid_img && params.is_simulation && !params.is_simulation_with_cam_connected)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    return params;
}

#endif