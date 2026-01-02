/**
 * @file cameraHighSpeedSimulator.cpp
 * @author mohandass (mohan@robrosystems.com)
 * @brief maintaines the image queue by reading the image using camera simulator
 *  and publish the image on service request.
 * @version 0.1
 * @date 2023-04-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cameraHighSpeedSimulator.hpp>

/**
 * Construct a new object
 */
CameraHighSpeedSimulator::CameraHighSpeedSimulator()
{
    ros::NodeHandle n;

    if (!(
            // private node parameters
            ros::param::get("~simulation_images_path", simulation_images_path) &&
            ros::param::get("~simulation_images_path_prefix", image_path_prefix) &&
            ros::param::get("~simulation_images_path_suffix", image_path_suffix) &&
            ros::param::get("~simulation_camera_node_number", camera_node_number)))

    {
        ROS_ERROR("[camera_simulator] Couldn't read some params from the launch file. Check their names and types!");
    }

    camera = new RobroCameraSimulation(simulation_images_path, image_path_prefix, image_path_suffix, false, true);

    // Start the service to deliver images
    simulation_image_service = n.advertiseService(camera_node_number, &CameraHighSpeedSimulator::getImageServiceCallback, this);
}

/**
 * Destructor
 */
CameraHighSpeedSimulator::~CameraHighSpeedSimulator()
{
}

/**
 * Service callback to deliver images from the queue
 */
bool CameraHighSpeedSimulator::getImageServiceCallback(weaving_inspection::GetSimulationImage::Request &req, weaving_inspection::GetSimulationImage::Response &res)
{
    if (!acquired_image_queue.empty())
    {
        img_bridge.encoding = "bgr8";
        img_bridge.image = acquired_image_queue.front();
        img_bridge.toImageMsg(img_msg);
        res.img = img_msg;
        acquired_image_queue.pop();
        return true;
    }
    else
    {
        ROS_ERROR("Image is not avaliable in the Queue!");
        return false;
    }
}

/**
 * @brief main to keep the node alive
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Brightness Checker");
    CameraHighSpeedSimulator camerasimulator;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    cv::Mat image;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (camerasimulator.acquired_image_queue.size() < 20)
        {
            camerasimulator.camera->grabNextImage(image);
            camerasimulator.acquired_image_queue.push(image.clone());
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        loop_rate.sleep();
    }
    spinner.stop();
    return 0;
}