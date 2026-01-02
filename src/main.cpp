/**
 * @file weavingInspection.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Weaving Inspection definations for KWIS
 * @version 1.0
 * @date 2022-03-03
 * @updated 2022-08-26
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <weavingInspection.hpp>

/**
 * @brief Main Function Entry Point for Weaving Inspection.
 *
 * @param argc - Number of arguments
 * @param argv - Arguments
 * @return int - Return 0 on success
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "weaving_main");
    WeavingInspection weaving_inspection;

    ros::spin();

    ROS_WARN("Shutting Main..");

    return 0;
}
