#include <json.hpp>

#ifndef KWIS_RECIPE_HPP
#define KWIS_RECIPE_HPP

/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Store the recipe for KWIS application. Which help to manage the network and job.
 */
struct Recipe
{
    /**
     * @brief It stores the recipe available or not.
     */
    bool is_available{false};

    /**
     * @brief It stores the tiling option for the recipe.
     */
    bool tiling{false};

    /**
     * @brief It stores the recipe name.
     */
    std::string name;

    /**
     * @brief It stores the network type for the recipe.
     * which can be either yolov.
     */
    std::string network_type;

    /**
     * @brief It stores the trt network file path for the recipe.
     */
    std::string engine_file_path;

    /**
     * @brief It stores the trt network file path for the recipe.
     */
    std::string trt_file_path;

    /**
     * @brief It stores the yolov network weights file path for the recipe.
     */
    std::string weights_file_path;

    /**
     * @brief It stores the yolov network names file path for the recipe.
     */
    std::string names_file_path;

    /**
     * @brief It stores the yolov network cfg file path for the recipe.
     */
    std::string cfg_file_path;

    /**
     * @brief It stores the default exposure for the recipe.
     */
    int exposure{0};

    /**
     * @brief It stores the number of classes for the recipe.
     */
    int number_of_classes{0};

    /**
     * @brief It store the default threshold for the recipe.
     */
    float threshold{0.0};

    /**
     * @brief It stores the default nms threshold for the recipe.
     */
    float nms_threshold{0.0};

    /**
     * @brief It stores the default defect area for the recipe to consider as defect.
     */
    int min_defect_area_px{0};

    /**
     * @brief Includes the groups for the recipe.It is used to group the classes for the recipe to consider as a single class.
     * Following is the example of the groups:
     * {
     *    "<group_id>": {
     * "ids":[group_id,class_id1, class_id2, class_id3, ...],
     * "sensitivity": 0-10,
     * "enabled": true/false,
     *    }
     * }
     */
    json groups;

    /**
     * @brief It stores defect image cropping margin
     */
    int cropping_margin;
};

#endif // RECIPE_HPP
