/**
 * @file defect.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Defect class blue print which is used to store the data of a defect in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <baseType.hpp>

#ifndef DEFECT_HPP
#define DEFECT_HPP

/**
 * @brief The defect class is used to store the data of a defect.
 *
 */
class Defect : public BaseType
{

public:
    /**
     * @brief The ID of the defect.
     */
    int64_t defect_id{0};

    /**
     * @brief The ID of the roll used by the system.
     */
    int64_t robro_roll_id{0};

    /**
     * @brief The ID of group for width calculation
     */
    int group_id{0};

    /**
     * @brief The defect belongs to which camera
     */
    int cam_id{0};

    /**
     * @brief The x-coordinate of the top-left corner of the defect bounding box.
     */
    float defect_top_left_x_mm{0.0f};

    /**
     * @brief The y-coordinate of the top-left corner of the defect bounding box.
     */
    float defect_top_left_y_mm{0.0f};

    /**
     * @brief The width of the defect
     */
    float defect_width_mm{0.0f};

    /**
     * @brief The height of the defect
     */
    float defect_height_mm{0.0f};

    /**
     * @brief The type of the defect.
     */
    std::string defect_type{""};

    /**
     * @brief The confidence of the defect detection algorithm.
     */
    float confidence{0.0f};

    /**
     * @brief it define the path of the cropped img
     */
    std::string cropped_image_path{""};

    /**
     * @brief it define the full image path
     */
    std::string full_image_path{""};

    /**
     * @brief The y-coordinate of the top-left corner of the defect bounding box.
     */
    int defect_top_left_x_px{0};

    /**
     * @brief The y-coordinate of the top-left corner of the defect bounding box.
     */
    int defect_top_left_y_px{0};

    /**
     * @brief The width of the defect
     */
    int defect_width_px{0};

    /**
     * @brief The height of the defect
     */
    int defect_height_px{0};

    /**
     * @brief the defect is enble or nor
     */
    bool is_enabled{0};

    /**
     * @brief body id of the defect
     */
    uint64_t body_id{0};

    /**
     * @brief Whether the operator took any action on the defect.
     */

    bool operator_action{false};

    /**
     * @brief The command issued to stop the system.
     */

    int stopping_command_issued{0};
    
    /**
     * @brief the sensitivity_x of the defect.
     */
    int sensitivity_x{0};

    /**
     * @brief the sensitivity_y of the defect.
     */
    int sensitivity_y ;

    /**
     * @brief the required sensitivity_x of the defect.
     */
    int required_sensitivity_x  ;

    /**
     * @brief the required sensitivity_y of the defect.
     */
    int required_sensitivity_y;

    std::string updated_at{""};

    int merge_id{0};

    int delete_status{0};

    int suggest_for_deletion{0};

    int splice_id{0};

    int repair_status{0};

    int model_id{0};

    std::string ai_suggestion{""};

    std::string user_suggestion{""};

    int merge_status{0};

    /**
     * @brief Construct a new Defect object
     */
    Defect();

    /**
     * @brief Initialize the defect.
     *
     * @param _defect_id - The ID of the defect.
     * @param _robro_roll_id - The ID of the roll used by the system.
     * @param _full_image_path - The path to the image containing the defect.
     * @param _defect_width_mm - The width of the defect bounding box.
     * @param _defect_height_mm - The height of the defect bounding box.
     * @param _defect_top_left_x_mm - The y-coordinate of the top-left corner of the defect bounding box.
     * @param _defect_top_left_y_mm - The x-coordinate of the top-left corner of the defect bounding box.
     * @param _defect_type -the type of defct
     * @param _defect_width_px - The width of the defect bounding box.
     * @param _defect_height_px - The height of the defect bounding box.
     * @param _defect_top_left_x_px - The y-coordinate of the top-left corner of the defect bounding box.
     * @param _defect_top_left_y_px - The x-coordinate of the top-left corner of the defect bounding box.
     * @param _body_id -the id of the defected body
     * @param _confidence -confidence of the defect
     * @param _defect_type - The type of the defect.
     */
    void initDefect(int64_t _defect_id, int64_t _robro_job_id,int cam_id,int group_id,
        std::string _cropped_image_path,std::string _full_image_path,bool _is_enabled, float _defect_width_mm, float _defect_height_mm,
        float _defect_top_left_y_mm , float _defect_top_left_x_mm, 
        float _defect_width_px, float _defect_height_px,
        float _defect_top_left_y_px , float _defect_top_left_x_px, int _sensitivity_x , int _sensitivity_y,
        int _required_sensitivity_x ,int _required_sensitivity_y,
        std::string _defect_type, float _confidence);


    /**
     * @brief Convert the defect to json.
     *
     * @return json - The defect in json format
     */
    json toJSON();

    /**
     * @brief Load the defect from json.
     *
     * @param defect - The defect in json format.
     * @return true - If the defect is loaded successfully.
     * @return false - If the defect is not loaded successfully.
     */
    bool loadJSON(json defect);
};

#endif // DEFECT_HPP