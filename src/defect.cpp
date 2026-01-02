/**
 * @file defect.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Defect class definition which is used to store the data of a Defect in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <defect.hpp>

#ifndef DEFECT_CPP
#define DEFECT_CPP

/**
 * @brief Construct a new Defect object
 */
Defect::Defect() : BaseType("defect") {};

/**
 * @brief Initialize the defect object.
 *
 * @param _id - The ID of the defect.
 * @param _job_id - The ID of the job that the defect belongs to.
 * @param _roll_id - The ID of the roll used by the system.
 * @param _image_path - The path of the image of the defect.
 * @param _width - The width of the defect bounding box.
 * @param _height - The height of the defect bounding box.
 * @param _top_left_y - The y-coordinate of the top-left corner of the defect bounding box.
 * @param _top_left_x - The x-coordinate of the top-left corner of the defect bounding box.
 * @param _position_in_roll - The position of the defect in the roll.
 * @param _defect_type - The type of the defect.
 * @param _confidence - The confidence of the defect.
 */
void Defect::initDefect(int64_t _defect_id, int64_t _robro_job_id,int _cam_id , int _group_id,
                        std::string _cropped_image_path,std::string _full_image_path,bool _is_enabled, float _defect_width_mm, float _defect_height_mm,
                        float _defect_top_left_y_mm , float _defect_top_left_x_mm , float _defect_width_px, float _defect_height_px,
                        float _defect_top_left_y_px , float _defect_top_left_x_px, int _sensitivity_x , int _sensitivity_y,
                        int _required_sensitivity_x ,int _required_sensitivity_y,  std::string _defect_type, float _confidence) // <- no semicolon here
{
    defect_id = _defect_id;
    robro_roll_id = _robro_job_id; // fixed variable name typo here too
    cam_id=_cam_id;
    group_id=_group_id;
    cropped_image_path = _cropped_image_path;
    full_image_path = _full_image_path;
    is_enabled=_is_enabled;
    defect_width_mm = _defect_width_mm;
    defect_height_mm = _defect_height_mm;
    defect_top_left_x_mm = _defect_top_left_x_mm;
    defect_top_left_y_mm = _defect_top_left_y_mm;
    defect_width_px = _defect_width_px;
    defect_height_px = _defect_height_px;
    defect_top_left_x_px = _defect_top_left_x_px;
    defect_top_left_y_px = _defect_top_left_y_px;
    sensitivity_x=_sensitivity_x;
    sensitivity_y=_sensitivity_y;
    required_sensitivity_x=_required_sensitivity_x;
    required_sensitivity_y=_required_sensitivity_y;
    defect_type = _defect_type;
    confidence = _confidence;
}

/**
 * @brief Convert the defect to json
 *
 * @return json - The defect in json format
 */
json Defect::toJSON()
{
    json defect;
    defect["defect_id"] = defect_id;
    defect["robro_roll_id"] = robro_roll_id;
    defect["cropped_image_path"] = cropped_image_path;
    defect["full_image_path"] = full_image_path;
    defect["is_enabled"]=is_enabled;
    defect["defect_width_mm"] = defect_width_mm;
    defect["cam_id"]=cam_id;
    defect["group_id"]=group_id;
    defect["defect_height_mm"] = defect_height_mm;
    defect["defect_top_left_x_mm"] = defect_top_left_x_mm;
    defect["defect_top_left_y_mm"] = defect_top_left_y_mm;
    defect["defect_width_px"] = defect_width_px;
    defect["defect_height_px"] = defect_height_px;
    defect["defect_top_left_x_px"] = defect_top_left_x_px;
    defect["defect_top_left_y_px"] = defect_top_left_y_px;
    defect["body_id"] = body_id;
    defect["stopping_command_issued"]=stopping_command_issued;
    defect["defect_type"] = defect_type;
    defect["confidence"] = confidence;
    defect["sensitivity_x"]=sensitivity_x;
    defect["sensitivity_y"]=sensitivity_y;
    defect["required_sensitivity_x"]=required_sensitivity_x;
    defect["required_sensitivity_y"]=required_sensitivity_y;
    return defect;
}

/**
 * @brief Load the defect from json.
 *
 * @param defect - The defect in json format
 * @return true - If the defect is loaded successfully.
 * @return false - If the defect is not loaded successfully.
 */
bool Defect::loadJSON(json defect)
{
    // Declare data
    int64_t _defect_id;
    int64_t _robro_roll_id;
    std::string _cropped_image_path;
    std::string _full_image_path;
    bool _is_enabled;
    float _defect_width_mm;
    float _defect_height_mm;
    float _defect_top_left_x_mm;
    float _defect_top_left_y_mm;
    int _defect_width_px;
    int _cam_id;
    int _group_id;
    int _defect_height_px;
    int _defect_top_left_x_px;
    int _defect_top_left_y_px;
    int64_t _body_id;
    int _stopping_command_issued;
    std::string _defect_type;
    float _confidence;
    int _sensitivity_x;
    int _sensitivity_y;
    int _required_sensitivity_x;
    int _required_sensitivity_y;

    // Parse required fields
    try
    {
        _defect_id = defect.at("defect_id").get<int64_t>();
        _robro_roll_id = defect.at("robro_roll_id").get<int64_t>();
        _cropped_image_path = defect.at("cropped_image_path").get<std::string>();
        _full_image_path = defect.at("full_image_path").get<std::string>();
        _is_enabled =defect.at("is_enabled").get<bool>();
        _defect_width_mm = defect.at("defect_width_mm").get<float>();
        _defect_height_mm = defect.at("defect_height_mm").get<float>();
        _defect_top_left_x_mm = defect.at("defect_top_left_x_mm").get<float>();
        _defect_top_left_y_mm = defect.at("defect_top_left_y_mm").get<float>();
        _defect_width_px = defect.at("defect_width_px").get<int>();
        _defect_height_px = defect.at("defect_height_px").get<int>();
        _defect_top_left_x_px = defect.at("defect_top_left_x_px").get<int>();
        _defect_top_left_y_px = defect.at("defect_top_left_y_px").get<int>();
        _body_id = defect.at("body_id").get<int64_t>();
        _cam_id = defect.at("cam_id").get<int64_t>();
        _group_id = defect.at("group_id").get<int64_t>();
        _stopping_command_issued = defect.at("stopping_command_issued").get<int64_t>();
        _defect_type = defect.at("defect_type").get<std::string>();
        _confidence = defect.at("confidence").get<float>();
        _sensitivity_x=defect.at("sensitivity_x").get<int>();
        _sensitivity_y=defect.at("sensitivity_y").get<int>();
        _required_sensitivity_x=defect.at("required_sensitivity_x").get<int>();
        _required_sensitivity_y=defect.at("required_sensitivity_y").get<int>();
    }
    catch (const std::exception &e)
    {
        std::cout << "[Defect] Invalid or missing field in JSON object: " << e.what() << std::endl;
        return false;
    }

    // Update object
    defect_id = _defect_id;
    robro_roll_id = _robro_roll_id;
    cropped_image_path = _cropped_image_path;
    full_image_path = _full_image_path;
    is_enabled=_is_enabled;
    defect_width_mm = _defect_width_mm;
    cam_id=_cam_id;
    group_id=_group_id;
    defect_height_mm = _defect_height_mm;
    defect_top_left_x_mm = _defect_top_left_x_mm;
    defect_top_left_y_mm = _defect_top_left_y_mm;
    defect_width_px = _defect_width_px;
    defect_height_px = _defect_height_px;
    defect_top_left_x_px = _defect_top_left_x_px;
    defect_top_left_y_px = _defect_top_left_y_px;
    body_id = _body_id;
    stopping_command_issued=_stopping_command_issued;
    defect_type = _defect_type;
    confidence = _confidence;
    sensitivity_x=_sensitivity_x;
    sensitivity_y=_sensitivity_y;
    required_sensitivity_x=_required_sensitivity_x;
    required_sensitivity_y=_required_sensitivity_y;

    return true;
}

#endif // DEFECT_CPP