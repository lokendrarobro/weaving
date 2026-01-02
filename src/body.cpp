/**
 * @file body.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Body class definition which is used to store the data of a body in system.
 * @version 1.0
 * @date 2023-05-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <body.hpp>

#ifndef BODY_CPP
#define BODY_CPP

/**
 * @brief Construct a new Body object
 */
Body::Body() : BaseType("body"){};

/**
 * @brief Convert the body to json
 *
 * @return json - The body in json format
 */
json Body::toJSON()
{
    json body;
    body["body_id"] =body_id;
    body["job_id"] = job_id;
    body["robro_roll_id"] = robro_roll_id;
    body["actual_cut_length"] = actual_cut_length;
    body["body_cut_type"] = body_cut_type;
    body["estimated_length_saved"] = estimated_length_saved;
    body["balance_roll_length"] = balance_roll_length;
    body["punch_saved"] = punch_saved;
    body["cut_position_in_roll"] = cut_position_in_roll;
    body["updated_at"]=updated_at;
    body["work_order_id"]=work_order_id;
    return body;
}

/**
 * @brief Load the body from json.
 *
 * @param body - The body in json format
 * @return true - If the body is loaded successfully.
 * @return false - If the body is not loaded successfully.
 */
bool Body::loadJSON(json body)
{
    // Declare data
    int64_t _body_id;
    int64_t _job_id;
    int64_t _robro_roll_id;
    int _actual_cut_length;
    std::string _body_cut_type;
    float _estimated_length_saved;
    float _balance_roll_length;
    bool _punch_saved;
    float _cut_position_in_roll;
    std::string _updated_at;
    int64_t _work_order_id;


    // Parse required fields
    try
    {
        _body_id = body.at("body_id").get<int64_t>();
        _job_id = body.at("job_id").get<int64_t>();
        _robro_roll_id = body.at("robro_roll_id").get<int64_t>();
        _actual_cut_length = body.at("actual_cut_length").get<int>();
        _body_cut_type = body.at("body_cut_type").get<std::string>();
        _estimated_length_saved = body.at("estimated_length_saved").get<float>();
        _balance_roll_length = body.at("balance_roll_length").get<float>();
        _punch_saved = body.at("punch_saved").get<bool>();
        _cut_position_in_roll = body.at("cut_position_in_roll").get<float>();
        _updated_at = body.at("updated_at").get<std::string>();
        _work_order_id = body.at("work_order_id").get<int64_t>();

    }
    catch (const std::exception &e)
    {
        std::cout << "[Body] Invalid or missing field in JSON object: " << std::string(e.what());
        return false;
    }

    // Update object
    body_id=_body_id;
    job_id = _job_id;
    robro_roll_id = _robro_roll_id;
    actual_cut_length = _actual_cut_length;
    body_cut_type = _body_cut_type;
    estimated_length_saved = _estimated_length_saved;
    balance_roll_length = _balance_roll_length;
    punch_saved = _punch_saved;
    cut_position_in_roll = _cut_position_in_roll;
    updated_at=_updated_at;
    work_order_id=_work_order_id;
    return true;
}

#endif // BODY_CPP