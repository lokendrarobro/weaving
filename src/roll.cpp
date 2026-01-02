/**
 * @file roll.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Roll class definition which is used to store the data of a Roll in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <roll.hpp>

#ifndef ROLL_CPP
#define ROLL_CPP

/**
 * @brief Construct a new Roll object
 */
Roll::Roll() : BaseType("roll"){};

/**
 * @brief Convert the roll to JSON.
 *
 * @return json - The roll in JSON format.
 */
json Roll::toJSON()
{
    json roll;


    roll["robro_roll_id"]      = robro_roll_id;
    roll["customer_roll_id"]   = customer_roll_id;
    roll["machine_id"]         = machine_id;
    roll["gsm"]                = gsm;
    roll["weight"]             = weight;
    roll["width"]              = width;
    roll["material_type"]      = material_type;
    roll["quality_code"]       = quality_code;
    roll["roll_length"]        = roll_length;
    roll["inspected_length"]   = inspected_length;
    roll["total_defects"]      = total_defects;
    roll["roll_start_time"]    = roll_start_time;
    roll["roll_end_time"]      = roll_end_time;
    roll["total_defects"]      = total_defects;
    roll["loom_number"]        =loom_number;
    
    return roll;
}


/**
 * @brief Load the roll from json.
 *
 * @param roll - The roll in json format
 * @return true - If the roll is loaded successfully.
 * @return false - If the roll is not loaded successfully.
 */
bool Roll::loadJSON(json roll)
{
    // Parse required fields
    try
    {
        if (roll.contains("robro_roll_id") && !roll.at("robro_roll_id").is_null())
        {
            robro_roll_id = roll.at("robro_roll_id").get<std::int64_t>();
        }
        else
        {
            robro_roll_id = 0;
        }

        if (roll.contains("machine_id") && !roll.at("machine_id").is_null())
        {
            machine_id = roll.at("machine_id").get<std::string>();
        }
        else
        {
            machine_id = "null";
        }

        if (roll.contains("customer_roll_id") && !roll.at("customer_roll_id").is_null())
        {
            customer_roll_id = roll.at("customer_roll_id").get<std::string>();
        }
        else
        {
            customer_roll_id = "null";
        }

        if (roll.contains("gsm") && !roll.at("gsm").is_null())
        {
            gsm = roll.at("gsm").get<int>();
        }
        else
        {
            gsm = 0;
        }

        if (roll.contains("weight") && !roll.at("weight").is_null())
        {
            weight = roll.at("weight").get<float>();
        }
        else
        {
            weight = 0.0;
        }

        if (roll.contains("width") && !roll.at("width").is_null())
        {
            width = roll.at("width").get<float>();
        }
        else
        {
            width = 0.0;
        }

        if (roll.contains("material_type") && !roll.at("material_type").is_null())
        {
            material_type = roll.at("material_type").get<std::string>();
        }
        else
        {
            material_type = "null";
        }


        if (roll.contains("quality_code") && !roll.at("quality_code").is_null())
        {
            quality_code = roll.at("quality_code").get<std::string>();
        }
        else
        {
            quality_code = "null";
        }

        if (roll.contains("roll_length") && !roll.at("roll_length").is_null())
        {
            roll_length = roll.at("roll_length").get<float>();
        }
        else
        {
            roll_length = 0.0;
        }

        if (roll.contains("inspected_length") && !roll.at("inspected_length").is_null())
        {
            inspected_length = roll.at("inspected_length").get<float>();
        }
        else
        {
            inspected_length = 0.0;
        }

        if (roll.contains("roll_start_time") && !roll.at("roll_start_time").is_null())
        {
            roll_start_time = roll.at("roll_start_time").get<std::string>();
        }
        else
        {
            roll_start_time = "null";
        }

        if (roll.contains("roll_end_time") && !roll.at("roll_end_time").is_null())
        {
            roll_end_time = roll.at("roll_end_time").get<std::string>();
        }
        else
        {
            roll_end_time = "null";
        }

        if (roll.contains("loom_number") && !roll.at("loom_number").is_null())
        {
            loom_number = roll.at("loom_number").get<std::string>();
        }
        else
        {
            loom_number = "null";
        }
        

        if (roll.contains("total_defects") && !roll.at("total_defects").is_null())
        {
            total_defects = roll.at("total_defects").get<int>();
        }
        else
        {
            total_defects = 0;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Roll] Invalid or missing field in JSON object: " << std::string(e.what());
        return false;
    }
    return true;
}



#endif // ROLL_CPP