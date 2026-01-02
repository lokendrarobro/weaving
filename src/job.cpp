/**
 * @file job.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Job class definition which is used to store the data of a Job in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <job.hpp>

#ifndef JOB_CPP
#define JOB_CPP

/**
 * @brief Construct a new Job object
 */
Job::Job() : BaseType("job") {};

/**
 * @brief Convert the job to json
 *
 * @return json - The job in json format
 */
json Job::toJSON()
{
    json job;
    job["job_id"] = job_id;
    job["robro_roll_id"] = robro_roll_id;
    job["cut_length"] = cut_length;
    job["recipe"] = recipe;
    job["secondary_cut_length"] = secondary_cut_length;
    job["tertiary_cut_length"] = tertiary_cut_length;
    job["start_time"] = start_time;
    job["end_time"] = end_time;
    job["primary_body_count"] = primary_body_count;
    job["secondary_body_count"] = secondary_body_count;
    job["tertiary_body_count"] = tertiary_body_count;
    job["defective_body_count"] = defective_body_count;
    job["batch_count"]   = batch_count;
    job["job_start_meter"] = job_start_meter;
    job["job_end_meter"] = job_end_meter;
    job["work_order_id"] = work_order_id;
    return job;
}

/**
 * @brief Load the job from json.
 *
 * @param job - The job in json format
 * @return true - If the job is loaded successfully.
 * @return false - If the job is not loaded successfully.
 */
bool Job::loadJSON(json job)
{
    int64_t _job_id;
    int64_t _robro_roll_id;
    int _cut_length;
    std::string _recipe;

    int _secondary_cut_length;
    int _tertiary_cut_length;

    std::string _start_time;
    std::string _end_time;

    int _primary_body_count;
    int _secondary_body_count;
    int _tertiary_body_count;
    int _defective_body_count;
    float _job_start_meter;
    float _job_end_meter;

    int64_t _work_order_id;



    int _batch_count;

    try
    {
        _job_id = job.at("job_id").get<int64_t>();    
        _robro_roll_id = job.at("robro_roll_id").get<int64_t>();
        _cut_length = job.at("cut_length").get<int>();
        _recipe = job.at("recipe").get<std::string>();
        _secondary_cut_length = job.value("secondary_cut_length", 0);
        _tertiary_cut_length = job.value("tertiary_cut_length", 0);

        _work_order_id = job.value("work_order_id", 0);
        _start_time = job.value("start_time", "");
        _end_time = job.value("end_time", "");
        _batch_count  = job.value("batch_count", 0);
        _primary_body_count = job.value("primary_body_count", 0);
        _secondary_body_count = job.value("secondary_body_count", 0);
        _tertiary_body_count = job.value("tertiary_body_count", 0);
        _defective_body_count = job.value("defective_body_count", 0);
        _job_start_meter = job.value("job_start_meter", 0.0f);
        _job_end_meter = job.value("job_end_meter", 0.0f);


    }
    catch (const std::exception &e)
    {
        std::cout << "[Job] Invalid or missing field in JSON object: " << std::string(e.what()) << std::endl;
        return false;
    }
    job_id = _job_id;
    robro_roll_id = _robro_roll_id;
    cut_length = _cut_length;
    recipe = _recipe;
    start_time = _start_time;
    end_time = _end_time;
    secondary_cut_length = _secondary_cut_length;
    tertiary_cut_length = _tertiary_cut_length;
    batch_count = _batch_count;
    primary_body_count = _primary_body_count;
    secondary_body_count = _secondary_body_count;
    tertiary_body_count = _tertiary_body_count;
    defective_body_count = _defective_body_count;
    job_start_meter = _job_start_meter;
    job_end_meter = _job_end_meter;
    work_order_id = _work_order_id;
    return true;
}

#endif // JOB_CPP