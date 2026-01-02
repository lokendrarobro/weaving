
/**
 * @file job.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Job class blue print which is used to store the data of a job in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <baseType.hpp>

#ifndef JOB_HPP
#define JOB_HPP

/**
 * @brief The job class is used to store the data of a job.
 *
 */
class Job : public BaseType
{

public:
    /**
     * @brief The ID of the job.
     */
    int64_t job_id{0};

    /**
     * @brief The ID of the roll used by the systems.
     */
    int64_t robro_roll_id{0};

    /**
     * @brief The length of the cut body.
     */
    int cut_length{0};

    float min_fabric_width{0.0f};

    float max_fabric_width{0.0f};

    float width_measurement_rate{0.0f};

    /**
     * @brief The recipe_name used for defect detection algorithm.
     */
    std::string recipe{""};

    std::string updated_at{""};

    std::string start_time{""};

    std::string end_time{""};

    int user_id{0};

    int secondary_cut_length{0};

    int tertiary_cut_length{0};

    /**
     * @brief The Batch count
     */
    int batch_count{0};

    float job_start_meter{0.0f};

    float job_end_meter{0.0f};
    int primary_body_count{0};
    int secondary_body_count{0};
    int tertiary_body_count{0};
    int defective_body_count{0};
    int work_order_id{0};

    Job();

    /**
     * @brief Convert the job to json
     *
     * @return json - The job in json format
     */
    json toJSON();

    /**
     * @brief Load the job from json.
     *
     * @param  job - The job in json format
     * @return true - If the job is loaded successfully.
     * @return false - If the job is not loaded successfully.
     */
    bool loadJSON(json job);
};

#endif // JOB_HPP