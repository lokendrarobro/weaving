/**
 * @file body.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Body class blue print which is used to store the data of a body in system.
 * @version 1.0
 * @date 2023-05-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <baseType.hpp>

#ifndef BODY_HPP
#define BODY_HPP

/**
 * @brief The body class is used to store the data of a body in system.
 *
 */
class Body : public BaseType
{

public:
    /**
     * @brief The ID of the body.
     */
    int64_t body_id{0};

    /**
     * @brief The ID of the job that the body belongs to.
     */
    int64_t job_id{0};

    /**
     * @brief The ID of the roll used by the systems.
     */
    int64_t robro_roll_id{0};

    /**
     * @brief The actual length of the cut body.
     */
    int actual_cut_length{0};

    /**
     * @brief The type of cut made on the body.
     */
    std::string body_cut_type{""};

    /**
     * @brief The estimated length saved by the systems cut
     */
    float estimated_length_saved{0.0f};

    /**
     * @brief The remaining length of the roll after the cut
     */
    float balance_roll_length{0.0f};

    /**
     * @brief Whether the systems was able to save a punch.
     */
    bool punch_saved{false};

    /**
     * @brief The position of the cut in the roll.
     */
    float cut_position_in_roll{0.0f};

    /**
     * @brief The position of the cut in the roll.
     */
    std::string updated_at{""};

        /**
     * @brief The position of the cut in the roll.
     */
    int64_t work_order_id{0};

    /**
     * @brief Construct a new Body object
     */
    Body();

    /**
     * @brief Convert the body to json
     *
     * @return json - The body in json format
     */
    json toJSON();

    /**
     * @brief Load the body from json.
     *
     * @param body - The body in json format
     * @return true - If the body is loaded successfully.
     * @return false - If the body is not loaded successfully.
     */
    bool loadJSON(json body);
};

#endif // BODY_HPP