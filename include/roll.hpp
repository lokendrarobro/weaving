/**
 * @file roll.hpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief Roll class blue print which is used to store the data of a Roll in system.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <baseType.hpp>

#ifndef ROLL_HPP
#define ROLL_HPP

/**
 * @brief The roll class is used to store the data of a roll.
 *
 */
class Roll : public BaseType
{
public:
    /**
     * @brief The ID of the roll used by the system.
     */
    int64_t robro_roll_id{0};

    /**
     * @brief The ID of the customer's roll.
     */
    std::string customer_roll_id{""};

    /**
     * @brief The ID of the machine that the roll is processed on.
     */
    std::string machine_id{""};

    /**
     * @brief The GSM of the roll.
     */
    int gsm{0};

    /**
     * @brief The weight of the roll.
     */
    float weight{0.0f};

    /**
     * @brief The width of the roll.
     */
    float width{0.0f};

    /**
     * @brief The type of material of the roll.
     */
    std::string material_type{"Single"};

        /**
     * @brief The type of material of the roll.
     */
    std::string quality_code{""};


        /**
     * @brief The type of material of the roll.
     */
    float roll_length{0.0f};
    
    /**
     * @brief the loom number of the roll
     */
    std::string loom_number{"0"};


    /**
     * @brief The calculated length of the roll.
     */
    float inspected_length{0.0f};

    /**
     * @brief The time when the roll is started.
     */
    std::string roll_start_time{""};

    /**
     * @brief The time when the roll is ended.
     */
    std::string roll_end_time{""};

    /**
     * @brief it is shown the roll status
     */
    int roll_status{0};

    /**
     * @brief total number of defects in the roll
     */

    int total_defects{0};

    /**
     * @brief the time when its updated
     */

    std::string updated_at{""};

    /**
     * @brief it is shown the current repair meter
     */

    float current_repair_meter{0.0f};

    /**
     * @brief it is shown the notes which the user wants to give
     */

    std::string note{""};

    /**
     * @brief Construct a new Roll object
     */
    Roll();

    /**
     * @brief Convert the roll to JSON.
     *
     * @return json - The roll in JSON format.
     */
    json toJSON();

    /**
     * @brief Load the roll from JSON.
     *
     * @param roll - The roll  in JSON format.
     * @return true - If the roll  is loaded successfully.
     * @return false - If the roll  is not loaded successfully.
     */
    bool loadJSON(json roll);
};

#endif // ROLL_HPP