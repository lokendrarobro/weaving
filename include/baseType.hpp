/**
 * @file baseType.hpp
 * @author Umang G. Patel(umang@robrosystems.com)
 * @brief Base class which create the type which used in the database operation.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <json.hpp>
#include <iostream>

/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

#ifndef BASE_TYPE_HPP
#define BASE_TYPE_HPP

class BaseType
{
    /**
     * @brief The type name of the type. Which used to identify the type as well as
     * use to generate the unique ID for the type.
     */
    std::string type_name;

public:
    /**
     * @brief Construct a new Base Type object
     * 
     * @param name - The type name of the type.
     */
    BaseType(std::string name);

    /**
     * @brief Convert the type to json
     *
     * @return json - The type in json format
     */
    virtual json toJSON() = 0;

    /**
     * @brief Load the type from json.
     *
     * @param type - The type in json format
     * @return true - If the type is loaded successfully.
     * @return false - If the type is not loaded successfully.
     */
    virtual bool loadJSON(json type) = 0;

    /**
     * @brief Convert the type to string
     *
     * @return std::string - The type in json string formate
     */
    std::string toString();

    /**
     * @brief load the type from string
     *
     * @param type - The type in json string format
     * @return true - If the type is loaded successfully.
     * @return false - If the type is not loaded successfully.
     */
    bool loadString(std::string type);

    /**
     * @brief Generate a new ID for the given type.It uses boost::uuids::random_generator
     * to generate a new ID.
     *
     * @return std::string - The new ID.
     */
    std::string genNewID();
};

#endif // BASE_TYPE_HPP