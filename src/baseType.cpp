/**
 * @file baseType.cpp
 * @author Umang G. Patel(umang@robrosystems.com)
 * @brief Base class which create the type which used in the database operation.
 * @version 1.0
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <baseType.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#ifndef BASE_TYPE_CPP
#define BASE_TYPE_CPP

/**
 * @brief Construct a new Base Type object
 *
 * @param name - The type name of the type.
 */
BaseType::BaseType(std::string name) : type_name(name){};

/**
 * @brief load the type from string
 *
 * @param type - The type in json string format
 * @return true - If the type is loaded successfully.
 * @return false - If the type is not loaded successfully.
 */
bool BaseType::loadString(std::string type)
{
    try
    {
        json j = json::parse(type);
        return loadJSON(j);
    }
    catch (const std::exception &e)
    {
        std::cout << "[" + type_name + "] Invalid JSON string: " << std::string(e.what());
        return false;
    }
}

/**
 * @brief Convert the type to string
 *
 * @return std::string - The type in json string formate
 */
std::string BaseType::toString()
{
    return toJSON().dump();
}

/**
 * @brief Generate a new ID for the body.It uses boost::uuids::random_generator
 * to generate a new ID.
 *
 * @return std::string - The new ID.
 */
std::string BaseType::genNewID()
{
    boost::uuids::random_generator generator;
    boost::uuids::uuid uuid = generator();
    return type_name + "_" + boost::uuids::to_string(uuid);
}

#endif // BASE_TYPE_CPP