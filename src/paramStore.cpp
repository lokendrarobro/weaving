/**
 * @file paramStore.cpp
 * @author Umang G. Patel (umang@robrosytems.com)
 * @brief  paramStore class implementation
 * @version 1.0.0
 * @date 2023-07-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <paramStore.hpp>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

/**
 * @brief Construct a new Param Store:: Param Store object
 *
 * @param base_path
 */
ParamStore::ParamStore(const std::string &base_path)
{
    filename = fs::path(fs::path(base_path) / "config" / "params.json").string();
    if (fs::exists(filename))
    {
        std::ifstream file(filename);
        if (file.is_open())
        {
            try
            {
                file >> params;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed to parse JSON file: " << e.what() << std::endl;
            }
        }
        else
        {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
        file.close();
    }
    else
    {
        params = json::object();
        writeToFile();
    }
}

/**
 * @brief Set the Param object and write to file
 *
 * @param key - key of the parameter
 * @param value - value of the parameter
 */
void ParamStore::setParam(const std::string &key, const json &value)
{
    params[key] = value;
    writeToFile();
}

/**
 * @brief - Get the Param object.
 *
 * @param key - key of the parameter.
 * @return json - value of the parameter.
 */
json ParamStore::getParam(const std::string &key) const
{
    if (params.contains(key))
    {
        return params[key];
    }
    else
    {
        std::cerr << "Parameter with key '" << key << "' does not exist." << std::endl;
        return nullptr;
    }
}

/**
 * @brief Get the Params object
 *
 * @return json - all parameters.
 */
json ParamStore::getParams() const
{
    return params;
}

/**
 * @brief Write the parameters to file.
 */
void ParamStore::writeToFile() const
{

    std::ofstream file(filename);
    if (file.is_open())
    {
        file << params.dump(2) << std::endl;
    }
    else
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
    file.close();
}
