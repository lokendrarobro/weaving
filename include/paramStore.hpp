#ifndef PARAMSTORE_HPP
#define PARAMSTORE_HPP

#include <string>
#include <json.hpp>

using json = nlohmann::json;
class ParamStore
{
public:
    /**
     * @brief Construct a new Param Store:: Param Store object
     *
     * @param base_path
     */
    ParamStore(const std::string &filename);

    /**
     * @brief Set the Param object and write to file
     *
     * @param key - key of the parameter
     * @param value - value of the parameter
     */
    void setParam(const std::string &key, const json &value);

    /**
     * @brief - Get the Param object.
     *
     * @param key - key of the parameter.
     * @return json - value of the parameter.
     */
    json getParam(const std::string &key) const;

    /**
     * @brief Get the Params object
     *
     * @return json - all parameters.
     */
    json getParams() const;

private:
    /**
     * @brief Write the parameters to file.
     */
    void writeToFile() const;

    /**
     * @brief Filename of the parameter file.
     */
    std::string filename;

    /**
     * @brief  The parameters.
     *
     */
    json params;
};

#endif // PARAMSTORE_HPP
