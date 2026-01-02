/**
 * @file jobAndRecipeManager.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief KWIS application job recipe blue print.
 * @version 0.1.0
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <jobAndRecipeManager.hpp>

#ifndef JOBRECIPECLASS_CPP
#define JOBRECIPECLASS_CPP

/**
 * @brief It checks the recipe by checking if files exists or not.
 *
 * @param recipe_name - name of the recipe
 * @return true - if recipe exists
 * @return false - if recipe does not exists
 */
bool JobAndRecipeManager::checkRecipeAndFiles(std::string recipe_name)
{
    // Check all recipes are loaded or not
    if (all_recipes_json.empty() || !all_recipes_json.contains(recipe_name))
    {
        return false;
    }

    // Check if all files exists or not
    bool files_exist = true;
    boost::filesystem::path nn_f(nn_file_path);
    if (!all_recipes_json[recipe_name].contains("nn"))
    {
        ROS_ERROR("No nn block in json of recipe: %s", recipe_name.c_str());
        return false;
    }

    json nn_info = all_recipes_json[recipe_name]["nn"];

    if (nn_info.contains("yolov8"))
    {
        if (nn_info["yolov8"].contains("engine_file") && nn_info["yolov8"].contains("names_file"))
        {
            files_exist = files_exist &&
                          boost::filesystem::exists(nn_f / (nn_info["yolov8"]["engine_file"])) &&
                          boost::filesystem::exists(nn_f / (nn_info["yolov8"]["names_file"]));
        }
        else
        {
            ROS_ERROR("Unable to load yolov8 files for recipe: %s", recipe_name.c_str());
        }
    }
    else
    {
        ROS_ERROR("One NN block is needed yolov8");
        return false;
    }

    if (nn_info.contains("disabled_class_ids"))
    {
        if (!nn_info["disabled_class_ids"].is_array())
        {
            ROS_ERROR("Disabled Class IDs should be an array!");
            std::cout << "[ERROR] Disbaled Class IDs should be an array!\n";
            return false;
        }
    }

    if (!nn_info.contains("groups"))
    {
        ROS_ERROR("Recipe doesn't have groups");
        return false;
    }

    if (!all_recipes_json.contains(recipe_name) ||
        !all_recipes_json[recipe_name].contains("nn") ||
        !all_recipes_json[recipe_name].contains("camera") ||
        !all_recipes_json[recipe_name]["camera"].contains("exposure_time"))
    {
        ROS_ERROR("Recipe Doesn't have camera things.");
        return false;
    }

    if (!nn_info.contains("tiling") ||
        !nn_info.contains("threshold") ||
        !nn_info.contains("nms_threshold") ||
        !nn_info.contains("num_classes"))
    {
        ROS_ERROR("Recipe doesn't have tiling, threshold, nms_threshold, num_classes");
        return false;
    }

    return files_exist;
}

/**
 * @brief Get the Recipes list in json format as string.
 *
 * @return std::string - json string
 */
std::string JobAndRecipeManager::getRecipesListInJSONStr()
{
    json list;

    // Return empty list if no recipes are loaded
    if (all_recipes_json.empty() || current_recipe_name.empty())
    {
        return list.dump();
    }

    // Add all recipes to the list
    for (auto it = all_recipes_json.begin(); it != all_recipes_json.end(); ++it)
    {
        list += json::object_t::value_type(it.key(), (bool)(it.key() == current_recipe_name));
    }

    return list.dump();
}

/**
 * @brief Get the Current Recipe in json format as string.
 *
 * @return std::string - json string
 */
std::string JobAndRecipeManager::getCurrentRecipeInJSONStr()
{
    if (all_recipes_json.empty() || current_recipe_name.empty())
    {
        return json().dump();
    }
    return all_recipes_json[current_recipe_name].dump();
}

/**
 * @brief Get the Current Recipe Json object
 *
 * @return json - json object
 */
json JobAndRecipeManager::getCurrentRecipeJson()
{
    if (all_recipes_json.empty() || current_recipe_name.empty())
    {
        return json();
    }
    return all_recipes_json[current_recipe_name];
}

/**
 * @brief Get the Current Recipe object
 *
 * @return Recipe - recipe object
 */
Recipe JobAndRecipeManager::getCurrentRecipe()
{
    if (all_recipes_json.empty() || current_recipe_name.empty())
    {
        return Recipe();
    }
    return recipe;
}

/**
 * @brief Check if recipe is different from current recipe.
 *
 * @param recipe - recipe name
 * @return true - if recipe is different
 * @return false - if recipe is same.
 */
bool JobAndRecipeManager::isRecipeDifferent(std::string recipe_name)
{
    return recipe_name != current_recipe_name;
}

/**
 * @brief load the recipes from project id base json file in given folder.
 *
 * @param project_base_path - project base path
 * @param project_id - project id as json file name
 * @return true - if recipes loaded successfully
 * @return false - if recipes not loaded successfully
 */
bool JobAndRecipeManager::loadRecipes(std::string project_base_path, std::string project_id)
{
    boost::filesystem::path base_path(project_base_path);
    std::string filename = project_id + ".json";

    recipes_json_file_path = boost::filesystem::path(base_path / "config" / filename).string();
    nn_file_path = boost::filesystem::path(base_path / "nn");

    if (boost::filesystem::exists(recipes_json_file_path))
    {
        all_recipes_json = json::parse(std::ifstream(recipes_json_file_path));
    }
    else
    {
        ROS_ERROR("%s doesn't exist", recipes_json_file_path.c_str());
        current_recipe_name.clear();
        return false;
    }

    std::string first_recipe = all_recipes_json.begin().key();

    if (!setCurrentRecipe(first_recipe))
    {
        ROS_ERROR("Recipe %s isn't proper", first_recipe.c_str());
        current_recipe_name.clear();
        return false;
    }

    return true;
}

/**
 * @brief save the new recipes file
 *
 * @param new_all_recipes_json - the json of the new file
 * @return true - if recipe is saved successfully
 */
bool JobAndRecipeManager::saveRecipesJSON(json new_all_recipes_json, bool write_to_file)
{
    json all_recipes_json_copy = all_recipes_json;

    all_recipes_json = new_all_recipes_json;

    if (!setCurrentRecipe(current_recipe_name))
    {
        ROS_ERROR("In Saving New Json: Recipe %s isn't proper", current_recipe_name.c_str());
        all_recipes_json = all_recipes_json_copy;
        return false;
    }

    if (write_to_file)
    {
        std::ofstream recipes_file_stream(recipes_json_file_path);
        recipes_file_stream << std::setw(2) << new_all_recipes_json;
        recipes_file_stream.close();
    }

    return true;
}

/**
 * @brief Get All recipes
 *
 * @return the all recipes json
 */
json JobAndRecipeManager::getAllRecipesJSON()
{
    return all_recipes_json;
}

std::string JobAndRecipeManager::getAllAvailableNamesAndEngineFiles(std::string project_base_path)
{
    boost::filesystem::path base_path(project_base_path);

    json nn_files;
    json nn_file_content;
    nn_file_path = boost::filesystem::path(base_path / "nn");
    nn_files["names"] = json::array();
    nn_files["engines"] = json::array();

    for (const auto &entry : boost::filesystem::directory_iterator(nn_file_path))
    {
        std::string file_name = entry.path().filename().string();
        std::string full_path = entry.path().string();
        if (file_name.find(".names") != std::string::npos)
        {
            nn_files["names"].push_back(file_name);
            json name_file_data = json::array();
            // Create an array for the .names file with the file name, full path, and content
            // Read the content of the .names file line by line
            std::ifstream file(full_path);
            if (file.is_open())
            {
                std::string line;
              // Create a JSON array to hold the lines of the file

                // Read the file line by line
                while (std::getline(file, line))
                {
                    name_file_data.push_back(line); // Add each line as a separate element in the array
                }

                file.close();
            }

            nn_file_content[file_name] = name_file_data;
        }
        // Process .engine files and add them to the engines array
        else if (file_name.find(".engine") != std::string::npos)
        {
            nn_files["engines"].push_back(file_name);
        }
        nn_files["names_data"]=nn_file_content;
    }
    return nn_files.dump();
}

/**
 * @brief Set the Current Recipe object
 *
 * @param recipe_name - name of the recipe
 * @return true - if recipe set successfully
 * @return false - if recipe not set successfully
 */
bool JobAndRecipeManager::setCurrentRecipe(std::string recipe_name)
{
    if (all_recipes_json.empty() || !all_recipes_json.contains(recipe_name))
    {
        return false;
    }

    if (!checkRecipeAndFiles(recipe_name))
    {
        return false;
    }

    current_recipe_name = recipe_name;
    Recipe new_recipe;

    json nn_info = all_recipes_json[recipe_name]["nn"];

    new_recipe.cropping_margin = cropping_margin;

    new_recipe.is_available = true;

    new_recipe.name = recipe_name;

    if (nn_info.contains("yolov8"))
    {
        new_recipe.network_type = "yolov8";
        new_recipe.engine_file_path = boost::filesystem::path(nn_file_path / nn_info["yolov8"]["engine_file"]).string();
        new_recipe.names_file_path = boost::filesystem::path(nn_file_path / nn_info["yolov8"]["names_file"]).string();
    }

    new_recipe.tiling = nn_info["tiling"];
    new_recipe.number_of_classes = nn_info["num_classes"];
    new_recipe.threshold = nn_info["threshold"];
    new_recipe.nms_threshold = nn_info["nms_threshold"];
    new_recipe.exposure = all_recipes_json[recipe_name]["camera"]["exposure_time"];

    if (nn_info.contains("min_defect_area_px"))
    {
        new_recipe.min_defect_area_px = nn_info["min_defect_area_px"];
    }
    else
    {
        new_recipe.min_defect_area_px = 0;
    }

    if (nn_info.contains("groups"))
    {
        new_recipe.groups = nn_info["groups"];
    }
    else
    {
        new_recipe.groups = json::array();
    }

    // load recipe
    recipe = new_recipe;
    return true;
}


#endif
