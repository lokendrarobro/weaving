/**
 * @file jobRecipe.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief KWIS application job recipe blue print.
 * @version 0.1.0
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <mutex>
#include <json.hpp>
#include <iostream>
#include <ros/ros.h>
#include <recipe.hpp>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>

#ifndef JOBRECIPECLASS_HPP
#define JOBRECIPECLASS_HPP

/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Class to manage the job and recipe for KWIS application.
 *
 */
class JobAndRecipeManager
{
    /**
     * @brief It stores all the recipes in json format for the KWIS application..
     */
    json all_recipes_json;

    /**
     * @brief It stores the current recipe name for the KWIS application.
     */
    std::string current_recipe_name;

    /**
     * @brief It stores the json file path for the KWIS application.
     */
    std::string recipes_json_file_path;

    /**
     * @brief It stores the nn file path for the KWIS application.
     */
    boost::filesystem::path nn_file_path;

    /**
     * @brief It stores the recipe for the KWIS application.
     */
    Recipe recipe;

public:
    /**
     * @brief It checks the recipe by checking if files exists or not.
     *
     * @param recipe_name - name of the recipe
     * @return true - if recipe exists
     * @return false - if recipe does not exists
     */
    bool checkRecipeAndFiles(std::string recipe_name);

    /**
     * @brief Get all Recipes list in json format as string.
     *
     * @return json : all recipes file in json format
     */
    json getAllRecipesJSON();

    /**
     * @brief Get the Recipes list in json format as string.
     *
     * @return std::string - json string
     */
    std::string getRecipesListInJSONStr();

    int cropping_margin{0};

    /**
     * @brief Get the Current Recipe in json format as string.
     *
     * @return std::string - json string
     */
    std::string getCurrentRecipeInJSONStr();

    /**
     * @brief Get the Current Recipe Json object
     *
     * @return json - json object
     */
    json getCurrentRecipeJson();

    /**
     * @brief Get the Current Recipe object
     *
     * @return Recipe - recipe object
     */
    Recipe getCurrentRecipe();

    /**
     * @brief Check if recipe is different from current recipe.
     *
     * @param recipe - recipe name
     * @return true - if recipe is different
     * @return false - if recipe is same.
     */
    bool isRecipeDifferent(std::string new_recipe);

    /**
     * @brief load the recipes from project id base json file in given folder.
     *
     * @param project_base_path - project base path
     * @param project_id - project id as json file name
     * @return true - if recipes loaded successfully
     * @return false - if recipes not loaded successfully
     */
    bool loadRecipes(std::string project_base_path, std::string project_id);

    /**
     * @brief Set the Current Recipe object
     *
     * @param recipe_name - name of the recipe
     * @return true - if recipe set successfully
     * @return false - if recipe not set successfully
     */
    bool setCurrentRecipe(std::string recipe_name);

    /**
     * @brief save the new recipes file
     *
     * @param new_all_recipes_json - the json of the new file
     * @return true - if recipe is saved successfully
     */
    bool saveRecipesJSON(json new_all_recipes_json, bool write_to_file);

    /**
     * @brief list files
     *
     * @return string
     */
    std::string getAllAvailableNamesAndEngineFiles(std::string project_base_path);

      /**
     * @brief update cropping margin value
     */
    inline void updateCroppingMarging(int value)
    {
        cropping_margin = value;
    }
};
#endif
