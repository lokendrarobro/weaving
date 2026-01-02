#ifndef RECIPE_JSON_HPP
#define RECIPE_JSON_HPP

#include <json.hpp>
#include <string>
#include <vector>
#include <map>
#include <iostream>

using json = nlohmann::json;

namespace RecipeJSON
{
    // Camera structure
    struct Camera
    {
        int exposure_time;
    };

    // Group structure inside NN
    struct Group
    {
        bool enabled;
        std::vector<int> ids;
        int sensitivity_x;
        int sensitivity_y;
        double threshold;
        int region_continuity_mm;
        int region_threshold_mm;
    };

    // Yolov8 structure
    struct Yolov8
    {
        std::string engine_file;
        std::string names_file;
    };

    // NN (Neural Network) structure
    struct NN
    {
        std::map<std::string, Group> groups;
        double nms_threshold;
        int num_classes;
        double threshold;
        bool tiling;
        Yolov8 yolov8;
    };

    // Recipe class encapsulating Camera and NN
    class Recipe
    {
    public:
        Camera camera; // Camera data
        NN nn;         // Neural network settings

        // Default constructor
        Recipe() = default;

        // Method to load JSON data into the Recipe object
        inline bool loadJson(const json &j)
        {
            try
            {
                // Directly parse the "camera" and "nn" JSON data
                camera.exposure_time = j["camera"]["exposure_time"];
                nn.nms_threshold = j["nn"]["nms_threshold"];
                nn.num_classes = j["nn"]["num_classes"];
                nn.threshold = j["nn"]["threshold"];
                nn.tiling = j["nn"]["tiling"];
                nn.yolov8.engine_file = j["nn"]["yolov8"]["engine_file"];
                nn.yolov8.names_file = j["nn"]["yolov8"]["names_file"];

                // Parse groups inside nn
                for (auto &group : j["nn"]["groups"].items())
                {
                    Group g;
                    g.enabled = group.value()["enabled"];
                    g.ids = group.value()["ids"].get<std::vector<int>>(); // Directly get vector of ints
                    g.sensitivity_x = group.value()["sensitivity_x"];
                    g.sensitivity_y = group.value()["sensitivity_y"];
                    g.threshold = group.value()["threshold"];
                    nn.groups[group.key()] = g; // Insert the group into the map

                   
                }

                return true; // Successfully loaded JSON data
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error loading JSON: " << e.what() << std::endl;
                return false; // Failed to load JSON
            }
        }

        // Display the loaded recipe data (for testing purposes)
        inline void display() const
        {
            // Display Camera data
            std::cout << "Camera Exposure Time: " << camera.exposure_time << std::endl;

            // Display NN data
            std::cout << "NN Settings:" << std::endl;
            std::cout << "  NMS Threshold: " << nn.nms_threshold << std::endl;
            std::cout << "  Num Classes: " << nn.num_classes << std::endl;
            std::cout << "  Threshold: " << nn.threshold << std::endl;
            std::cout << "  Tiling: " << (nn.tiling ? "True" : "False") << std::endl;
            std::cout << "  Yolov8 Engine File: " << nn.yolov8.engine_file << std::endl;
            std::cout << "  Yolov8 Names File: " << nn.yolov8.names_file << std::endl;

            // Display Group data (example for group "0")
            std::cout << "Group 0 Settings:" << std::endl;
            const Group &g = nn.groups.at("0");
            std::cout << "  Enabled: " << (g.enabled ? "True" : "False") << std::endl;
            std::cout << "  Sensitivity X: " << g.sensitivity_x << std::endl;
            std::cout << "  Sensitivity Y: " << g.sensitivity_y << std::endl;
            std::cout << "  Threshold: " << g.threshold << std::endl;
            std::cout << "  Region Continuity MM: " << g.region_continuity_mm << std::endl;
            std::cout << "  Region Threshold MM: " << g.region_threshold_mm << std::endl;
        }
    };
} // namespace RecipeJSON

#endif // RECIPE_JSON_HPP
