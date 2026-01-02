/**
 * @file worldMapDrawer.hpp
 * @author Umang G. Patel(umang@robrosystems.com)
 * @brief  This file contains the class definition for the WorldMapDrawer class.
 * @version 1.0
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <worldMapDefect.hpp>
#include <iostream>
#include <map>
#include <json.hpp>
#include <ros/ros.h>
using json = nlohmann::json;

class WorldMapDrawer
{

    /**
     * @brief Store the map height. This is the height of the world map for
     * visualization.
     */
    int map_height{256};

    /**
     * @brief Store the map width. This is the width of the world map for
     * visualization.
     */
    int map_width{1024};

    /**
     * @brief Map constant in y direction. This is the constant that we use to map the y direction.
     * use to convert world map y coordinate to visualization y coordinate.
     */
    float map_const_y{1};

    /**
     * @brief Map constant in x direction. This is the constant that we use to map the x direction.
     * use to convert world map x coordinate to visualization x coordinate.
     */
    float map_const_x{1};

    /**
     * @brief Original cam2stopper distance in mm. This is the original distance of the camera to the stopper with offset.
     */
    int original_cam2stopper_mm{0};

    /**
     * @brief Current cam2stopper distance in mm. This is the current distance of the camera to the stopper with offset.
     * Which calculated at runtime.
     */
    int current_cam2stopper_distance_mm{0};

    /**
     * @brief Dance end to cutter distance in mm. This is the distance of the dancer end to the cutter.
     */
    int dancer_end2stopper_distance_mm{0};

    /**
     * @brief Number of cameras. This is the number of cameras that we have.
     */
    int number_of_cameras;

    /**
     * @brief Cut length in mm. This is the length of the cut that we are going to make or panel we are going to cut.
     */
    int cut_length_mm{0};

    /**
     * @brief World map image. This is the image that we'll be updating with the panel positions and defects.
     * This is the image that we'll be using to display the world map.
     */
    cv::Mat map;

    /**
     * @brief Processed body image.This current processed body image under the cutter which is going to cut the panel.
     */
    cv::Mat processed_body;

    /**
     * @brief cut body image. This is the image of the body that has been cut. used to display the cut body.
     */
    cv::Mat cut_body;

public:
    /**
     * @brief Construct a new Draw World Map object. Initialize Map with ros parameters.
     */
    WorldMapDrawer();

    /**
     * @brief Destroy the Draw World Map object. Release all image resources.
     */
    ~WorldMapDrawer();

    /**
     * @brief Get the World Map visualization image
     *
     * @param panel_positions - panel positions
     * @param cam_config_map - camera configuration map
     * @param defects - defects
     * @param auto_cut_next_pos - auto cut next position
     * @param auto_cut_color - auto cut color
     * @return cv::Mat - world map
     */
    cv::Mat getWorldMap(const std::vector<int> panel_positions,
                        const std::map<std::string, json> cam_config_map,
                        const std::vector<WorldMapDefect> new_master_defects,
                        int auto_cut_next_pos,
                        cv::Scalar auto_cut_color);

    /**
     * @brief Get the Last Defect In Process Panel Map visualization image
     *
     * @param cam_config_map - camera configuration map
     * @param defects - defects
     * @param stopped_for_seq_id - stopped for sequence id
     * @return cv::Mat - last defect in process panel map
     */
    cv::Mat getLastDefectInProcessPanelMap(const std::map<std::string, json> cam_config_map,
                                           const std::vector<WorldMapDefect> new_master_defects,
                                           int stopped_for_seq_id);

    /**
     * @brief Get the First Defect In Process Panel Map visualization image
     *
     * @param cam_config_map - camera configuration map
     * @param defects - defects
     * @param stopped_for_seq_id - stopped for sequence id
     * @return cv::Mat - first defects in process panel map
     */
    cv::Mat getFirstDefectInProcessPanelMap(const std::map<std::string, json> cam_config_map,
                                            const std::vector<WorldMapDefect> new_master_defects,
                                            int stopped_for_seq_id);

    /**
     * @brief Get the visualization for the next cutting panel
     *
     * @param cam_config_map - camera configuration map
     * @param defects - defects
     * @param next_cut_len - the next body that is going to be cut
     * @return cv::Mat - first defects in process panel map
     */
    cv::Mat getAutoCutMapVisualization(const std::map<std::string, json> cam_config_map,
                                            const std::vector<WorldMapDefect> new_master_defects, int next_cut_len,cv::Scalar auto_cut_color);

    /**
     * @brief Draw the dashed line on the image.
     *
     * @param image - image on which line is to be drawn.
     * @param pt1 - start point of the line.
     * @param pt2 - end point of the line.
     * @param color - color of the line.
     * @param dash_length - length of the dash.
     * @param gap_length - length of the gap.
     */
    void drawDashedLine(cv::Mat &image, cv::Point pt1, cv::Point pt2, cv::Scalar color,
                        int dash_length = 5, int gap_length = 5);
    /**
     * @brief Rect Boundary check and update teh value.
     *
     * @param rect - passed rect
     * @param size - image size param
     */
    void rectBoundaryCheck(cv::Rect &rect, cv::Size size);

    /**
     * @brief Update the map size.
     *
     * @param width - Current width of the map, Which is current camera total fov.
     * @param height - Current height of the map, Which is current camera to cutter distance.
     */
    void updateMapSize(int width, int height);

    /**
     * @brief Update the map distances.
     *
     * @param cam2stopper_mm - Current camera to stopper distance.
     * @param cut_length - Current cut length.
     * @param dancer_end2cutter_mm - Current dancer end to cutter distance with offset.
     */
    void updateMapDistances(int cam2stopper_mm, int cut_length, int dancer_end2cutter_mm);
};