/**
 * @file worldMapDrawer.cpp
 * @author Umang G. Patel (umang@robrosystems.com)
 * @brief This file contains the class implementation for the WorldMapDrawer class.
 * @version 1.0
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <worldMapDrawer.hpp>

/**
 * @brief Construct a new Draw World Map object. Initialize Map with ros parameters.
 */
WorldMapDrawer::WorldMapDrawer()
{
    if (!(ros::param::get("~map_height", map_height) &&
          ros::param::get("~map_width", map_width) &&
          ros::param::get("number_of_cameras", number_of_cameras)))
    {
        std::cout << "\033[1;31m ERROR : \033[0m[WorldMapDrawer]Couldn't read all params from launch file. Please check names and types!";
    }
}

/**
 * @brief Destroy the Draw World Map object. Release all image resources.
 */
WorldMapDrawer::~WorldMapDrawer()
{
    map.release();
    processed_body.release();
    cut_body.release();
}

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
cv::Mat WorldMapDrawer::getWorldMap(const std::vector<int> panel_positions,
                                    const std::map<std::string, json> cam_config_map,
                                    const std::vector<WorldMapDefect> defects,
                                    int auto_cut_next_pos,
                                    cv::Scalar auto_cut_color)
{
    try
    {
        // map height is draw.
        int cam2stopper_h_px = (int)(3.0 * map_height / 4.0);
        float map_y_px_per_mm = cam2stopper_h_px / (float)(original_cam2stopper_mm + 500);
        cv::Mat defect_map = cv::Mat::zeros(cv::Size(map_width, map_height), CV_8UC3);
        // apply base color map.
        defect_map = cv::Scalar(112, 48, 7);

        // Draw the cutter line line
        cv::line(defect_map,
                 cv::Point(0, cam2stopper_h_px),
                 cv::Point(map_width, cam2stopper_h_px),
                 cv::Scalar(0, 75, 150),
                 2,
                 cv::LINE_4);

        cv::Point p1(0, (cam2stopper_h_px - (current_cam2stopper_distance_mm * map_y_px_per_mm)));
        cv::Point p2(map_width, (cam2stopper_h_px - (current_cam2stopper_distance_mm * map_y_px_per_mm)));

        // Camera line
        cv::line(defect_map,
                 p1,
                 p2,
                 cv::Scalar(0, 0, 0),
                 5,
                 cv::LINE_4);

        // Draw panel positions
        for (auto position : panel_positions)
        {
            cv::line(defect_map,
                     cv::Point(0, (int)(cam2stopper_h_px - ceil(position * map_y_px_per_mm))),
                     cv::Point(map_width, (int)(cam2stopper_h_px - ceil(position * map_y_px_per_mm))),
                     cv::Scalar(255, 255, 255),
                     2,
                     cv::LINE_4);
        }
        // Add Dancer End to Stopper distance offset line
        drawDashedLine(defect_map, cv::Point(0, (int)(cam2stopper_h_px - ceil(dancer_end2stopper_distance_mm * map_y_px_per_mm))),
                       cv::Point(map_width, (int)(cam2stopper_h_px - ceil(dancer_end2stopper_distance_mm * map_y_px_per_mm))),
                       cv::Scalar(86, 167, 255), 5, 5);
        if (auto_cut_next_pos > 0)
        {
            cv::line(defect_map,
                     cv::Point(0, (int)(cam2stopper_h_px - ceil(auto_cut_next_pos * map_y_px_per_mm))),
                     cv::Point(map_width, (int)(cam2stopper_h_px - ceil(auto_cut_next_pos * map_y_px_per_mm))),
                     auto_cut_color,
                     2,
                     cv::LINE_4);
        }
        // Draw vertical lines for each camera
        int w = map_width / number_of_cameras;
        int cam_counter = 1;
        for (auto config : cam_config_map)
        {
            cv::line(defect_map,
                     cv::Point(w * cam_counter, 0),
                     cv::Point(w * cam_counter, map_height),
                     cv::Scalar(161, 152, 125),
                     2,
                     cv::LINE_4);
            cam_counter++;
        }
        // to plot red defect on top of white
        std::vector<cv::Rect> enabled_rects;
        for (auto defect : defects)
        {
            cv::Rect r;
            int distance_from_stopper_mm = (defect.detection.bbox.center.y - defect.detection.bbox.size_y);
            r.y = cam2stopper_h_px - distance_from_stopper_mm * map_y_px_per_mm;
            r.x = (defect.detection.bbox.center.x - defect.detection.bbox.size_x) * map_const_x;
            r.width = ceil(2 * defect.detection.bbox.size_x * map_const_x);
            r.height = ceil(2 * defect.detection.bbox.size_y * map_y_px_per_mm);
            rectBoundaryCheck(r, cv::Size(map_width, map_height));
            int this_defect_id = defect.detection.results[0].id;

            if (!defect.is_defect_class_disabled)
            {
                enabled_rects.push_back(r);
            }
            else
            {
                cv::rectangle(defect_map,
                              r,
                              cv::Scalar(255, 255, 255),
                              -1);
            }
        }
        for (auto r : enabled_rects)
        {
            cv::rectangle(defect_map,
                          r,
                          cv::Scalar(0, 0, 255),
                          -1);
        }

        return defect_map;
    }
    catch (...)
    {
        std::cerr << "[getWorldMap] An exception occurred." << std::endl;
        cv::Mat defect_map = cv::Mat::zeros(cv::Size(map_width, map_height), CV_8UC3);
        defect_map = cv::Scalar(112, 48, 7);
        return defect_map;
    }
}

/**
 * @brief Get the visualization for the next cutting panel
 *
 * @param cam_config_map - camera configuration map
 * @param defects - defects
 * @return cv::Mat - first defects in process panel map
 */
cv::Mat WorldMapDrawer::getAutoCutMapVisualization(const std::map<std::string, json> cam_config_map,
                                                   const std::vector<WorldMapDefect> defects,
                                                   int next_cut_len, cv::Scalar auto_cut_color)
{
    int processed_body_map_h = map_height;
    int processed_body_map_w = map_width;
    try
    {
        processed_body = cv::Mat::zeros(cv::Size(processed_body_map_w, processed_body_map_h), CV_8UC3);
        processed_body = auto_cut_color;
        int count = 1;
        for (auto defect : defects)
        {
            if (defect.detection.bbox.center.y - defect.detection.bbox.size_y < next_cut_len)
            {
                cv::Point center;
                center.x = (defect.detection.bbox.center.x) * map_const_x;
                if (defect.detection.bbox.center.y - next_cut_len < 0)
                {
                    center.y = (-defect.detection.bbox.center.y + next_cut_len) * map_const_y;
                }
                else
                {
                    center.y = 0;
                }

                // Calculate the radius to cover the full text
                std::string id_text = "D" + std::to_string(count);
                count++;
                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.6;
                int thickness = 2;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(id_text, font, font_scale, thickness, &baseline);
                int radius = (text_size.width + text_size.height) / 2; // Adjust the division factor as per your preference
                cv::circle(processed_body, center, radius - 3, cv::Scalar(255, 255, 255), -1);

                if (defect.is_defect_class_disabled)
                {
                    cv::circle(processed_body,
                               center,
                               radius - 3,
                               cv::Scalar(255, 255, 255),
                               3);
                }
                else
                {
                    cv::circle(processed_body,
                               center,
                               radius - 3,
                               cv::Scalar(0, 0, 255),
                               3);
                }

                // Write the ID inside the circle
                cv::Point text_origin(center.x - text_size.width / 2, center.y + text_size.height / 2);
                cv::putText(processed_body, id_text, text_origin, font, font_scale, cv::Scalar(0, 0, 0), thickness);
            }
        }
        int w = processed_body_map_w / number_of_cameras;
        int cam_counter = 1;
        for (auto config : cam_config_map)
        {
            cv::line(processed_body,
                     cv::Point(w * cam_counter, 0),
                     cv::Point(w * cam_counter, map_height),
                     cv::Scalar(161, 152, 125),
                     2,
                     cv::LINE_4);
            cam_counter++;
        }

        return processed_body;
    }
    catch (...)
    {
        std::cerr << "[getAutoCutMapVisualization] An exception occurred." << std::endl;
        processed_body = cv::Mat::zeros(cv::Size(processed_body_map_w, processed_body_map_h), CV_8UC3);
        processed_body = cv::Scalar(112, 48, 7);
        return processed_body;
    }
}

/**
 * @brief Get the Last Defect In Process Panel Map visualization image
 *
 * @param cam_config_map - camera configuration map
 * @param defects - defects
 * @param stopped_for_seq_id - stopped for sequence id
 * @return cv::Mat - last defect in process panel map
 */
cv::Mat WorldMapDrawer::getLastDefectInProcessPanelMap(const std::map<std::string, json> cam_config_map,
                                                       const std::vector<WorldMapDefect> defects,
                                                       int stopped_for_seq_id)
{
    int processed_body_map_h = map_height;
    int processed_body_map_w = map_width;
    try
    {

        processed_body = cv::Mat::zeros(cv::Size(processed_body_map_w, processed_body_map_h), CV_8UC3);
        processed_body = cv::Scalar(112, 48, 7);
        int count = 1;
        for (auto defect : defects)
        {
            if (defect.detection.bbox.center.y - defect.detection.bbox.size_y < 0 || defect.sequence_id <= stopped_for_seq_id)
            {
                cv::Point center;
                center.x = (defect.detection.bbox.center.x) * map_const_x;
                if (defect.detection.bbox.center.y < 0)
                {
                    center.y = (-defect.detection.bbox.center.y) * map_const_y;
                }
                else
                {
                    center.y = 0;
                }

                // Calculate the radius to cover the full text
                std::string id_text = "D" + std::to_string(count);
                count++;
                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.6;
                int thickness = 2;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(id_text, font, font_scale, thickness, &baseline);
                int radius = (text_size.width + text_size.height) / 2; // Adjust the division factor as per your preference
                cv::circle(processed_body, center, radius - 3, cv::Scalar(255, 255, 255), -1);

                if (defect.is_defect_class_disabled)
                {
                    cv::circle(processed_body,
                               center,
                               radius - 3,
                               cv::Scalar(255, 255, 255),
                               3);
                }
                else
                {
                    cv::circle(processed_body,
                               center,
                               radius - 3,
                               cv::Scalar(0, 0, 255),
                               3);
                }

                // Write the ID inside the circle
                cv::Point text_origin(center.x - text_size.width / 2, center.y + text_size.height / 2);
                cv::putText(processed_body, id_text, text_origin, font, font_scale, cv::Scalar(0, 0, 0), thickness);
            }
        }
        int w = processed_body_map_w / number_of_cameras;
        int cam_counter = 1;
        for (auto config : cam_config_map)
        {
            cv::line(processed_body,
                     cv::Point(w * cam_counter, 0),
                     cv::Point(w * cam_counter, map_height),
                     cv::Scalar(161, 152, 125),
                     2,
                     cv::LINE_4);
            cam_counter++;
        }

        return processed_body;
    }
    catch (...)
    {
        std::cerr << "[getLastDefectInProcessPanelMap] An exception occurred." << std::endl;
        processed_body = cv::Mat::zeros(cv::Size(processed_body_map_w, processed_body_map_h), CV_8UC3);
        processed_body = cv::Scalar(112, 48, 7);
        return processed_body;
    }
}

/**
 * @brief Get the First Defect In Process Panel Map visualization image
 *
 * @param cam_config_map - camera configuration map
 * @param defects - defects
 * @param stopped_for_seq_id - stopped for sequence id
 * @return cv::Mat - first defects in process panel map
 */
cv::Mat WorldMapDrawer::getFirstDefectInProcessPanelMap(const std::map<std::string, json> cam_config_map,
                                                        const std::vector<WorldMapDefect> defects,
                                                        int stopped_for_seq_id)
{
    int cut_body_map_h = map_height;
    int cut_body_map_w = map_width;
    try
    {
        cut_body = cv::Mat::zeros(cv::Size(cut_body_map_w, cut_body_map_h), CV_8UC3);
        cut_body = cv::Scalar(112, 48, 7);
        int count{1};
        for (auto defect : defects)
        {
            if (defect.detection.bbox.center.y - defect.detection.bbox.size_y < 0 || defect.sequence_id <= stopped_for_seq_id)
            {
                cv::Point center;
                center.x = (defect.detection.bbox.center.x) * map_const_x;
                if (defect.detection.bbox.center.y < 0)
                {
                    center.y = (-defect.detection.bbox.center.y) * map_const_y;
                }
                else
                {
                    center.y = 0;
                }
                // Calculate the radius to cover the full text
                std::string id_text = "D" + std::to_string(count);
                count++;
                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.6;
                int thickness = 2;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(id_text, font, font_scale, thickness, &baseline);
                int radius = (text_size.width + text_size.height) / 2; // Adjust the division factor as per your preference

                cv::circle(cut_body, center, radius - 3, cv::Scalar(255, 255, 255), -1);

                if (defect.is_defect_class_disabled)
                {
                    cv::circle(cut_body,
                               center,
                               radius - 3,
                               cv::Scalar(255, 255, 255),
                               3);
                }
                else
                {
                    cv::circle(cut_body,
                               center,
                               radius - 3,
                               cv::Scalar(0, 0, 255),
                               3);
                }

                // Write the ID inside the circle
                cv::Point text_origin(center.x - text_size.width / 2, center.y + text_size.height / 2);
                cv::putText(cut_body, id_text, text_origin, font, font_scale, cv::Scalar(0, 0, 0), thickness);
            }
        }
        // Draw vertical lines for each camera
        int w = cut_body_map_w / number_of_cameras;
        int cam_counter = 1;
        for (auto config : cam_config_map)
        {
            cv::line(cut_body,
                     cv::Point(w * cam_counter, 0),
                     cv::Point(w * cam_counter, map_height),
                     cv::Scalar(161, 152, 125),
                     2,
                     cv::LINE_4);
            cam_counter++;
        }
        return cut_body;
    }
    catch (...)
    {
        std::cerr << "[getFirstDefectInProcessPanelMap] An exception occurred." << std::endl;
        cut_body = cv::Mat::zeros(cv::Size(cut_body_map_w, cut_body_map_h), CV_8UC3);
        cut_body = cv::Scalar(112, 48, 7);
        return cut_body;
    }
}

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
void WorldMapDrawer::drawDashedLine(cv::Mat &image, cv::Point pt1, cv::Point pt2, cv::Scalar color,
                                    int dash_length, int gap_length)
{
    double delta_x = pt2.x - pt1.x;
    double delta_y = pt2.y - pt1.y;

    double dash_length_sq = dash_length * dash_length;
    double gap_length_sq = gap_length * gap_length;

    double distance_sq = delta_x * delta_x + delta_y * delta_y;

    // Calculate the normalized direction vector
    double norm_dx = delta_x / sqrt(distance_sq);
    double norm_dy = delta_y / sqrt(distance_sq);

    cv::Point current_point = pt1;

    while (distance_sq > 0)
    {
        // Draw a dash
        cv::Point end_point;
        end_point.x = current_point.x + norm_dx * std::min(dash_length_sq, distance_sq);
        end_point.y = current_point.y + norm_dy * std::min(dash_length_sq, distance_sq);

        cv::line(image, current_point, end_point, color, 1); // 2 is the line thickness

        // Move to the next starting point
        current_point.x = end_point.x + norm_dx * gap_length;
        current_point.y = end_point.y + norm_dy * gap_length;

        // Update remaining distance
        distance_sq = (pt2.x - current_point.x) * (pt2.x - current_point.x) +
                      (pt2.y - current_point.y) * (pt2.y - current_point.y);
    }
}

/**
 * @brief Rect Boundary check and update teh value.
 *
 * @param rect - passed rect
 * @param size - image size param
 */
void WorldMapDrawer::rectBoundaryCheck(cv::Rect &rect, cv::Size size)
{
    if (rect.x < 0)
    {
        rect.x = 0;
    }
    if (rect.y < 0)
    {
        rect.y = 0;
    }
    if (rect.br().x > size.width)
    {
        rect.width = size.width - rect.x;
    }
    if (rect.br().y > size.height)
    {
        rect.height = size.height - rect.y;
    }
}

/**
 * @brief Update the map size.
 *
 * @param width - Current width of the map, Which is current camera total fov.
 * @param height - Current height of the map, Which is current camera to cutter distance.
 */
void WorldMapDrawer::updateMapSize(int width, int height)
{
    if (width > 0)
    {
        map_const_x = map_width / (float)width;
    }
    if (height > 0)
    {
        map_const_y = map_height / (float)height;
    }
    current_cam2stopper_distance_mm = height;
}

/**
 * @brief Update the map distances.
 *
 * @param cam2stopper_mm - Current camera to stopper distance.
 * @param cut_length - Current cut length.
 * @param dancer_end2cutter_mm - Current dancer end to cutter distance with offset.
 */
void WorldMapDrawer::updateMapDistances(int cam2stopper_mm, int cut_length, int dancer_end2cutter_mm)
{
    current_cam2stopper_distance_mm = original_cam2stopper_mm = cam2stopper_mm;
    dancer_end2stopper_distance_mm = dancer_end2cutter_mm;
    cut_length_mm = cut_length;
}