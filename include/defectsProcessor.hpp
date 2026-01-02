/**
 * @file defectsProcessor.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Blue print of Defect Processors KWIS.
 * @version 0.1
 * @date 2023-04-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <recipe.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection2DArray.h>
#include <opencv4/opencv2/opencv.hpp>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <robroInferenceAbstractClass.h>
#include <worldMapDefect.hpp>

struct PointXY
{
    int int_x;
    int int_y;

    PointXY(int x, int y) : int_x(x), int_y(y) {}
};

struct Annotation
{
    int class_idx;
    std::string name;
    std::vector<PointXY> points;
    int required_sensitivity_x;
    int required_sensitivity_y;

    // This should be in the same namespace or globally so ADL finds it
    json getJson() const
    {
        json j;
        j["class_idx"] = class_idx;
        j["name"] = name;
        j["required_sensitivity_x"] = required_sensitivity_x;
        j["required_sensitivity_y"] = required_sensitivity_y;

        j["points"] = json::array();
        for (const auto &p : points)
        {
            json point_json;
            point_json["x"] = p.int_x;
            point_json["y"] = p.int_y;
            j["points"].push_back(point_json);
        }
        return j;
    }
};

/**
 * @brief A Class to process the predictions made by the neural network
 * The processing of predictions can be to shortlist them, merge them
 * into regions or account for recipes etc.
 */
class DefectsProcessor
{
    /**
     * @brief Store the names of the classes of the neural network.
     * The names are stored in a vector of strings.
     */
    std::vector<std::string> class_names;

    /**
     * @brief Y pixel per mm value. will used for converting the pixel values to mm
     * for sensitivity calculations.
     */
    float y_px_per_mm{0.0f};

    /**
     * @brief X pixel per mm value. will used for converting the pixel values to mm
     * for sensitivity calculations.
     */
    float x_px_per_mm{0.0f};

    /**
     * @brief Maximum sensitivity in pixels in the x and y direction.
     */
    const float MAX_SENSITIVITY_DIRECTION_PX{1000.0f};

    /**
     * @brief Maximum sensitivity value.
     */
    const int MAX_SENSITIVITY_VALUE{10};

    /**
     * @brief Full Ignore Disabled Classes
     */
    bool fully_ignore_disabled_classes{false};

    // Define sensitivity levels and corresponding thresholds (in mm)
    // std::map<int, float> sensitivityThresholds;

    std::vector<Annotation> annotations;

public:
    DefectsProcessor()
    {
        // Define sensitivity levels and corresponding thresholds (in mm)
        // Defects smaller in width and height than this size in mm will be marked: disabled
        // sensitivityThresholds = {
        //     {1, 500.0},
        //     {2, 250.0},
        //     {3, 100.0},
        //     {4, 50.0},
        //     {5, 40.0},
        //     {6, 20.0},
        //     {7, 10.0},
        //     {8, 5.0},
        //     {9, 2.5},
        //     {10, 1.0}};
    }

    /**
     * @brief update the pixel per mm values.
     *
     * @param x_px_per_mm - x pixel per mm value.
     * @param y_px_per_mm - y pixel per mm value.
     */
    void updatePxPerMM(float x_px_per_mm, float y_px_per_mm);

    /**
     * @brief update the names of the classes from the names file.
     *
     * @param names_file_path - path to the names file.
     * @return true - if the names file is updated.
     * @return false - if the names file is not updated.
     */
    bool updateNames(std::string names_file_path);

    /**
     * @brief Check the boundary of the rect and correct it if it is out of bounds.
     *
     * @param rect - rect to be checked.
     * @param size - size of the image.
     */
    void rectBoundaryCheck(cv::Rect &rect, cv::Size size);

    /**
     * @brief Return a rect of all boxes that belong to a particular class and have higher confidence than threshold.
     *
     * @param predictions - predictions made by the neural network.
     * @param filtered_rects - vector of rects of the predictions.
     * @param class_id - class ID of the predictions.
     * @param threshold - threshold value for the predictions.
     * @param max_confidence  - maximum confidence value of the predictions.
     * @param min_defect_area_px - minimum defect area in pixels.
     *
     */
    void filterBoundingBox_v2(DarkHelp::PredictionResults predictions,
                              std::vector<cv::Rect> &filtered_rects,
                              int class_id,
                              float threshold,
                              float &max_confidence,
                              int min_defect_area_px);
    /**
     * @brief Return a height and width in pixel format
     *
     * @param sensitivity - 1 to 100 sensititivy value from ui
     */
    std::pair<int, int> interpolateSize(int sensitivity);

    /**
     * @brief  Function to interpolate size X (width) based on sensitivity
     *
     */
    int interpolateSize_X(int sensitivity_x)
    {
        int minWidth = 0;
        int maxWidth = 2048;

        int width = minWidth + (maxWidth - minWidth) * (100 - sensitivity_x) / 99;

        return width;
    }

    /**
     * @brief  Function to interpolate size Y (height) based on sensitivity
     *
     */
    int interpolateSize_Y(int sensitivity_y)
    {
        int minHeight = 0;
        int maxHeight = 1024;

        int height = minHeight + (maxHeight - minHeight) * (100 - sensitivity_y) / 99;

        return height;
    }

    /**
     * breif: finding the required sensitivity (sensitivity_x and sensitivity_y) for defect
     * @param boxHeight: height of box in pixel
     * @param boxWidth : width of box in pixel
     */
    int findRequiredSensitivity(int boxHeight, int boxWidth);

    /**
     * @brief Find the required sensitivity pair (sensitivity_x and sensitivity_y) for a given box height and width.
     *
     * @param boxHeight - height of the box in pixels.
     * @param boxWidth - width of the box in pixels.
     */
    std::pair<int, int> findRequiredSensitivityPair(int boxHeight, int boxWidth)
    {
        int leftX = 0, leftY = 0;
        int rightX = 100, rightY = 100;
        int sensitivityX = 100, sensitivityY = 100;

        while (leftX <= rightX || leftY <= rightY)
        {
            if (leftX <= rightX)
            {
                int midX = (leftX + rightX) / 2;
                int interpolatedWidth = interpolateSize_X(midX);

                if (boxWidth >= interpolatedWidth)
                {
                    sensitivityX = midX;
                    rightX = midX - 1;
                }
                else
                {
                    leftX = midX + 1;
                }
            }

            if (leftY <= rightY)
            {
                int midY = (leftY + rightY) / 2;
                int interpolatedHeight = interpolateSize_Y(midY);

                if (boxHeight >= interpolatedHeight)
                {
                    sensitivityY = midY;
                    rightY = midY - 1;
                }
                else
                {
                    leftY = midY + 1;
                }
            }
        }

        return {sensitivityX, sensitivityY};
    }

    /**
     * @brief Return a rect of all boxes that belong to a particular class and have higher confidence than threshold.
     *
     * @param predictions - predictions made by the neural network.
     * @param class_id - class ID of the predictions.
     * @param threshold - threshold value for the predictions.
     * @param max_confidence  - maximum confidence value of the predictions.
     * @param min_defect_area_px - minimum defect area in pixels.
     *
     * @return std::vector<cv::Rect> - vector of rects of the predictions.
     */
    std::vector<cv::Rect> filterBoundingBox(DarkHelp::PredictionResults predictions,
                                            int class_id,
                                            float threshold,
                                            float &max_confidence,
                                            int min_defect_area_px);

    /**
     * @brief Morph the bounding boxes to get a single bounding box for each class.
     *
     * @param img_size - size of the image
     * @param boxes - vector of bounding boxes
     * @param output - output image
     */
    void morphBoundingBoxes(cv::Size img_size,
                            std::vector<cv::Rect> boxes,
                            cv::Mat &output);

    /**
     * @brief Draw bounding boxes on the annotated image.
     *
     * @param box - bounding box to be drawn.
     * @param class_id  - class ID of the predictions.
     * @param annotated_image - image to be annotated.
     */
    void drawBoundingBoxes(cv::Rect box, int class_id, cv::Mat &annotated_image);

    /**
     * @brief Get defect region by merging the predictions with confidence than threshold into regions.
     *
     * @param predictions - predictions made by the neural network.
     * @param annotated_image - image to be annotated.
     * @param recipe - recipe to be loaded
     *
     * @return vision_msgs::Detection2DArray - detection2DArray of the predictions.
     */
    vision_msgs::Detection2DArray getDefectRegion_v2(DarkHelp::PredictionResults predictions,
                                                     cv::Mat &annotated_image,
                                                     Recipe recipe);

    /**
     * @brief Get defect region by merging the predictions with confidence than threshold into regions.
     *
     * @param predictions - predictions made by the neural network.
     * @param annotated_image - image to be annotated.
     * @param num_classes - number of classes.
     * @param threshold - threshold value for the predictions.
     * @param min_defect_area_px - minimum defect area in pixels.
     *
     * @return vision_msgs::Detection2DArray - detection2DArray of the predictions.
     */
    vision_msgs::Detection2DArray getDefectRegion(DarkHelp::PredictionResults predictions,
                                                  cv::Mat &annotated_image,
                                                  int num_classes,
                                                  float threshold,
                                                  int min_defect_area_px, int cropping_margin);

    void setFullyIgnoreDisabledClasses(bool set_val);

    bool getFullyIgnoreDisabledClasses();

    /**
     * @brief Group defects by tiles.
     *
     * @param tileSize - size of the tile in pixels.
     * @param imageRows - number of rows in the image.
     * @param imageCols - number of columns in the image.
     * @return std::vector<std::vector<Annotation>> - vector of annotations grouped by tiles.
     */
    std::vector<std::vector<Annotation>> groupDefectsByTile(
        int tileSize,
        int imageRows,
        int imageCols)
    {
        int numTilesPerRow = (imageCols + tileSize - 1) / tileSize; // ceil division
        int numTilesPerCol = (imageRows + tileSize - 1) / tileSize;
        int numTilesTotal = numTilesPerRow * numTilesPerCol;

        std::vector<std::vector<Annotation>> tiles_annotation(numTilesTotal);

        for (const Annotation &defect : annotations)
        {
            // Compute bounding box from points (if not already available)
            int min_x = std::min({defect.points[0].int_x, defect.points[1].int_x, defect.points[2].int_x, defect.points[3].int_x});
            int min_y = std::min({defect.points[0].int_y, defect.points[1].int_y, defect.points[2].int_y, defect.points[3].int_y});
            int max_x = std::max({defect.points[0].int_x, defect.points[1].int_x, defect.points[2].int_x, defect.points[3].int_x});
            int max_y = std::max({defect.points[0].int_y, defect.points[1].int_y, defect.points[2].int_y, defect.points[3].int_y});

            int startTileCol = min_x / tileSize;
            int endTileCol = max_x / tileSize;
            int startTileRow = min_y / tileSize;
            int endTileRow = max_y / tileSize;

            for (int tileRow = startTileRow; tileRow <= endTileRow; ++tileRow)
            {
                int tileStartY = tileRow * tileSize;
                int tileEndY = tileStartY + tileSize;

                for (int tileCol = startTileCol; tileCol <= endTileCol; ++tileCol)
                {
                    int tileStartX = tileCol * tileSize;
                    int tileEndX = tileStartX + tileSize;

                    // Calculate overlap
                    int interLeft = std::max(min_x, tileStartX);
                    int interTop = std::max(min_y, tileStartY);
                    int interRight = std::min(max_x, tileEndX);
                    int interBottom = std::min(max_y, tileEndY);

                    if (interRight <= interLeft || interBottom <= interTop)
                        continue; // No overlap

                    int tileIndex = tileRow * numTilesPerRow + tileCol;
                    if (tileIndex < 0 || tileIndex >= numTilesTotal)
                        continue;

                    // Create a new Annotation relative to this tile
                    Annotation ann;
                    ann.class_idx = defect.class_idx;
                    ann.name = defect.name;
                    ann.required_sensitivity_x = defect.required_sensitivity_x;
                    ann.required_sensitivity_y = defect.required_sensitivity_y;

                    ann.points.push_back({interLeft - tileStartX, interTop - tileStartY});     // top-left
                    ann.points.push_back({interLeft - tileStartX, interBottom - tileStartY});  // bottom-left
                    ann.points.push_back({interRight - tileStartX, interBottom - tileStartY}); // bottom-right
                    ann.points.push_back({interRight - tileStartX, interTop - tileStartY});    // top-right
    

                    tiles_annotation[tileIndex].push_back(std::move(ann));
                }
            }
        }

        return tiles_annotation;
    }
};