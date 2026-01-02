#include <defectsProcessor.hpp>

/**
 * @brief update the pixel per mm values.
 *
 * @param x_px_per_mm - x pixel per mm value.
 * @param y_px_per_mm - y pixel per mm value.
 */
void DefectsProcessor::updatePxPerMM(float x_px_per_mm, float y_px_per_mm)
{
    this->x_px_per_mm = x_px_per_mm;
    this->y_px_per_mm = y_px_per_mm;
}

/**
 * @brief update the names of the classes from the names file.
 *
 * @param names_file_path - path to the names file.
 * @return true - if the names file is updated.
 * @return false - if the names file is not updated.
 */
bool DefectsProcessor::updateNames(std::string names_file_path)
{
    std::ifstream ifs(names_file_path.c_str());
    try
    {
        // Load the class names
        if (!ifs.is_open())
        {
            throw std::runtime_error("Could not open the names file");
        }
        std::string line;
        class_names.clear();
        while (std::getline(ifs, line))
        {
            class_names.push_back(line);
        }
        ifs.close();
        return true;
    }
    catch (const std::exception &e)
    {
        ifs.close();
        std::cout << "[ERROR] Names not updated:" << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Check the boundary of the rect and correct it if it is out of bounds.
 *
 * @param rect - rect to be checked.
 * @param size - size of the image.
 */
void DefectsProcessor::rectBoundaryCheck(cv::Rect &rect, cv::Size size)
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
void DefectsProcessor::filterBoundingBox_v2(DarkHelp::PredictionResults predictions,
                                            std::vector<cv::Rect> &filtered_rects,
                                            int class_id,
                                            float threshold,
                                            float &max_confidence,
                                            int min_defect_area_px)
{

    for (const auto &prediction : predictions)
    {
        if (prediction.best_class == class_id && prediction.best_probability >= threshold && prediction.rect.area() > min_defect_area_px)
        {
            if (prediction.best_probability > max_confidence)
            {
                max_confidence = prediction.best_probability;
            }
            filtered_rects.push_back(prediction.rect);
        }
    }
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
std::vector<cv::Rect> DefectsProcessor::filterBoundingBox(DarkHelp::PredictionResults predictions,
                                                          int class_id,
                                                          float threshold,
                                                          float &max_confidence,
                                                          int min_defect_area_px)
{
    max_confidence = 0.0;

    std::vector<cv::Rect> filtered_rects;

    for (const auto &prediction : predictions)
    {
        if (prediction.best_class == class_id && prediction.best_probability >= threshold && prediction.rect.area() > min_defect_area_px)
        {
            if (prediction.best_probability > max_confidence)
            {
                max_confidence = prediction.best_probability;
            }
            filtered_rects.push_back(prediction.rect);
        }
    }
    return filtered_rects;
}

/**
 * @brief Morph the bounding boxes to get a single bounding box for each class.
 *
 * @param img_size - size of the image
 * @param boxes - vector of bounding boxes
 * @param output - output image
 */
void DefectsProcessor::morphBoundingBoxes(cv::Size img_size,
                                          std::vector<cv::Rect> boxes,
                                          cv::Mat &output)
{
    // Make an empty image
    cv::Mat img = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);

    // If there are more than 10 boxes of a given class in this frame,
    // let's call this whole frame as defect
    // if (boxes.size() > 10)
    // {
    //     // remove all small small boxes and add 1 full img size box
    //     boxes.clear();
    //     boxes.push_back(cv::Rect(0, 0, img_size.width, img_size.height));
    // }

    int morph_size{25};

    // For each box, draw white rectangles in the black image created above.
    for (cv::Rect box : boxes)
    {
        cv::rectangle(img, box.tl(), box.br(), 255, -1, 1);
        // The morph size for closing in the next step to be
        // calculated automatically by a factor of w,h of rectangles
        morph_size = std::max(morph_size, std::min(3 * box.width, 3 * box.height));
    }

    // Apply closing operation to fill the holes in the white rectangles.
    morph_size /= 2;

    cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(img, output, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);

    cv::bitwise_or(img, output, output);
}

/**
 * @brief Draw bounding boxes on the annotated image.
 *
 * @param box - bounding box to be drawn.
 * @param class_id  - class ID of the predictions.
 * @param annotated_image - image to be annotated.
 */
void DefectsProcessor::drawBoundingBoxes(cv::Rect box, int class_id, cv::Mat &annotated_image)
{
    // Draw different color boxes for different classes
    if (class_id == 0)
    {
        cv::rectangle(annotated_image, box, cv::Scalar(0, 0, 255), 20, cv::LINE_8);
    }
    else if (class_id == 1)
    {
        cv::rectangle(annotated_image, box, cv::Scalar(0, 255, 0), 20, cv::LINE_8);
    }
    else if (class_id == 2)
    {
        cv::rectangle(annotated_image, box, cv::Scalar(0, 255, 255), 20, cv::LINE_8);
    }
    else if (class_id == 3)
    {
        cv::rectangle(annotated_image, box, cv::Scalar(255, 0, 0), 20, cv::LINE_8);
    }
    else
    {
        cv::rectangle(annotated_image, box, cv::Scalar(255, 255, 0), 20, cv::LINE_8);
    }
}

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
vision_msgs::Detection2DArray DefectsProcessor::getDefectRegion(DarkHelp::PredictionResults predictions,
                                                                cv::Mat &annotated_image,
                                                                int num_classes,
                                                                float threshold,
                                                                int min_defect_area_px, int cropping_margin)
{
    vision_msgs::Detection2DArray regions;
    annotations.clear();
    float max_confidence = 0.0;
    if (!predictions.empty())
    {
        cv::Mat defect_cropping_img = annotated_image.clone();
        // For each class
        for (int class_id = 0; class_id < num_classes; class_id++)
        {
            cv::Mat output;
            std::vector<std::vector<cv::Point>> contours;

            // Get the boxes belonging to this class
            std::vector<cv::Rect> boxes = filterBoundingBox(predictions, class_id, threshold,
                                                            max_confidence, min_defect_area_px);

            // Morph the boxes to get the defect region
            morphBoundingBoxes(annotated_image.size(),
                               boxes,
                               output);
            // Find contours in the binary image
            cv::findContours(output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (size_t i = 0; i < contours.size(); i++)
            {
                // Get the box of the contour
                cv::Rect box = cv::boundingRect(contours[i]);

                // Draw the box on the annotated image.
                vision_msgs::ObjectHypothesisWithPose result_score;
                vision_msgs::Detection2D this_detection;

                // adding defect best class to id & probability to score.
                result_score.id = class_id;
                result_score.score = max_confidence;

                // Copy the defect portion img to the detection object
                sensor_msgs::ImagePtr this_defect;
                this_detection.source_img.header.stamp = ros::Time::now();
                this_detection.source_img.header.frame_id = class_names[class_id] + " " + std::to_string((int)(max_confidence * 100)) + "%";

                Annotation this_annotation;
                this_annotation.class_idx = class_id;
                this_annotation.name = class_names[class_id];
                this_annotation.required_sensitivity_x = 0;
                this_annotation.required_sensitivity_y  = 0;
                this_annotation.points.push_back(PointXY(box.tl().x, box.tl().y));
                this_annotation.points.push_back(PointXY(box.tl().x, box.br().y));
                this_annotation.points.push_back(PointXY(box.br().x, box.tl().y));
                this_annotation.points.push_back(PointXY(box.br().x, box.br().y));
                annotations.push_back(this_annotation);

                // The bbox is center with w/2 and h/2 around it.
                this_detection.bbox.center.x = (double)((box.br().x + box.tl().x) / 2.0);
                this_detection.bbox.center.y = (double)((box.br().y + box.tl().y) / 2.0);
                this_detection.bbox.size_x = (double)box.width / 2.0;
                this_detection.bbox.size_y = (double)box.height / 2.0;
                this_detection.results.push_back(result_score);

                // Check if the box is within the image boundary
                rectBoundaryCheck(box, annotated_image.size());
                drawBoundingBoxes(box, class_id, annotated_image);

                // margin for cropping
                box.x -= cropping_margin;
                box.y -= cropping_margin;
                box.width += 2 * cropping_margin;
                box.height += 2 * cropping_margin;

                rectBoundaryCheck(box, annotated_image.size());
                this_defect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", defect_cropping_img(box)).toImageMsg();
                this_detection.source_img = *this_defect;

                if (class_id == 1 && box.width < 800)
                {
                    std::cout << "IGNORING SMALL SHUTTLE FAULT!\n";
                    continue;
                }
                // Add the detection to the detection array
                regions.detections.push_back(this_detection);
            }
        }
    }
    else // no defects found
    {
        regions.detections.clear();
    }

    return regions;
}

// Function to interpolate size based on sensitivity
std::pair<int, int> DefectsProcessor::interpolateSize(int sensitivity)
{
    int minHeight = 0;
    int maxHeight = 1024;
    int minWidth = 0;
    int maxWidth = 2048;

    int height = minHeight + (maxHeight - minHeight) * (100 - sensitivity) / 99;
    int width = minWidth + (maxWidth - minWidth) * (100 - sensitivity) / 99;

    return {height, width};
}

// Function to find the required sensitivity for a given box height and width
int DefectsProcessor::findRequiredSensitivity(int boxHeight, int boxWidth)
{

    int left = 0;
    int right = 100;
    int sensitivity = 100;

    while (left <= right)
    {
        int mid = (left + right) / 2;
        std::pair<int, int> size = interpolateSize(mid);

        if (boxHeight >= size.first || boxWidth >= size.second)
        {
            // mid satisfies the condition; try to see if there is a smaller mid
            sensitivity = mid;
            right = mid - 1;
        }
        else
        {
            // mid does not satisfy; we need a larger mid
            left = mid + 1;
        }
    }

    return sensitivity;
}

/**
 * @brief Get defect region by merging the predictions with confidence than threshold into regions.
 *
 * @param predictions - predictions made by the neural network.
 * @param annotated_image - image to be annotated.
 * @param recipe - recipe to be loaded
 * @return vision_msgs::Detection2DArray - detection2DArray of the predictions.
 */
vision_msgs::Detection2DArray DefectsProcessor::getDefectRegion_v2(DarkHelp::PredictionResults predictions,
                                                                   cv::Mat &annotated_image,
                                                                   Recipe recipe)
{
    int num_classes = recipe.number_of_classes;
    float threshold = recipe.threshold;
    int min_defect_area_px = recipe.min_defect_area_px;
    json groups = recipe.groups;
    // int required_sensitivity = 100;

    vision_msgs::Detection2DArray regions;
    annotations.clear();
    float max_confidence = 0.0;
    if (!predictions.empty())
    {
        cv::Mat defect_cropping_img = annotated_image.clone();

        // For each class
        for (auto it = groups.begin(); it != groups.end(); ++it)
        {
            cv::Mat output;
            int group_id = std::stoi(it.key());
            std::vector<int> group_classes = it.value()["ids"];

            std::pair<int, int> sensitivity_size = {interpolateSize_X(int(it.value()["sensitivity_x"])), interpolateSize_Y(int(it.value()["sensitivity_y"]))};

            bool enabled = it.value()["enabled"];

            if (!enabled && fully_ignore_disabled_classes)
            {
                continue;
            }

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Rect> boxes;
            for (int class_id : group_classes)
            {
                // Get the boxes belonging to this class
                filterBoundingBox_v2(predictions, boxes, class_id, threshold,
                                     max_confidence, min_defect_area_px);
            }
            // Morph the boxes to get the defect region
            morphBoundingBoxes(annotated_image.size(),
                               boxes,
                               output);

            // Find contours in the binary image
            cv::findContours(output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (size_t i = 0; i < contours.size(); i++)
            {
                // Draw the box on the annotated image.
                vision_msgs::ObjectHypothesisWithPose result_score;
                vision_msgs::Detection2D this_detection;
                json detection_info;

                // Copy the defect portion img to the detection object
                sensor_msgs::ImagePtr this_defect;

                // Get the box of the contour
                cv::Rect box = cv::boundingRect(contours[i]);
                cv::Rect cropbox = cv::boundingRect(contours[i]);

                result_score.id = group_id;
                result_score.score = max_confidence;
                double confidence_threshold = it.value()["threshold"];

                bool is_small = (box.height < sensitivity_size.second && box.width < sensitivity_size.first);
                bool is_confidence_low = (result_score.score < confidence_threshold);

                // adding defect best class to id & probability to score.
                if (is_small)
                {
                    std::cout << "Ignoring the Defect: H: " << box.height << " W: " << box.width << "\n";
                }

                // calculate the required sensitivity for the box
                std::pair<int, int> required_sensitivity = findRequiredSensitivityPair(box.height, box.width);

                if ((is_small || is_confidence_low) && fully_ignore_disabled_classes)
                {

                    continue;
                }

                this_detection.source_img.header.stamp = ros::Time::now();
                detection_info["title"] = class_names[group_id] + " " + std::to_string((int)(max_confidence * 100)) + "%";
                detection_info["class_name"] = class_names[group_id];
                detection_info["is_disabled"] = ((!enabled) || is_small || is_confidence_low);
                detection_info["area"] = box.area();
                detection_info["sensitivity_x"] = int(it.value()["sensitivity_x"]);
                detection_info["sensitivity_y"] = int(it.value()["sensitivity_y"]);
                detection_info["required_sensitivity_x"] = required_sensitivity.first;
                detection_info["required_sensitivity_y"] = required_sensitivity.second;

                Annotation this_annotation;
                this_annotation.class_idx = group_id;
                this_annotation.name = class_names[group_id];
                this_annotation.required_sensitivity_x = required_sensitivity.first;
                this_annotation.required_sensitivity_y = required_sensitivity.second;
                this_annotation.points.push_back(PointXY(box.tl().x, box.tl().y));
                this_annotation.points.push_back(PointXY(box.tl().x, box.br().y));
                this_annotation.points.push_back(PointXY(box.br().x, box.tl().y));
                this_annotation.points.push_back(PointXY(box.br().x, box.br().y));
                annotations.push_back(this_annotation);

                this_detection.header.frame_id = detection_info.dump();
                // std::cout << this_detection.header.frame_id << std::endl;
                // The bbox is center with w/2 and h/2 around it.
                this_detection.bbox.center.x = (double)((box.br().x + box.tl().x) / 2.0);
                this_detection.bbox.center.y = (double)((box.br().y + box.tl().y) / 2.0);
                this_detection.bbox.size_x = (double)box.width / 2.0;
                this_detection.bbox.size_y = (double)box.height / 2.0;
                this_detection.results.push_back(result_score);

                // Check if the box is within the image boundary
                rectBoundaryCheck(box, annotated_image.size());
                drawBoundingBoxes(box, group_id, annotated_image);
                box.x -= recipe.cropping_margin;
                box.y -= recipe.cropping_margin;
                box.width += 2 * recipe.cropping_margin;
                box.height += 2 * recipe.cropping_margin;

                rectBoundaryCheck(box, annotated_image.size());
                this_defect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", defect_cropping_img(box)).toImageMsg();
                this_detection.source_img = *this_defect;
                this_detection.source_img.header.frame_id = class_names[group_id] +
                                                            "[cX:" + std::to_string(int(it.value()["sensitivity_x"])) + ",cY:" + std::to_string(int(it.value()["sensitivity_y"])) + "] " + "[rX:" + std::to_string(required_sensitivity.first) + ",rY:" + std::to_string(required_sensitivity.second) + "] " + std::to_string(int(max_confidence * 100)) + "%";

                // Add the detection to the detection array
                regions.detections.push_back(this_detection);
            }
        }
    }
    else // no defects found
    {
        regions.detections.clear();
    }

    return regions;
}

void DefectsProcessor::setFullyIgnoreDisabledClasses(bool set_val)
{
    this->fully_ignore_disabled_classes = set_val;
}

bool DefectsProcessor::getFullyIgnoreDisabledClasses()
{
    return this->fully_ignore_disabled_classes;
}