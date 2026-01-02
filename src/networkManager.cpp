#include <networkManager.hpp>
#include "robroYOLOV8.h"

/**
 * @brief Destroy the Network Manger:: Network Manger object
 *
 */
NetworkManager::~NetworkManager()
{
    if (inference != nullptr)
    {
        delete inference;
    }
}

/**
 * @brief Set the threshold
 *
 * @param threshold - threshold value
 * @return true - if threshold is set
 * @return false - if threshold is not set
 */
bool NetworkManager::setThreshold(float threshold)
{
    std::lock_guard<std::mutex> lock(mtx);
    threshold_value = threshold;
    return inference->setThresholdTo(threshold_value);
}

/**
 * @brief Get the threshold
 *
 * @return float - threshold value
 */
float NetworkManager::getThreshold()
{
    return threshold_value;
}

/**
 * @brief Check if the network is loaded.
 *
 * @return true - if network is loaded
 * @return false -  if network is not loaded
 */
bool NetworkManager::isNetworkLoaded()
{
    return inference->isInitialized();
}

/**
 * @brief Get the Last Prediction Time
 *
 * @return int - time taken for prediction
 */
int NetworkManager::getLastPredictionTime()
{
    return inference->getLastInferenceTimeInMs();
}

/**
 * @brief Do Prediction
 *
 * @param img - image to be predicted
 * @param annotated_image - image with annotations
 * @return DarkHelp::PredictionResults - predictions
 */
DarkHelp::PredictionResults NetworkManager::doPrediction(cv::Mat &img, cv::Mat &annotated_image)
{
    img.copyTo(annotated_image);
    std::lock_guard<std::mutex> lock(mtx);
    inference->predict(img, false);

    return inference->predictions;
}

/**
 * @brief Load the network with the given recipe.
 *
 * @param recipe - recipe to be loaded
 * @return true - if network is loaded
 * @return false - if network is not loaded
 */
bool NetworkManager::loadNetwork(Recipe recipe)
{
    // lock the inference mutex
    std::lock_guard<std::mutex> lock(mtx);
    if (inference != nullptr)
    {
        delete inference;
    }

    std::cout << "Loading Network. Recipe: " << recipe.name;

    if (recipe.network_type == "yolov8")
    {
        std::cout << " Engine: " << recipe.engine_file_path << " Names: " << recipe.names_file_path << std::endl;
        inference = new RobroYOLOV8(recipe.engine_file_path, recipe.names_file_path, recipe.threshold, true, false);
    }

    if (inference == nullptr)
    {
        std::cout << "\033[1;31m [ERROR] Inference object not initialized: \033[0m" << std::endl;
        return false;
    }

    if (!inference->isInitialized())
    {
        return false;
    }

    // Set the thresholds
    threshold_value = recipe.threshold;
    inference->setNMSValueTo(recipe.nms_threshold);

    // Dummy prediction to initialize the network
    inference->predict(cv::Mat(1024, 1024, CV_8UC3, cv::Scalar(0)));
    return true;
}
