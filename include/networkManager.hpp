#include <iostream>
#include <string.h>

#include <mutex>
#include <robroInferenceAbstractClass.h>
#include <recipe.hpp>
#include <jobAndRecipeManager.hpp>

/**
 * @brief A Class to process the predictions made my the neural network
 * The processing of predictions can be to shortlist them, merge them into regions or account for recipes etc.
 */
class NetworkManager
{
    /**
     * @brief Inference object to be used for prediction. This is a pointer to the abstract class.
     * Supported networks are: yolov
     */
    RobroInferenceAbstractClass *inference{nullptr};

    /**
     * @brief mutex to lock the inference object for prediction and network loading
     * operations.
     */
    std::mutex mtx;

    /**
     * @brief It stores the threshold value to be used for prediction.
     */
    float threshold_value{0.5};

public:
    /**
     * @brief Destroy the Network Manger object
     *
     */
    ~NetworkManager();

    /**
     * @brief Set the threshold
     *
     * @param threshold - threshold value
     * @return true - if threshold is set
     * @return false - if threshold is not set
     */
    bool setThreshold(float threshold);

    /**
     * @brief Get the threshold
     *
     * @return float - threshold value
     */
    float getThreshold();

    /**
     * @brief Check if the network is loaded.
     *
     * @return true - if network is loaded
     * @return false -  if network is not loaded
     */
    bool isNetworkLoaded();

    /**
     * @brief Get the Last Prediction Time
     *
     * @return int - time taken for prediction
     */
    int getLastPredictionTime();

    /**
     * @brief Do Prediction
     *
     * @param img - image to be predicted
     * @param annotated_image - image with annotations
     * @return DarkHelp::PredictionResults - predictions
     */
    DarkHelp::PredictionResults doPrediction(cv::Mat &img, cv::Mat &annotated_image);

    /**
     * @brief Load the network with the given recipe.
     *
     * @param recipe - recipe to be loaded
     * @return true - if network is loaded
     * @return false - if network is not loaded
     */
    bool loadNetwork(Recipe recipe);
};
