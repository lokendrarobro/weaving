#ifndef CAMERA_INDICATOR_HPP
#define CAMERA_INDICATOR_HPP

#include <iostream>
#include <string>
#include <chrono>
#include <map>

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::seconds;
using std::chrono::time_point;

/**
 * @brief Enum representing different states of a camera.
 */
enum class CameraState
{
    OPERATIONAL, /**< Camera is operational. */
    ERROR,       /**< Camera encountered an error. */
    BACKLOG      /**< Camera has a backlog. */
};

/**
 * @brief Enum representing different types of errors.
 */
enum class ErrorType
{
    GRABBING,  /**< Error occurred during image grabbing. */
    INFERENCE, /**< Error occurred during inference. */
    QUEUE,     /**< Error occurred due to queue. */
    COUNTER,   /**< Error occurred related to counter. */
    EXPOSURE   /**< Error occurred during exposure. */
};

/**
 * @brief Class representing an indicator for a camera's status.
 */
class CameraIndicator
{
private:
    CameraState current_state;                                              /**< Current state of the camera. */
    ErrorType last_error;                                                   /**< Last error that occurred. */
    std::string color;                                                      /**< Color representation of the camera indicator. */
    int queue_size;                                                         /**< Size of the camera's queue. */
    std::map<ErrorType, time_point<high_resolution_clock>> last_error_time; /**< Time of last error for each error type. */
    double total_inference_time;                                            /**< Total inference time. */
    int total_inferences;                                                   /**< Total number of inferences. */

    /**
     * @brief Updates the time of last error for a given error type.
     * @param status Whether an error occurred or not.
     * @param type Type of the error.
     *
     * @return false - if there was an error.
     *  true - if there was no error.
     */
    bool UpdateErrorTime(bool status, ErrorType type);

    /**
     * @brief Updates the state of the camera based on various conditions.
     * @param camera_status Status of the camera.
     * @param inference_status Status of inference.
     * @param inference_time Time taken for inference.
     * @param queue_size Size of the camera's queue.
     * @param counter_status Status of the counter.
     * @param exposure_status Status of the exposure.
     */
    void UpdateState(bool camera_status, bool inference_status, double inference_time,
                     int queue_size, bool counter_status, bool exposure_status);

public:
    /**
     * @brief Constructor for CameraIndicator class.
     */
    CameraIndicator();

    /**
     * @brief Updates the status of the camera.
     * @param camera_status Status of the camera.
     * @param inference_status Status of inference.
     * @param inference_time Time taken for inference.
     * @param new_queue_size New size of the camera's queue.
     * @param counter_status Status of the counter.
     * @param exposure_status Status of the exposure.
     */
    void UpdateStatus(bool camera_status, bool inference_status, double inference_time,
                      int new_queue_size, bool counter_status, bool exposure_status);

    /**
     * @brief Gets the color of the camera indicator.
     * @return Color in hexadecimal format.
     */
    std::string GetColor() const;

    /**
     * @brief Gets the average inference time.
     * @return Average inference time.
     */
    double GetAverageInferenceTime() const;

    /**
     * @brief Gets the last error message.
     *
     * @return std::string - Last error message.
     */
    std::string GetLastErrorMessage() const;
};

#endif // CAMERA_INDICATOR_HPP