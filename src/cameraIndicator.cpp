#include <cameraIndicator.hpp>

/**
 * @brief Updates the time of last error for a given error type.
 * @param status Whether an error occurred or not.
 * @param type Type of the error.
 * 
 * @return false - if there was an error.
 *  true - if there was no error.
 */
bool CameraIndicator::UpdateErrorTime(bool status, ErrorType type)
{
    if (!status)
    {
        last_error_time[type] = high_resolution_clock::now();
        last_error = type;
        current_state = CameraState::ERROR; // Any error will set the state to ERROR
        return false;
    }
    return true;
}

/**
 * @brief Updates the state of the camera based on various conditions.
 * @param camera_status Status of the camera.
 * @param inference_status Status of inference.
 * @param inference_time Time taken for inference.
 * @param queue_size Size of the camera's queue.
 * @param counter_status Status of the counter.
 * @param exposure_status Status of the exposure.
 */
void CameraIndicator::UpdateState(bool camera_status, bool inference_status, double inference_time,
                                  int queue_size, bool counter_status, bool exposure_status)
{
    int all_good = true;
    // Update states based on the error conditions
    all_good &= UpdateErrorTime(camera_status, ErrorType::GRABBING);
    all_good &= UpdateErrorTime(inference_status, ErrorType::INFERENCE);
    all_good &= UpdateErrorTime(!(queue_size > 30), ErrorType::QUEUE); // Example condition for queue size
    all_good &= UpdateErrorTime(counter_status, ErrorType::COUNTER);
    all_good &= UpdateErrorTime(exposure_status, ErrorType::EXPOSURE);
    if (all_good)
    {
        current_state = CameraState::OPERATIONAL;
    }
    // Update inference time tracking
    if (inference_status)
    {
        total_inference_time += inference_time;
        ++total_inferences;
    }

    // Update overall state based on error conditions and performance
    if (current_state != CameraState::ERROR)
    { // Error state takes precedence
        if (queue_size > 30)
        {
            current_state = CameraState::BACKLOG;
        }
        else
        {
            current_state = CameraState::OPERATIONAL; // Default to operational if no other flags raised
        }
    }

    // Update indicator color based on the current state
    switch (current_state)
    {
    case CameraState::OPERATIONAL:
        color = "#187318"; // Green
        break;
    case CameraState::ERROR:
        color = "#ff0000"; // Red
        break;
    case CameraState::BACKLOG:
        color = "#0000ff"; // Blue
        break;
    default:
        // Gray
        color = "#808080"; // If for any reason the state is none of the above
        break;
    }
}

/**
 * @brief Constructor for CameraIndicator class.
 */
CameraIndicator::CameraIndicator()
    : current_state(CameraState::OPERATIONAL), color("#187318"),
      queue_size(0), total_inference_time(0), total_inferences(0) {}

/**
 * @brief Updates the status of the camera.
 * @param camera_status Status of the camera.
 * @param inference_status Status of inference.
 * @param inference_time Time taken for inference.
 * @param new_queue_size New size of the camera's queue.
 * @param counter_status Status of the counter.
 * @param exposure_status Status of the exposure.
 */
void CameraIndicator::UpdateStatus(bool camera_status, bool inference_status, double inference_time,
                                   int new_queue_size, bool counter_status, bool exposure_status)
{
    queue_size = new_queue_size; // Update current queue size
    UpdateState(camera_status, inference_status, inference_time, queue_size, counter_status, exposure_status);
}

/**
 * @brief Gets the color of the camera indicator.
 * @return Color in hexadecimal format.
 */
std::string CameraIndicator::GetColor() const
{
    return color;
}

/**
 * @brief Gets the average inference time.
 * @return Average inference time.
 */
double CameraIndicator::GetAverageInferenceTime() const
{
    return total_inferences > 0 ? total_inference_time / total_inferences : 0.0;
}

/**
 * @brief Gets the last error message.
 *
 * @return std::string - Last error message.
 */
std::string CameraIndicator::GetLastErrorMessage() const
{
    switch (last_error)
    {
    case ErrorType::GRABBING:
        return "Error grabbing image";
    case ErrorType::INFERENCE:
        return "Error during inference";
    case ErrorType::QUEUE:
        return "Queue size too large" + std::to_string(queue_size);
    case ErrorType::COUNTER:
        return "Error with counter";
    case ErrorType::EXPOSURE:
        return "Error with exposure";
    default:
        return "";
    }
}