
#ifndef WORLD_MAP_DEFECT_HPP
#define WORLD_MAP_DEFECT_HPP

#include <string>
#include <vision_msgs/Detection2D.h>

/**
 * @brief class to store the defect information in the world map.
 */
class WorldMapDefect
{
public:
    /**
     * @brief Camera id of the defect.
     */
    std::string cam_id{""};

    /**
     *  @brief Storing full frame path
     */
    std::string full_frame_path{""};

    /**
     * @brief Defect id of the defect.
     */
    int64_t defect_id{0};

    int64_t robro_roll_id{0};

    /**
     * @brief Detection of the defect.
     */
    vision_msgs::Detection2D detection;

    /**
     * @brief is stopping command issued for this defect
     */
    bool stopping_command_issued{false};

    /**
     * @brief is defect class disabled for stopping
     */
    bool is_defect_class_disabled{false};

    /**
     * @brief Sequence id of the defect which represents the order frame in which the defect was detected.
     */
    int sequence_id{-1};

    /**
     * @brief Defect y coordinate with respect to the frame. which is used in dynamic defect distance calculation.
     */
    float defect_y_coord_wrt_frame{0.0f};
};

#endif // WORLD_MAP_DEFECT_HPP