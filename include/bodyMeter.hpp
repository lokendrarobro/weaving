#ifndef BODY_METER_HPP
#define BODY_METER_HPP

/**
 * @class BodyMeter
 * @brief Represents the meter readings for a production body.
 *
 * This class is used to track different meter readings in a production
 * environment, including primary, secondary, tertiary, and defective  fabric measurements.
 */
class BodyMeter
{
public:
    /// Meter count for primary (usable) material
    float primary_meter{0.0};

    /// Meter count for secondary material (partially usable or second-grade)
    float secondary_meter{0.0};

    /// Meter count for tertiary material (third-grade or rework)
    float tertiary_meter{0.0};

    /// Meter count for defective or waste material
    float defective_meter{0.0};

    int defective_weight_factor{1};

    /**
     * @brief Resets all meter values to 0.0.
     *
     * Useful for initializing or clearing the current readings.
     */
    void reset()
    {
        primary_meter = 0.0;
        secondary_meter = 0.0;
        tertiary_meter = 0.0;
        defective_meter = 0.0;
    }

    void setDefectiveWeightFactor(std::string type)
    {
        if (type == "Single")
        {
            defective_weight_factor = 1;
        }
        else if (type == "Double")
        {
            defective_weight_factor = 2;
        }
        else
        {
            defective_weight_factor = 1;
        }
    }
};

#endif