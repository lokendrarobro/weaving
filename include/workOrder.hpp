#ifndef WORK_ORDER_HPP
#define WORK_ORDER_HPP

#include "baseType.hpp"
#include <sstream> // for std::to_string

class WorkOrder : public BaseType
{
public:
    int64_t work_order_id;
    std::string order_name;
    int cut_length;
    int target_pcs;
    int current_pcs;
    int status;
    int gsm;
    float width;
    int batch_current_pcs;

    bool isValid{false};

    WorkOrder() : BaseType("WorkOrder"), cut_length(0), target_pcs(0), current_pcs(0), status(0),batch_current_pcs{0} {}

    WorkOrder(int64_t _work_order_id, std::string _order_name, int _cut_length, int _target_pcs, int _current_pcs, int _status)
        : BaseType("WorkOrder"),
          work_order_id(_work_order_id), order_name(_order_name), cut_length(_cut_length),
          target_pcs(_target_pcs), current_pcs(_current_pcs), status(_status),
          isValid(true),batch_current_pcs{0} {}

          json toJSON() override
          {
              return json{
                  {"work_order_id", work_order_id},
                  {"order_name", order_name},
                  {"cut_length", cut_length},
                  {"target_pcs", target_pcs},
                  {"current_pcs", current_pcs},
                  {"status", status}};
          }
          
    bool loadJSON(json type) override
    {
        if (type.empty())
        {
            // Clear previous values
            work_order_id =0;
            order_name.clear();
            cut_length = 0;
            target_pcs = 0;
            current_pcs = 0;
            status = 0;

            isValid = false;
            return false;
        }

        try
        {
            work_order_id = type.at("work_order_id").get<int64_t>();
            order_name = type.at("order_name").get<std::string>();
            cut_length = type.at("cut_length").get<int>();
            target_pcs = type.at("target_pcs").get<int>();
            current_pcs = type.at("current_pcs").get<int>();
            status = type.at("status").get<int>();

            isValid = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Failed to load WorkOrder from JSON: " << e.what() << std::endl;

            // Clear data on failure
            work_order_id =0;
            order_name.clear();
            cut_length = 0;
            target_pcs = 0;
            current_pcs = 0;
            status = 0;

            isValid = false;
            return false;
        }
    }

    /**
     * @brief Get the target piece count as a string.
     *
     * @return std::string - target_pcs if valid, else "NA"
     */
    std::string getTargetPcs() const
    {
        return isValid ? std::to_string(target_pcs) : "NA";
    }

    /**
     * @brief Check if the current work order is completed.
     *
     * @return true if current_pcs == target_pcs and isValid is true
     * @return false otherwise
     */
    bool isCompleted() const
    {
        return isValid && (current_pcs >= target_pcs);
    }

    bool isBatchComplete(int batch_count)
    {
        return (current_pcs%batch_count==0) && (current_pcs!=0);

    }

    /**
     * @brief Resets all fields of the work order to default values.
     */
    void reset()
    {
        work_order_id =0;
        order_name.clear();
        cut_length = 0;
        target_pcs = 0;
        current_pcs = 0;
        status = 0;
        isValid = false;
    }
};

#endif // WORK_ORDER_HPP
