#ifndef BATCH_COUNT_HPP
#define BATCH_COUNT_HPP

class BatchCount
{
public:
    int target_batch_count;
    int current_pcs;
    bool isValid{false};

    BatchCount() : target_batch_count(0), current_pcs(0) {}

    BatchCount(int _target_batch_count)
        : target_batch_count(_target_batch_count), current_pcs(0)
    {
        isValid = (target_batch_count != 0);
    }

    /**
     * @brief Set the target batch count.
     * @param count Target batch count
     */
    void setTargetBatchCount(int count)
    {
        target_batch_count = count;
        isValid = (target_batch_count != 0);
    }

    /**
     * @brief Get the target batch count.
     * @return int - target batch count if valid, else 0
     */
    int getTargetBatchCount() const
    {
        return isValid ? target_batch_count : 0;
    }

    /**
     * @brief Get the current piece count.
     * @return int - current_pcs
     */
    int getCurrentPcs() const
    {
        return current_pcs;
    }

    /**
     * @brief Check if the batch is complete based on internal current_pcs.
     * @return true if current_pcs % target_batch_count == 0 and current_pcs != 0
     */
    bool isCompleted() const
    {
        if (!isValid || target_batch_count == 0)
            return false;

        return (current_pcs % target_batch_count == 0) && (current_pcs != 0);
    }

    /**
     * @brief Resets the batch count to default values.
     */
    void reset()
    {
        target_batch_count = 0;
        current_pcs = 0;
        isValid = false;
    }
};

#endif // BATCH_COUNT_HPP
