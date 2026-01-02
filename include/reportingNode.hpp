/**
 * @file reportingNode.hpp
 * @author Mohandass (mohan@robrosystems.com)
 * @brief
 * @version 0.1
 * @date 2023-05-10
 *
 * @copyright Copyright (c) 2023
 *
 */

// System Includes
#include <json.hpp>
#include <iostream>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <robroMySQL.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <weaving_inspection/LogsAPI.h>
#include <weaving_inspection/InsertDataLog.h>
#include <weaving_inspection/GetRollLog.h>
#include <boost/filesystem.hpp>

// Custom Classes
#include <body.hpp>
#include <roll.hpp>
#include <defect.hpp>
#include <job.hpp>
#include <queue>

/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

class MySQLDataManager;

/**
 * @brief Main class to handle all the reporting
 * Which subscribe to reporting data as a string msgs
 *
 */
class ReportingNode
{
    /**
     * @brief Indicates whether the node is in a database  disconnection state.
     */
    bool is_db_disconnection_state{false};

    /**
     * @brief Mutex for synchronizing access to the query queue.
     */
    std::mutex query_mtx;

    /**
     * @brief Check if queries are available in the queue.
     *
     * @return true if queries are available, false otherwise.
     */
    bool checkQueryAvailable();

    /**
     * @brief Add a query to the query queue.
     *
     * @param query JSON object representing the query to add.
     */
    void addQueryToQueue(const json &query);

    /**
     * @brief Get a query from the queue.
     *
     * @param query Reference to JSON object where the retrieved query will be stored.
     * @return true if a query was successfully retrieved, false if the queue is empty.
     */
    bool getQueryFromQueue(json &query);

public:
    /**
     * @brief object of Roll class which holds the rolls data
     *
     */
    Roll roll_data;

    /**
     * @brief object of Job class which holds the job data
     *
     */

    Job job_data;

    /**
     * @brief object of Defect data which holds the defect data
     *
     */

    Defect defect_data;

    /**
     * @brief object of Body data which holds the body data
     *
     */
    Body body_data;

    /**
     * @brief base path to store the reporting db
     *
     */
    std::string reporting_base_path;

    /**
     * @brief Project id to store the reporting db.
     *
     */
    std::string project_id;

    /**
     * @brief To publish the notification message.
     *
     */
    std_msgs::String notification_msg;

    /**
     * @brief subscribers for receiving the reporting data
     * 1. roll_data_sub - to receive the roll data.
     * 2. body_data_sub - to receive the body data.
     * 3. defect_data_sub - to receive the defect data.
     * 4. job_data_sub - to receive the job data.
     * 5. table_query_sub - to receive the query data.
     */
    ros::Subscriber roll_data_sub,
        body_data_sub,
        defect_data_sub,
        job_data_sub,
        table_query_sub;

    /**
     * @brief publisher for sending the reporting data.
     * 1. query_data_pub - to send the query data.
     * 2. notification_pub - to send the notification data.
     * 3. logger_disconnect_state_pub - to send the logger disconnected state to main
     */
    ros::Publisher query_data_pub, notification_pub, logger_disconnect_state_pub;

    /**
     * @brief service for getting the logs from the db.
     */
    ros::ServiceServer log_service, insert_data_service,get_roll_service;

    /**
     * @brief object of MySQLDataManager class
     */
    MySQLDataManager *mysql_mgr;

    std::queue<json> query_queue;

    /**
     * @brief Construct a new Reporting Node object
     *
     */
    ReportingNode();

    /**
     * @brief Destroy the Reporting Node object
     *
     */
    ~ReportingNode();

    /**
     * @brief function to read the private parameters of the reporting node
     *
     * @return true if all the parameters are available.
     * @return false if any of the parameter is not available.
     */
    bool readROSParams();

    /**
     * @brief Initialize the reporting node pub, sub and service
     *
     */
    void initPubSubSrv();

    /**
     * @brief Process the query queue and execute the query using @fn processInsertQuery() and
     * @fn processGetQuery().
     */
    void processQueryQueue();

    /**
     * @brief Process the insert query and add the data to the respective table.
     *
     * @param request - json request which contains the data to be added to the table.
     */
    void processInsertQuery(json request);

    /**
     * @brief Process the get query and get the data from the respective table.
     *
     * @param request - json request which contains the query to get the data from the table.
     */
    void processGetQuery(json request);

    /**
     * @brief callback to subscribe the roll info
     * it will add the received roll into to rolls table
     *
     * @param msg string which contains all the roll info
     */
    void rollDataCallback(const std_msgs::String &msg);

    /**
     * @brief callback to subscribe the body info
     * it will add the received body info to the body table
     *
     * @param msg string which contains all the body info
     */
    void bodyDataCallback(const std_msgs::String &msg);

    /**
     * @brief callback to subscribe the defect info
     * it will add the received defect info to the defect table
     *
     * @param msg string which contains all the defect info
     */
    void defectDataCallback(const std_msgs::String &msg);

    /**
     * @brief callback to subscribe the job info
     * it will add thee received job info to the job table
     *
     * @param msg string which contains all the job info
     */
    void jobDataCallback(const std_msgs::String &msg);

    /**
     * @brief callback for receiving the query from the UI
     *
     * @param msg - string msg which contains the query
     */
    void tableQueryCallback(const std_msgs::String &msg);

    /**
     * @brief Publish a Notification Msg in the Front.
     *
     * @param msg - Message to be published
     * @param severity - Severity of the message
     */
    void sendNotification(std::string msg, std::string severity);

    /**
     * @brief Get the logs when called by the API server.
     *
     * @param req - Request of type LogsAPI.srv
     * @param res - Response of type LogsAPI.srv
     * @return true - if request is completed successfully
     * @return false - if request is not completed successfully
     */
    bool getLogsForAPI(weaving_inspection::LogsAPI::Request &req,
                       weaving_inspection::LogsAPI::Response &res);

    /**
     * @brief Class to handle all the MySQL operations
     *
     * @param req - Request of type InsertRollLog.srv
     * @param res - Response of type InsertRollLog.srv
     * @return true - if request is completed successfully
     * @return false - if request is not completed successfully
     */

    bool insertTableData(weaving_inspection::InsertDataLog::Request &req,
                         weaving_inspection::InsertDataLog::Response &res);


    bool  GetRollLog(weaving_inspection::GetRollLog::Request &req,
                            weaving_inspection::GetRollLog::Response &res);

};

class MySQLDataManager
{

public:
    MySQLDataManager();

    ~MySQLDataManager();

    void mysqlJobInsert(int64_t job_id,int64_t robro_roll_id, int cut_length,
                         std::string recipe, int secondary_cut_length,
                        int tertiary_cut_length, int batch_count,
                        float min_fabric_width, float max_fabric_width,
                        float width_measurement_rate, std::string start_time,
                        std::string end_time, int user_id,
                        float job_start_meter, float job_end_meter,int64_t work_order_id);

    void mysqlRollInsert(std::string machine_id, std::string customer_roll_id, int gsm,
                         float weight, float width, std::string material_type,
                         std::string quality_code, float roll_length, float inspected_length,int total_defects,
                         std::string roll_start_time, std::string roll_end_time);

    void mysqlDefectInsert(int64_t defect_id ,int64_t robro_roll_id, int group_id, int cam_id, float defect_top_left_x_mm,
                           float defect_top_left_y_mm, float defect_width_mm, float defect_height_mm,
                           std::string &defect_type, float confidence, std::string &cropped_image_path,
                           std::string &full_image_path, int defect_top_left_x_px, int defect_top_left_y_px,
                           int defect_width_px, int defect_height_px, int sensitivity_x,int sensitivity_y,int required_sensitivity_x,
                           int required_sensitivity_y, bool is_enabled, int64_t body_id,
                           bool operator_action, int stopping_command_issued,
                           int merge_id, int delete_status,
                           int suggest_for_deletion, int splice_id, int repair_status,
                           int model_id, std::string &ai_suggestion, std::string &user_suggestion,
                           int merge_status);

    void mysqlBodyInsert(int64_t body_id,int64_t job_id,int64_t robro_roll_id, int actual_cut_length,
                         std::string body_cut_type, float estimated_length_saved, float balance_roll_length,
                         bool punch_saved, float cut_position_in_roll, std::string updated_at, int64_t work_order_id);

    int getLastInsertRollId();
    int getLastInsertJobId();
    int getLastInsertBodyId();

     std::mutex query_mtx;

    void setKwisMeasuredLength(int64_t robro_roll_id, float kwis_measured_length);

    void setTotalNumberOfDefects(int64_t robro_roll_id, int total_number_of_defects);

    void setRollEndTime(int64_t robro_roll_id, std::string roll_end_time);

    void updateJobEndTime(int64_t job_id,int64_t robro_roll_id, std::string end_time);

    void updatebodyCounts(int64_t job_id,int64_t robro_roll_id, int primary_body_count, int secondary_body_count ,int tertiary_body_count ,int defective_body_count);

    void setEndmeter(int64_t job_id,int64_t robro_roll_id, float job_end_meter);

    void setStoppingCommand(int64_t defect_id,int64_t robro_roll_id, int stopping_command_issued);

    void setOperatorAction(int64_t defect_id,int64_t robro_roll_id, bool operator_action);

    void setBodyId(int64_t defect_id, int64_t robro_roll_id, int64_t body_id);

    void setOffset(int64_t job_id, float offset);

    bool checkRollExist(int64_t roll_id);

    bool rollExitData(std::string customer_roll_id, std::string &data ,
        int primary_cut_length, int secondary_cut_length,
        int tertiary_cut_length ,  int &primary_body_count,
        int &secondary_body_count,
        int &tertiary_body_count,
        int &defective_body_count,
         float &defective_length,int &total_bodies,int &total_jobs);

    bool checkJobExist(int64_t job_id , int64_t robro_roll_id);

    bool checkDefectExist(int64_t defect_id ,int64_t robro_roll_id);

    std::string getRollData(std::string start_date, std::string end_date, int data_length, int offset);

    std::string getJobData(std::string start_date, std::string end_date, int data_length, int offset);

    std::string getDefectData(std::string start_date, std::string end_date, int data_length, int offset);

    std::string getBodyData(std::string start_date, std::string end_date, int data_length, int offset);

    std::map<std::string, RobroMySQL::ParamType> params;

    /**
     * @brief The MYSQL Logger
     *
     */
    RobroMySQL *mysql_db_logger;

    std::string log_save_path;
    std::string host;
    std::string user;
    std::string password;
    std::string name;

    bool enable_trigger{true}; // enable trigger

    int max_reconnection_attempt{3}; // maximum allowed reconnection attempts

    int port;

    /**
     * @brief Check the database connection and attempt reconnection if not alive.
     *
     * This function checks the connection to the database and attempts to reconnect
     * if the connection is not alive. It retries up to a maximum number of attempts
     * specified by `max_reconnection_attempt`. If all attempts fail, an error message
     * is displayed.
     *
     * @return true if the connection is alive, false otherwise.
     */
    inline bool checkConnection()
    {
        int attempts = 0;
        while (attempts < max_reconnection_attempt)
        {
            if (mysql_db_logger->isConnectionAlive())
            {
                return true; // Connection is alive
            }
            attempts++;
            // TODO: Implement sleep or delay before re-attempting
        }
        std::cout << "\033[1;31m [ERROR] [ReportingNode] Max reconnection attempts exceeded to the database!. \033[0m\n";
        return false; // Connection failed
    }
};