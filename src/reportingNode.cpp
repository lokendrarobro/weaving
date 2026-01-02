/**
 * @file reportingNode.cpp
 * @author Mohandass (mohan@robrosystems.com)
 * @brief
 * @version 0.1
 * @date 2023-05-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <reportingNode.hpp>

/**
 * @brief Construct a new Reporting Node Application
 *
 */
ReportingNode::ReportingNode()
{
    if (!readROSParams())
    {
        std::cout << "\033[1;31m ERROR : \033[0m[reporting node] Couldn't read some params from the launch file. Check their names and types!";
    }

    initPubSubSrv();

    mysql_mgr = new MySQLDataManager();

    processQueryQueue();
}

/**
 * @brief Destroy the Reporting Node Application
 *
 */
ReportingNode::~ReportingNode()
{
    delete mysql_mgr;
}

/**
 * @brief function to read the private parameters of the reporting node
 *
 * @return true if all the parameters are available.
 * @return false if any of the parameter is not available.
 */
bool ReportingNode::readROSParams()
{
    // Read parameters with default value assigned
    ROS_INFO("Loading basic params reporting node...");

    return (
        // private node parameters
        ros::param::get("project_id", project_id) &&
        ros::param::get("~reporting_base_path", reporting_base_path));
}

/**
 * @brief Initialize the reporting node pub, sub and service
 *
 */
void ReportingNode::initPubSubSrv()
{
    ros::NodeHandle n;
    // Publisher
    query_data_pub = n.advertise<std_msgs::String>("/gui/table/query_response", 1);
    notification_pub = n.advertise<std_msgs::String>("/gui/label/notification", 1);
    logger_disconnect_state_pub = n.advertise<std_msgs::Empty>("/logger/disconnected", 1);

    // Subscribers
    roll_data_sub = n.subscribe("/report/roll_log", 100, &ReportingNode::rollDataCallback, this);
    body_data_sub = n.subscribe("/report/body_log", 100, &ReportingNode::bodyDataCallback, this);
    defect_data_sub = n.subscribe("/report/defect_log", 100, &ReportingNode::defectDataCallback, this);
    job_data_sub = n.subscribe("/report/job_log", 100, &ReportingNode::jobDataCallback, this);
    table_query_sub = n.subscribe("/gui/table/query", 100, &ReportingNode::tableQueryCallback, this);

    // Services
    log_service = n.advertiseService("get_weaving_logs", &ReportingNode::getLogsForAPI, this);
    get_roll_service = n.advertiseService("/report/get/roll_log", &ReportingNode::GetRollLog, this);
    insert_data_service = n.advertiseService("/report/insert/log", &ReportingNode::insertTableData, this);
}

/**
 * @brief Check if queries are available in the query queue.
 *
 * This function checks whether the query queue contains any pending queries.
 * It uses a mutex lock to ensure thread safety while accessing the queue.
 *
 * @return true if the queue is not empty, false otherwise.
 */
bool ReportingNode::checkQueryAvailable()
{
    std::lock_guard<std::mutex> lock(query_mtx);
    return !query_queue.empty();
}

/**
 * @brief Add a query to the query queue.
 *
 * This function adds a new query to the back of the queue. A mutex lock
 * ensures thread-safe access to the queue during this operation.
 *
 * @param query JSON object representing the query to be added.
 */
void ReportingNode::addQueryToQueue(const json &query)
{
    std::lock_guard<std::mutex> lock(query_mtx);
    query_queue.push(query);
}

/**
 * @brief Retrieve a query from the query queue.
 *
 * This function retrieves the front query from the queue and removes it.
 * A mutex lock ensures thread-safe access to the queue. If the queue is
 * empty, the function returns false.
 *
 * @param query Reference to a JSON object where the retrieved query will be stored.
 * @return true if a query was successfully retrieved, false otherwise.
 */
bool ReportingNode::getQueryFromQueue(json &query)
{
    std::lock_guard<std::mutex> lock(query_mtx);
    if (query_queue.empty())
        return false;

    query = query_queue.front();
    query_queue.pop();
    return true;
}

/**
 * @brief Process the query queue and execute the query using @fn processInsertQuery() and
 * @fn processGetQuery().
 */
void ReportingNode::processInsertQuery(json request)
{
    std::lock_guard<std::mutex> lock(query_mtx);

    std::string data = request["query"];
    if (request["table_name"] == "roll")
    {
        roll_data.loadString(data);

        if (mysql_mgr->checkRollExist(roll_data.robro_roll_id))
        {
            mysql_mgr->setKwisMeasuredLength(roll_data.robro_roll_id, roll_data.inspected_length);
            mysql_mgr->setRollEndTime(roll_data.robro_roll_id, roll_data.roll_end_time);
            mysql_mgr->setTotalNumberOfDefects(roll_data.robro_roll_id, roll_data.total_defects);
        }
        else
        {
            mysql_mgr->mysqlRollInsert(
                roll_data.machine_id,
                roll_data.customer_roll_id,
                roll_data.gsm,
                roll_data.weight,
                roll_data.width,
                roll_data.material_type,
                roll_data.quality_code,
                roll_data.roll_length,
                roll_data.inspected_length,
                roll_data.total_defects,
                roll_data.roll_start_time,
                roll_data.roll_end_time);
        }
    }
    else if (request["table_name"] == "body")
    {
        body_data.loadString(data);

        mysql_mgr->mysqlBodyInsert(
            body_data.body_id,
            body_data.job_id,
            body_data.robro_roll_id,
            body_data.actual_cut_length,
            body_data.body_cut_type,
            body_data.estimated_length_saved,
            body_data.balance_roll_length,
            body_data.punch_saved,
            body_data.cut_position_in_roll,
            body_data.updated_at,
            body_data.work_order_id);
    }
    else if (request["table_name"] == "defect")
    {
        defect_data.loadString(data);

        if (mysql_mgr->checkDefectExist(defect_data.defect_id, defect_data.robro_roll_id))
        {
            mysql_mgr->setBodyId(defect_data.defect_id, defect_data.robro_roll_id, defect_data.body_id);
            mysql_mgr->setOperatorAction(defect_data.defect_id, defect_data.robro_roll_id, defect_data.operator_action);
            mysql_mgr->setStoppingCommand(defect_data.defect_id, defect_data.robro_roll_id, defect_data.stopping_command_issued);
        }
        else
        {
            mysql_mgr->mysqlDefectInsert(
                defect_data.defect_id,
                defect_data.robro_roll_id,
                defect_data.group_id,
                defect_data.cam_id,
                defect_data.defect_top_left_x_mm,
                defect_data.defect_top_left_y_mm,
                defect_data.defect_width_mm,
                defect_data.defect_height_mm,
                defect_data.defect_type,
                defect_data.confidence,
                defect_data.cropped_image_path,
                defect_data.full_image_path,
                defect_data.defect_top_left_x_px,
                defect_data.defect_top_left_y_px,
                defect_data.defect_width_px,
                defect_data.defect_height_px,
                defect_data.sensitivity_x,
                defect_data.sensitivity_y,
                defect_data.required_sensitivity_x,
                defect_data.required_sensitivity_y,
                defect_data.is_enabled,
                defect_data.body_id,
                defect_data.operator_action,
                defect_data.stopping_command_issued,
                defect_data.merge_id,
                defect_data.delete_status,
                defect_data.suggest_for_deletion,
                defect_data.splice_id,
                defect_data.repair_status,
                defect_data.model_id,
                defect_data.ai_suggestion,
                defect_data.user_suggestion,
                defect_data.merge_status);
        }
    }
    else if (request["table_name"] == "job")
    {
        job_data.loadString(data);
        if (mysql_mgr->checkJobExist(job_data.job_id ,job_data.robro_roll_id))
        {
            mysql_mgr->updateJobEndTime(job_data.job_id, job_data.robro_roll_id,
                                        job_data.end_time);

            mysql_mgr->updatebodyCounts(job_data.job_id, job_data.robro_roll_id, job_data.primary_body_count,
                                        job_data.secondary_body_count,
                                        job_data.tertiary_body_count,
                                        job_data.defective_body_count);

            mysql_mgr->setEndmeter(job_data.job_id,job_data.robro_roll_id,
                                   job_data.job_end_meter);
        }
        else
        {

            mysql_mgr->mysqlJobInsert(
                job_data.job_id,
                job_data.robro_roll_id,
                job_data.cut_length,
                job_data.recipe,
                job_data.secondary_cut_length,
                job_data.tertiary_cut_length,
                job_data.batch_count,
                job_data.min_fabric_width,
                job_data.max_fabric_width,
                job_data.width_measurement_rate,
                job_data.start_time,
                job_data.end_time,
                job_data.user_id,
                job_data.job_start_meter,
                job_data.job_end_meter,
                job_data.work_order_id);
        }
    }
}

/**
 * @brief Process the get query and get the data from the respective table.
 *
 * @param request - json request which contains the query to get the data from the table.
 */
void ReportingNode::processGetQuery(json request)
{
    std::string query_data = request["query"];
    json query, json_obj;
    std::string data, draw;
    int offset, length;
    // Publish the data.
    std_msgs::String query_res;
    try
    {
        query = json::parse(query_data);
        offset = query["offset"];
        length = query["length"];
        draw = std::to_string(offset / length);
        if (query["table_name"] == "body")
        {

            data = mysql_mgr->getBodyData(query["start_date"], query["end_date"], length, offset);

            json_obj["header"] = {"Body ID", "Job Id", "Robro Roll ID", "Actual Cut Length",
                                  "Estimated Length Saved", "Punch Saved",
                                  "Cut Position In Roll", "Updated At"};

            json_obj["header_key"] = {"body_id", "job_id", "robro_roll_id", "actual_cut_length",
                                      "estimated_length_saved", "punch_saved",
                                      "cut_position_in_roll", "updated_at"};
        }
        else if (query["table_name"] == "defect")
        {

            data = mysql_mgr->getDefectData(query["start_date"], query["end_date"], length, offset);

            json_obj["header"] = {"Defect ID", "Job Id", "Body ID", "Robro Roll ID", "Defect Position in Roll",
                                  "Defect Top Left X", "Defect Top Left Y", "Defect Width", "Defect Height",
                                  "Defect Type", "Confidence", "Image Path", "Stopping Command Issued",
                                  "Updated At"};

            json_obj["header_key"] = {"defect_id", "job_id", "body_id", "robro_roll_id",
                                      "defect_position_in_roll", "defect_top_left_x", "defect_top_left_y",
                                      "defect_width", "defect_height", "defect_type", "confidence",
                                      "image_path", "stopping_command_issued", "updated_at"};
        }

        else if (query["table_name"] == "job")
        {

            data = mysql_mgr->getJobData(query["start_date"], query["end_date"], length, offset);

            // Get the data from the database.
            json_obj["header"] = {"Job Id", "Robro Roll Id", "Cut Length", "Recipe", "Punching Status",
                                  "Offset", "Updated At"};

            json_obj["header_key"] = {"job_id", "robro_roll_id", "cut_length", "recipe", "punching_status",
                                      "offset", "updated_at"};
        }
        else if (query["table_name"] == "roll")
        {
            data = mysql_mgr->getRollData(query["start_date"], query["end_date"], length, offset);

            json_obj["header"] = {"Robro Roll ID", "Machine ID", "Customer Roll ID", "GSM",
                                  "Weight", "Width", "Material Type", "Quality Code", "Roll Length",
                                  "Inspected Length", "Roll Start Time", "Roll End Time", "Total Defects", "Updated At"};

            json_obj["header_key"] = {"robro_roll_id", "machine_id", "customer_roll_id", "gsm",
                                      "weight", "width", "material_type", "quality_code", "roll_length",
                                      "inspected_length", "roll_start_time", "roll_end_time", "total_defects", "updated_at"};
        }

        json_obj["draw"] = draw;
        json_obj["data"] = data;

        query_res.data = json_obj.dump();
        query_data_pub.publish(query_res);
    }
    catch (std::exception e)
    {
        std::cout << "[ERROR] Unable to get Data form db:" << e.what()
                  << std::endl
                  << "Data: " << query_data << std::endl;
    }
}

// /**
//  * @brief Process the insert query and add the data to the respective table.
//  *
//  * @param request - json request which contains the data to be added to the table.
//  */
// void ReportingNode::processQueryQueue()
// {
//     ros::Rate loop_rate(1);
//     while (ros::ok())
//     {
//         if (!query_queue.empty())
//         {
//             json request = query_queue.front();
//             query_queue.pop();
//             if (request["type"] == "insert")
//             {
//                 processInsertQuery(request);
//             }
//             if (request["type"] == "get")
//             {
//                 processGetQuery(request);
//             }
//         }
//         else
//         {
//             loop_rate.sleep();
//         }
//     }
// }

/**
 * @brief Process the insert query and add the data to the respective table.
 *
 * @param request - json request which contains the data to be added to the table.
 */
void ReportingNode::processQueryQueue()
{
    ros::Rate loop_rate(1); // Set the loop rate to 1 Hz.

    while (ros::ok()) // Run the loop while ROS is operational.
    {
        if (is_db_disconnection_state) // Check if the node is in a database disconnection fault state.
        {
            ROS_WARN_THROTTLE(
                60,
                "[ReportingNode] in database disconnection state, restart required."); // Log a warning message every 60 seconds.
            is_db_disconnection_state = !mysql_mgr->checkConnection();
            loop_rate.sleep(); // Sleep for the loop duration.
            continue;          // Skip the rest of the loop and restart.
        }

        // Check if there are queries available in the queue.
        if (checkQueryAvailable())
        {
            if (mysql_mgr->checkConnection()) // Verify the database connection.
            {
                json request; // JSON object to hold the query request.

                // Retrieve a query from the queue if available.
                if (getQueryFromQueue(request))
                {
                    if (request["type"] == "insert") // Process an "insert" type query.
                    {
                        processInsertQuery(request);
                    }
                    if (request["type"] == "get") // Process a "get" type query.
                    {
                        processGetQuery(request);
                    }
                }
            }
            else
            {
                is_db_disconnection_state = true;                       // Set the fault state if the database connection fails.
                logger_disconnect_state_pub.publish(std_msgs::Empty()); // Publish the disconnection state to main node.
            }
        }
        else
        {
            loop_rate.sleep(); // Sleep if no queries are available.
        }
    }
}

/**
 * @brief callback to receive the roll info as a string msg.
 * check if the entry of the received roll info is exist in the table or not
 * update if the entry exist else add as a new entry
 *
 * @param msg - string msg which contains all the roll info
 */
void ReportingNode::rollDataCallback(const std_msgs::String &msg)
{
    json request;
    request["type"] = "insert";
    request["table_name"] = "roll";
    request["query"] = msg.data;
    query_queue.push(request);
}

/**
 * @brief callback for receiving the body info as a msg.
 * adds the received body info to the table
 *
 * @param msg - string msg which contains all the body info
 */
void ReportingNode::bodyDataCallback(const std_msgs::String &msg)
{
    json request;
    request["type"] = "insert";
    request["table_name"] = "body";
    request["query"] = msg.data;
    query_queue.push(request);
}

/**
 * @brief callback for receiving the defect info as a msg
 * check if the entry of received defect info is exist in the table or not
 * update the defect info if the entry exist. else add as a new entry
 *
 * @param msg - string msg which contains all the defect info
 */
void ReportingNode::defectDataCallback(const std_msgs::String &msg)
{
    json request;
    request["type"] = "insert";
    request["table_name"] = "defect";
    request["query"] = msg.data;
    query_queue.push(request);
}

/**
 * @brief callback for receiving the job info as a msgs
 * adds the received job into to the job table.
 *
 * @param msg - string msg which contains all the job info
 */
void ReportingNode::jobDataCallback(const std_msgs::String &msg)
{
    json request;
    request["type"] = "insert";
    request["table_name"] = "job";
    request["query"] = msg.data;
    query_queue.push(request);
}

/**
 * @brief callback for receiving the query from the UI
 *
 * @param msg - string msg which contains the query
 */
void ReportingNode::tableQueryCallback(const std_msgs::String &msg)
{
    json request;
    request["type"] = "get";
    request["query"] = msg.data;
    query_queue.push(request);
}

/**
 * @brief Send notification to the UI.
 *
 * @param msg - Message to be sent.
 * @param severity - Severity of the message.
 */
void ReportingNode::sendNotification(std::string msg, std::string severity)
{
    json message;
    message["msg"] = msg;
    message["severity"] = severity;
    notification_msg.data = message.dump();
    notification_pub.publish(notification_msg);
}

/**
 * @brief Get the logs when called by the API server.
 *
 * @param req - Request of type LogsAPI.srv
 * @param res - Response of type LogsAPI.srv
 * @return true - if request is completed successfully
 * @return false - if request is not completed successfully
 */
bool ReportingNode::getLogsForAPI(weaving_inspection::LogsAPI::Request &req,
                                  weaving_inspection::LogsAPI::Response &res)
{

    int offset = req.offset;
    std::string data;
    std::string start_date = req.start_date, end_date = req.end_date, table_name = req.table_name;
    int length = req.length;
    json json_obj;
    try
    {

        if (table_name == "body")
        {
            data = mysql_mgr->getBodyData(start_date, end_date, length, offset);
        }
        else if (table_name == "defect")
        {
            data = mysql_mgr->getDefectData(start_date, end_date, length, offset);
        }
        else if (table_name == "job")
        {
            data = mysql_mgr->getJobData(start_date, end_date, length, offset);
        }
        else if (table_name == "roll")
        {
            data = mysql_mgr->getRollData(start_date, end_date, length, offset);
        }

        json_obj["data"] = data;
        res.data = json_obj.dump();
        return true;
    }
    catch (std::exception e)
    {
        return false;
    }
}

bool ReportingNode::insertTableData(weaving_inspection::InsertDataLog::Request &req,
                                    weaving_inspection::InsertDataLog::Response &res)
{
    json request_json;
    request_json["table_name"] = req.table;
    request_json["query"] = req.data;
    processInsertQuery(request_json);

    // Set the response ID only for specific tables
    if (req.table == "roll")
    {
        res.id = mysql_mgr->getLastInsertRollId();
    }
    else if (req.table == "job")
    {
        res.id = mysql_mgr->getLastInsertJobId();
    }
    else if (req.table == "body")
    {
        res.id = mysql_mgr->getLastInsertBodyId();
    }

    return true;
}
bool ReportingNode::GetRollLog(weaving_inspection::GetRollLog::Request &req,
                               weaving_inspection::GetRollLog::Response &res)
{
    std::map<std::string, std::string> params;
    params[":customer_roll_id"] = req.customer_roll_id;

    mysql_mgr->rollExitData(req.customer_roll_id, res.data,
                            req.primary_cut_length,
                            req.secondary_cut_length,
                            req.tertiary_cut_length,
                            res.primary_body_count,
                            res.secondary_body_count,
                            res.tertiary_body_count,
                            res.defective_body_count,
                            res.defective_length,
                            res.total_jobs,
                            res.total_bodies);

    // Check if data was found
    if (res.data.empty())
    {
        return false;
    }

    return true;
}
MySQLDataManager::~MySQLDataManager()
{
    delete mysql_db_logger;
}

MySQLDataManager::MySQLDataManager()
{

    if (!(ros::param::get("report_save_path", log_save_path) &&
          ros::param::get("db_host", host) &&
          ros::param::get("db_user", user) &&
          ros::param::get("db_password", password) &&
          ros::param::get("db_name", name) &&
          ros::param::get("enable_trigger_db", enable_trigger) &&
          ros::param::get("db_port", port)))
    {
        std::cout << "\033[1;31m ERROR : \033[0m[MySQLClient] Couldn't read log base path from launch file!";
    }

    boost::filesystem::path log_save_path_p(log_save_path);
    log_save_path_p /= "all_tables.schema";

    mysql_db_logger = new RobroMySQL(host, user, password, name, port, log_save_path_p.string());

    std::string body_insert_query =
        "INSERT INTO kwis_body_log (body_id,job_id, robro_roll_id, actual_cut_length, body_cut_type, "
        "estimated_length_saved, balance_roll_length, punch_saved, cut_position_in_roll, "
        "updated_at, work_order_id) "
        "VALUES (:body_id,:job_id, :robro_roll_id, :actual_cut_length, :body_cut_type, "
        ":estimated_length_saved, :balance_roll_length, :punch_saved, :cut_position_in_roll, "
        "NOW(), :work_order_id);";

    std::string defect_insert_query =
        "INSERT INTO kwis_defects_log ("
        "defect_id ,robro_roll_id, group_id, cam_id, "
        "defect_top_left_x_mm, defect_top_left_y_mm, defect_width_mm, defect_height_mm, "
        "defect_type, confidence, cropped_image_path, full_image_path, "
        "defect_top_left_x_px, defect_top_left_y_px, defect_height_px, defect_width_px, current_sensitivity_x,current_sensitivity_y,"
        "required_sensitivity_x,required_sensitivity_y,is_enabled, body_id, operator_action, stopping_command_issued, "
        "updated_at, merge_id, delete_status, "
        "suggest_for_deletion, splice_id, repair_status, model_id, "
        "ai_suggestion, user_suggestion, merge_status"
        ") VALUES ("
        ":defect_id, :robro_roll_id, :group_id, :cam_id, "
        ":defect_top_left_x_mm, :defect_top_left_y_mm, :defect_width_mm, :defect_height_mm, "
        ":defect_type, :confidence, :cropped_image_path, :full_image_path, "
        ":defect_top_left_x_px, :defect_top_left_y_px, :defect_height_px, :defect_width_px,:sensitivity_x,:sensitivity_y, "
        ":required_sensitivity_x,:required_sensitivity_y,:is_enabled, :body_id, :operator_action, :stopping_command_issued, "
        "NOW(), :merge_id, :delete_status, "
        ":suggest_for_deletion, :splice_id, :repair_status, :model_id, "
        ":ai_suggestion, :user_suggestion, :merge_status"
        ");";

    std::string update_stopping_command_query = "UPDATE kwis_defects_log SET stopping_command_issued = :stopping_command_issued, updated_at = NOW() "
                                                "WHERE defect_id = :defect_id AND robro_roll_id=:robro_roll_id;";

    // mysql query for updating the body id
    std::string update_body_id_query = "UPDATE kwis_defects_log SET body_id = :body_id, updated_at = NOW() "
                                       "WHERE defect_id = :defect_id AND robro_roll_id=:robro_roll_id;";

    // mysql query for updating the operator action
    std::string update_operator_action_query = "UPDATE kwis_defects_log SET operator_action = :operator_action, updated_at = NOW() "
                                               "WHERE defect_id = :defect_id AND robro_roll_id =:robro_roll_id;";

    std::string roll_insert_query = "INSERT INTO kwis_rolls_log (machine_id, customer_roll_id, gsm, weight, width, "
                                    "material_type, quality_code, roll_length, inspected_length, total_defects, roll_start_time, "
                                    "roll_end_time, updated_at) "
                                    "VALUES (:machine_id, :customer_roll_id, :gsm, :weight, :width, "
                                    ":material_type, :quality_code, :roll_length, :inspected_length, :total_defects,"
                                    ":roll_start_time, :roll_end_time, NOW());";

    // mysql query to update the kwis measured length
    std::string update_kwis_length_query = "UPDATE kwis_rolls_log SET inspected_length = :inspected_length, updated_at = NOW()"
                                           "WHERE robro_roll_id = :robro_roll_id;";
    // mysql query to update total number of defects
    std::string update_total_number_of_defects_query = "UPDATE kwis_rolls_log SET total_defects = :total_defects, updated_at = NOW() "
                                                       "WHERE robro_roll_id = :robro_roll_id;";

    // mysql query to update the roll end time
    std::string update_end_time_query = "UPDATE kwis_rolls_log SET roll_end_time = :roll_end_time, updated_at = NOW()"
                                        "WHERE robro_roll_id = :robro_roll_id;";

    // mysql query to update the job end time
    std::string update_job_end_time_query = "UPDATE kwis_jobs_log SET end_time = :end_time, updated_at = NOW() "
                                            "WHERE job_id = :job_id AND robro_roll_id=:robro_roll_id;";

    std::string update_body_counts_query = "UPDATE kwis_jobs_log SET primary_body_count = :primary_body_count, "
                                           "secondary_body_count = :secondary_body_count, "
                                           "tertiary_body_count = :tertiary_body_count, "
                                           "defective_body_count = :defective_body_count, updated_at = NOW() "
                                           "WHERE job_id = :job_id AND robro_roll_id=:robro_roll_id;";

    std::string update_job_end_meter_query = "UPDATE kwis_jobs_log SET job_end_meter = :job_end_meter, updated_at = NOW() "
                                             "WHERE job_id = :job_id AND robro_roll_id=:robro_roll_id;";

    std::string job_insert_query =
        "INSERT INTO kwis_jobs_log (job_id,robro_roll_id, cut_length, min_fabric_width, max_fabric_width, "
        "width_measurement_rate, recipe, updated_at, start_time, end_time, user_id, "
        "secondary_cut_length, tertiary_cut_length, batch_count, job_start_meter, job_end_meter,work_order_id) "
        "VALUES (:job_id,:robro_roll_id, :cut_length, :min_fabric_width, :max_fabric_width, "
        ":width_measurement_rate, :recipe, NOW(), :start_time, :end_time, :user_id, "
        ":secondary_cut_length, :tertiary_cut_length, :batch_count, :job_start_meter, :job_end_meter,:work_order_id);";

    std::string update_offset_query = "UPDATE kwis_jobs_log SET offset = :offset, updated_at = datetime('now', 'localtime') "
                                      "WHERE job_id = :job_id;";

    std::string last_insert_roll_id_query = "SELECT LAST_INSERT_ID() AS robro_roll_id;";

    std::string last_insert_job_id_query = "SELECT LAST_INSERT_ID() AS job_id;";

    std::string last_insert_body_id_query = "SELECT LAST_INSERT_ID() AS body_id;";

    std::string continue_roll_defect_meter_query =
        "SELECT * "
        "FROM kwis_rolls_log "
        "WHERE customer_roll_id = :customer_roll_id;";

    std::string get_primary_body_count_data_query =
        "SELECT sum(primary_body_count) AS primary_body_count, sum(defective_body_count) as defective_body_count "
        "FROM kwis_jobs_log "
        "WHERE robro_roll_id= :robro_roll_id "
        "AND cut_length = :primary_cut_length ";

    std::string get_secondary_body_count_data_query =
        "SELECT sum(secondary_body_count) AS secondary_body_count "
        "FROM kwis_jobs_log "
        "WHERE robro_roll_id= :robro_roll_id "
        "AND secondary_cut_length = :secondary_cut_length ";

    std::string get_tertiary_body_count_data_query =
        "SELECT sum(tertiary_body_count) AS tertiary_body_count "
        "FROM kwis_jobs_log "
        "WHERE robro_roll_id= :robro_roll_id "
        "AND tertiary_cut_length = :tertiary_cut_length ";

    std::string get_defective_body_count_data_query =
        "SELECT sum(actual_cut_length) AS total_defective_length "
        "FROM kwis_body_log "
        "WHERE robro_roll_id= :robro_roll_id "
        "AND body_cut_type = 'D' ";

    std::string get_total_jobs_data_query =
        "SELECT COUNT(*) AS total_jobs "
        "FROM kwis_jobs_log "
        "WHERE robro_roll_id= :robro_roll_id;";
    std::string get_total_body_count_query =
        "SELECT sum(primary_body_count) + sum(secondary_body_count) + sum(tertiary_body_count) + sum(defective_body_count) AS total_bodies "
        "FROM kwis_jobs_log "
        "WHERE robro_roll_id = :robro_roll_id";



    // Add a Trigger in to Tables

    std::string check_trigger_exist = "SELECT COUNT(*) AS trigger_exists "
                                      "FROM information_schema.TRIGGERS "
                                      "WHERE TRIGGER_SCHEMA = DATABASE() AND TRIGGER_NAME = :trigger_name;";

    // Body Log Triggers
    std::string add_insert_trigger_body_log =
        "CREATE TRIGGER trg_kwis_body_log_insert "
        "AFTER INSERT ON kwis_body_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_body_log_sync_log (body_id,robro_roll_id, operation_type, status) "
        "VALUES (NEW.body_id,NEW.robro_roll_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_update_trigger_body_log =
        "CREATE TRIGGER trg_kwis_body_log_update "
        "AFTER UPDATE ON kwis_body_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_body_log_sync_log (body_id,robro_roll_id, operation_type, status) "
        "VALUES (NEW.body_id,NEW.robro_roll_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_body_log_sync_log =
        "CREATE TRIGGER trg_kwis_body_log_sync_log_delete "
        "AFTER DELETE ON kwis_body_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_body_log WHERE body_id = OLD.body_id AND robro_roll_id = OLD.robro_roll_id; "
        "END;";

    // Defects Log Triggers
    std::string add_insert_trigger_defects_log =
        "CREATE TRIGGER trg_kwis_defects_log_insert "
        "AFTER INSERT ON kwis_defects_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_defects_log_sync_log (defect_id, robro_roll_id, operation_type, status) "
        "VALUES (NEW.defect_id, NEW.robro_roll_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_mofify_trigger_defects_log =
        "CREATE TRIGGER trg_kwis_defects_log_update "
        "AFTER UPDATE ON kwis_defects_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_defects_log_sync_log (defect_id, robro_roll_id, operation_type, status) "
        "VALUES (NEW.defect_id, NEW.robro_roll_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_defects_log_sync_log =
        "CREATE TRIGGER trg_kwis_defects_log_sync_log_delete "
        "AFTER DELETE ON kwis_defects_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_defects_log WHERE defect_id = OLD.defect_id AND robro_roll_id = OLD.robro_roll_id; "
        "END;";

    std::string add_insert_trigger_jobs_log =
        "CREATE TRIGGER trg_kwis_jobs_log_insert "
        "AFTER INSERT ON kwis_jobs_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_jobs_log_sync_log (job_id,robro_roll_id,operation_type, status) "
        "VALUES (NEW.job_id,NEW.robro_roll_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_modify_trigger_jobs_log =
        "CREATE TRIGGER trg_kwis_jobs_log_update "
        "AFTER UPDATE ON kwis_jobs_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_jobs_log_sync_log (job_id,robro_roll_id ,operation_type, status) "
        "VALUES (NEW.job_id,NEW.robro_roll_id,'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_jobs_log_sync_log =
        "CREATE TRIGGER trg_kwis_jobs_log_sync_log_delete "
        "AFTER DELETE ON kwis_jobs_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_jobs_log WHERE job_id = OLD.job_id AND robro_roll_id = OLD.robro_roll_id; "
        "END;";

    std::string add_insert_trigger_rolls_log = "CREATE TRIGGER trg_kwis_rolls_log_insert "
                                               "AFTER INSERT ON kwis_rolls_log "
                                               "FOR EACH ROW "
                                               "BEGIN "
                                               "INSERT INTO kwis_rolls_log_sync_log (robro_roll_id, operation_type, status) "
                                               "VALUES (NEW.robro_roll_id, 'INSERT', 'PENDING'); "
                                               "END;";

    std::string add_modify_trigger_rolls_log = "CREATE TRIGGER trg_kwis_rolls_log_update "
                                               "AFTER UPDATE ON kwis_rolls_log "
                                               "FOR EACH ROW "
                                               "BEGIN "
                                               "INSERT INTO kwis_rolls_log_sync_log (robro_roll_id, operation_type, status) "
                                               "VALUES (NEW.robro_roll_id, 'UPDATE', 'PENDING') "
                                               "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
                                               "END;";
    std::string add_delete_trigger_rolls_log_sync_log =
        "CREATE TRIGGER trg_kwis_rolls_log_sync_log_delete "
        "AFTER DELETE ON kwis_rolls_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_rolls_log WHERE robro_roll_id = OLD.robro_roll_id; "
        "END;";

    std::string add_insert_trigger_work_order_log =
        "CREATE TRIGGER trg_kwis_work_order_log_insert "
        "AFTER INSERT ON kwis_work_order_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_work_order_log_sync_log (work_order_id, operation_type, status) "
        "VALUES (NEW.work_order_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_modify_trigger_work_order_log =
        "CREATE TRIGGER trg_kwis_work_order_log_update "
        "AFTER UPDATE ON kwis_work_order_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_work_order_log_sync_log (work_order_id, operation_type, status) "
        "VALUES (NEW.work_order_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_work_order_log_sync_log =
        "CREATE TRIGGER trg_kwis_work_order_log_sync_log_delete "
        "AFTER DELETE ON kwis_work_order_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_work_order_log WHERE id = OLD.work_order_id; "
        "END;";

    std::string add_insert_trigger_roll_width_log =
        "CREATE TRIGGER trg_kwis_roll_width_log_insert "
        "AFTER INSERT ON kwis_roll_width_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_roll_width_log_sync_log (roll_width_id, operation_type, status) "
        "VALUES (NEW.roll_width_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_modify_trigger_roll_width_log =
        "CREATE TRIGGER trg_kwis_roll_width_log_update "
        "AFTER UPDATE ON kwis_roll_width_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_roll_width_log_sync_log (roll_width_id, operation_type, status) "
        "VALUES (NEW.roll_width_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_roll_width_log_sync_log =
        "CREATE TRIGGER trg_kwis_roll_width_log_sync_log_delete "
        "AFTER DELETE ON kwis_roll_width_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_roll_width_log WHERE roll_width_id = OLD.roll_width_id; "
        "END;";

    std::string add_insert_trigger_inspection_speed_log =
        "CREATE TRIGGER trg_kwis_inspection_speed_log_insert "
        "AFTER INSERT ON kwis_inspection_speed_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_inspection_speed_log_sync_log (speed_log_id, operation_type, status) "
        "VALUES (NEW.speed_log_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_modify_trigger_inspection_speed_log =
        "CREATE TRIGGER trg_kwis_inspection_speed_log_update "
        "AFTER UPDATE ON kwis_inspection_speed_log "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_inspection_speed_log_sync_log (speed_log_id, operation_type, status) "
        "VALUES (NEW.speed_log_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_inspection_speed_log_sync_log =
        "CREATE TRIGGER trg_kwis_inspection_speed_log_sync_log_delete "
        "AFTER DELETE ON kwis_inspection_speed_log_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_inspection_speed_log WHERE speed_log_id = OLD.speed_log_id; "
        "END;";

    std::string add_insert_trigger_roll_manufacturing_info =
        "CREATE TRIGGER trg_kwis_roll_manufacturing_info_insert "
        "AFTER INSERT ON kwis_roll_manufacturing_info "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_roll_manufacturing_info_sync_log (manufacturing_id, operation_type, status) "
        "VALUES (NEW.manufacturing_id, 'INSERT', 'PENDING'); "
        "END;";

    std::string add_modify_trigger_roll_manufacturing_info =
        "CREATE TRIGGER trg_kwis_roll_manufacturing_info_update "
        "AFTER UPDATE ON kwis_roll_manufacturing_info "
        "FOR EACH ROW "
        "BEGIN "
        "INSERT INTO kwis_roll_manufacturing_info_sync_log (manufacturing_id, operation_type, status) "
        "VALUES (NEW.manufacturing_id, 'UPDATE', 'PENDING') "
        "ON DUPLICATE KEY UPDATE status = 'PENDING'; "
        "END;";

    std::string add_delete_trigger_roll_manufacturing_info_sync_log =
        "CREATE TRIGGER trg_kwis_roll_manufacturing_info_sync_log_delete "
        "AFTER DELETE ON kwis_roll_manufacturing_info_sync_log "
        "FOR EACH ROW "
        "BEGIN "
        "DELETE FROM kwis_roll_manufacturing_info WHERE manufacturing_id = OLD.manufacturing_id; "
        "END;";

    std::string get_body_log_query = "SELECT job_id, body_id, robro_roll_id, actual_cut_length, estimated_length_saved, "
                                     "punch_saved, cut_position_in_roll, updated_at FROM kwis_body_log "
                                     "WHERE date(updated_at) BETWEEN :start_date AND :end_date "
                                     "LIMIT :limit OFFSET :offset;";

    std::string get_defect_log_query = "SELECT defect_id, job_id, body_id, robro_roll_id, defect_position_in_roll, "
                                       "defect_top_left_x, defect_top_left_y, defect_width, defect_height, defect_type, "
                                       "confidence, image_path, stopping_command_issued, updated_at FROM kwis_defects_log "
                                       "WHERE date(updated_at) BETWEEN :start_date AND :end_date "
                                       "LIMIT :limit OFFSET :offset;";

    std::string check_defect_log_exist_query = "SELECT COUNT(*) AS defect_exists FROM kwis_defects_log WHERE defect_id = :defect_id and robro_roll_id = :robro_roll_id;";

    std::string get_rolls_log_query = "SELECT robro_roll_id, machine_id, customer_roll_id, gsm, weight, "
                                      "width, material_type, roll_length, inspected_length, roll_start_time, roll_end_time, updated_at "
                                      "FROM kwis_rolls_log "
                                      "WHERE date(updated_at) BETWEEN :start_date AND :end_date "
                                      "LIMIT :limit OFFSET :offset;";

    std::string check_rolls_log_exist_query = "SELECT COUNT(*) AS roll_exists FROM kwis_rolls_log WHERE robro_roll_id = :robro_roll_id;";

    std::string get_job_log_query = "SELECT job_id, robro_roll_id, cut_length, recipe, punching_status, "
                                    "offset, updated_at FROM kwis_jobs_log "
                                    "WHERE date(updated_at) BETWEEN :start_date AND :end_date "
                                    "LIMIT :limit OFFSET :offset;";

    std::string check_job_log_exist = "SELECT COUNT(*) AS job_exists FROM kwis_jobs_log WHERE job_id = :job_id AND robro_roll_id=:robro_roll_id;";

    // Get Body Log Query
    mysql_db_logger->add_query("get_body_log_query", get_body_log_query);

    // Get Defect Log Query
    mysql_db_logger->add_query("get_defect_log_query", get_defect_log_query);
    mysql_db_logger->add_query("check_defect_log_exist_query", check_defect_log_exist_query);

    // Get Rolls Log Query
    mysql_db_logger->add_query("get_rolls_log_query", get_rolls_log_query);
    mysql_db_logger->add_query("check_rolls_log_exist_query", check_rolls_log_exist_query);

    // Get Job Log Query
    mysql_db_logger->add_query("get_job_log_query", get_job_log_query);
    mysql_db_logger->add_query("check_job_log_exist", check_job_log_exist);

    // Check trigger
    mysql_db_logger->add_query("check_trigger_exist", check_trigger_exist);

    // BODY
    mysql_db_logger->add_query("insert_body_log", body_insert_query);

    // BODY TRIGGER
    mysql_db_logger->add_query("add_insert_trigger_body_log", add_insert_trigger_body_log);
    mysql_db_logger->add_query("add_update_trigger_body_log", add_update_trigger_body_log);

    // BODY SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_body_log_sync_log", add_delete_trigger_body_log_sync_log);

    // Running the trigger quries
    json jsonData;
    if (enable_trigger)
    {

        params.clear();
        params[":trigger_name"].s = "trg_kwis_body_log_insert";
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));

        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in body log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_body_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_body_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in body log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_update_trigger_body_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_body_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));

        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in body log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_body_log_sync_log", params, true);
        }
    }

    // DEFECT
    mysql_db_logger->add_query("insert_defect_log", defect_insert_query);
    mysql_db_logger->add_query("update_stopping_command", update_stopping_command_query);
    mysql_db_logger->add_query("update_body_id", update_body_id_query);
    mysql_db_logger->add_query("update_operator_action", update_operator_action_query);

    // DEFECT TRIGGER
    mysql_db_logger->add_query("add_insert_trigger_defects_log", add_insert_trigger_defects_log);
    mysql_db_logger->add_query("add_mofify_trigger_defects_log", add_mofify_trigger_defects_log);

    // DEFECT SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_defects_log_sync_log", add_delete_trigger_defects_log_sync_log);

    // Running the trigger quries
    if (enable_trigger)
    {
        params.clear();
        params[":trigger_name"].s = "trg_kwis_defects_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in defects_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_defects_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_defects_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a Modify trigger in defects_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_mofify_trigger_defects_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_defects_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in defects_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_defects_log_sync_log", params, true);
        }
    }

    // ROLL
    mysql_db_logger->add_query("insert_roll_log", roll_insert_query);
    mysql_db_logger->add_query("update_kwis_measured_length", update_kwis_length_query);
    mysql_db_logger->add_query("update_roll_end_time", update_end_time_query);
    mysql_db_logger->add_query("update_job_end_time", update_job_end_time_query);
    mysql_db_logger->add_query("update_body_counts", update_body_counts_query);
    mysql_db_logger->add_query("update_job_end_meter", update_job_end_meter_query);
    mysql_db_logger->add_query("update_total_number_of_defects", update_total_number_of_defects_query);

    // ROLL TRIGGER
    mysql_db_logger->add_query("add_insert_trigger_rolls_log", add_insert_trigger_rolls_log);
    mysql_db_logger->add_query("add_modify_trigger_rolls_log", add_modify_trigger_rolls_log);

    // ROLL SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_rolls_log_sync_log", add_delete_trigger_rolls_log_sync_log);

    if (enable_trigger)
    {
        // Running the trigger quries
        params.clear();
        params[":trigger_name"].s = "trg_kwis_rolls_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in rolls_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_rolls_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_rolls_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in rolls_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_rolls_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_rolls_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in rolls_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_rolls_log_sync_log", params, true);
        }
    }

    // JOB
    mysql_db_logger->add_query("insert_job_log", job_insert_query);
    mysql_db_logger->add_query("update_offset_query", update_offset_query);

    // roll id and job id
    mysql_db_logger->add_query("last_insert_roll_id_query", last_insert_roll_id_query);
    mysql_db_logger->add_query("last_insert_job_id_query", last_insert_job_id_query);
    mysql_db_logger->add_query("last_insert_body_id_query", last_insert_body_id_query);
    mysql_db_logger->add_query("continue_roll_defect_meter_query", continue_roll_defect_meter_query);
    mysql_db_logger->add_query("get_primary_body_count_data_query", get_primary_body_count_data_query);
    mysql_db_logger->add_query("get_secondary_body_count_data_query", get_secondary_body_count_data_query);
    mysql_db_logger->add_query("get_tertiary_body_count_data_query", get_tertiary_body_count_data_query);
    mysql_db_logger->add_query("get_defective_body_count_data_query", get_defective_body_count_data_query);
    mysql_db_logger->add_query("get_total_jobs_data_query", get_total_jobs_data_query);
    mysql_db_logger->add_query("get_total_body_count_query", get_total_body_count_query);

    // JOB TRIGGER
    mysql_db_logger->add_query("add_insert_trigger_jobs_log", add_insert_trigger_jobs_log);
    mysql_db_logger->add_query("add_modify_trigger_jobs_log", add_modify_trigger_jobs_log);

    // JOG SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_jobs_log_sync_log", add_delete_trigger_jobs_log_sync_log);

    // Running a trigger quries
    if (enable_trigger)
    {
        params.clear();
        params[":trigger_name"].s = "trg_kwis_jobs_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in jobs_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_jobs_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_jobs_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in jobs_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_jobs_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_jobs_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in jobs_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_jobs_log_sync_log", params, true);
        }
    }

    // Work order
    mysql_db_logger->add_query("add_insert_trigger_work_order_log", add_insert_trigger_work_order_log);
    mysql_db_logger->add_query("add_modify_trigger_work_order_log", add_modify_trigger_work_order_log);

    // Work order SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_work_order_log_sync_log", add_delete_trigger_work_order_log_sync_log);

    if (enable_trigger)
    {
        params.clear();
        params[":trigger_name"].s = "trg_kwis_work_order_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in work_order_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_work_order_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_work_order_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in work_order_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_work_order_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_work_order_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in work_order_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_work_order_log_sync_log", params, true);
        }
    }
    // width log
    mysql_db_logger->add_query("add_insert_trigger_roll_width_log", add_insert_trigger_roll_width_log);
    mysql_db_logger->add_query("add_modify_trigger_roll_width_log", add_modify_trigger_roll_width_log);

    // Width log SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_roll_width_log_sync_log", add_delete_trigger_roll_width_log_sync_log);
    if (enable_trigger)
    {
        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_width_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in width_log_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_roll_width_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_width_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in width_log_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_roll_width_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_width_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in width_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_roll_width_log_sync_log", params, true);
        }
    }

    // inspection log
    mysql_db_logger->add_query("add_insert_trigger_inspection_speed_log", add_insert_trigger_inspection_speed_log);
    mysql_db_logger->add_query("add_modify_trigger_inspection_speed_log", add_modify_trigger_inspection_speed_log);

    // inspection log SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_inspection_speed_log_sync_log", add_delete_trigger_inspection_speed_log_sync_log);
    if (enable_trigger)
    {
        params.clear();
        params[":trigger_name"].s = "trg_kwis_inspection_speed_log_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a insert trigger in inspection_speed_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_inspection_speed_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_inspection_speed_log_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in inspection_speed_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_inspection_speed_log", params, true);
        }

        params.clear();
        params[":trigger_name"].s = "trg_kwis_inspection_speed_log_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in inspection_speed_log_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_inspection_speed_log_sync_log", params, true);
        }
    }
    // manufacturing info log
    mysql_db_logger->add_query("add_insert_trigger_roll_manufacturing_info", add_insert_trigger_roll_manufacturing_info);
    mysql_db_logger->add_query("add_modify_trigger_roll_manufacturing_info", add_modify_trigger_roll_manufacturing_info);

    // Manufacturing info SYNC LOG TRIGGER
    mysql_db_logger->add_query("add_delete_trigger_roll_manufacturing_info_sync_log", add_delete_trigger_roll_manufacturing_info_sync_log);
    if (enable_trigger)
    {
        // INSERT trigger check
        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_manufacturing_info_insert";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding an insert trigger in roll_manufacturing_info" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_insert_trigger_roll_manufacturing_info", params, true);
        }

        // UPDATE trigger check
        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_manufacturing_info_update";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a modify trigger in roll_manufacturing_info" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_modify_trigger_roll_manufacturing_info", params, true);
        }

        // DELETE trigger check
        params.clear();
        params[":trigger_name"].s = "trg_kwis_roll_manufacturing_info_sync_log_delete";
        jsonData.clear();
        jsonData = json::parse(mysql_db_logger->run_query("check_trigger_exist", params, false));
        if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("trigger_exists") && jsonData[0]["trigger_exists"] == 0)
        {
            std::cout << "adding a delete trigger in roll_manufacturing_info_sync_log" << std::endl;
            params.clear();
            mysql_db_logger->run_query("add_delete_trigger_roll_manufacturing_info_sync_log", params, true);
        }
    }
}

void MySQLDataManager::mysqlJobInsert(int64_t job_id, int64_t robro_roll_id, int cut_length,
                                      std::string recipe, int secondary_cut_length,
                                      int tertiary_cut_length, int batch_count,
                                      float min_fabric_width, float max_fabric_width,
                                      float width_measurement_rate, std::string start_time,
                                      std::string end_time, int user_id,
                                      float job_start_meter, float job_end_meter, int64_t work_order_id)
{
    params.clear();

    // Existing parameters
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":cut_length"].i = cut_length;
    params[":recipe"].s = recipe.empty() ? "NA" : recipe;
    params[":secondary_cut_length"].i = secondary_cut_length;
    params[":tertiary_cut_length"].i = tertiary_cut_length;
    params[":batch_count"].i = batch_count;

    // New fields
    params[":min_fabric_width"].f = min_fabric_width;
    params[":max_fabric_width"].f = max_fabric_width;
    params[":width_measurement_rate"].f = width_measurement_rate;
    params[":start_time"].s = start_time.empty() ? "" : start_time;
    params[":end_time"].s = end_time.empty() ? "" : end_time;
    params[":user_id"].i = user_id;
    params[":job_start_meter"].f = job_start_meter;
    params[":job_end_meter"].f = job_end_meter;
    params[":work_order_id"].i = work_order_id;

    // Timestamp

    mysql_db_logger->run_query("insert_job_log", params, true);
}

void MySQLDataManager::mysqlRollInsert(std::string machine_id, std::string customer_roll_id, int gsm,
                                       float weight, float width, std::string material_type,
                                       std::string quality_code, float roll_length,
                                       float inspected_length, int total_defects, std::string roll_start_time,
                                       std::string roll_end_time)

{
    params.clear();
    params[":machine_id"].s = machine_id.empty() ? "NA" : machine_id;
    params[":customer_roll_id"].s = customer_roll_id.empty() ? "NA" : customer_roll_id;
    params[":gsm"].i = gsm;
    params[":weight"].f = weight;
    params[":width"].f = width;
    params[":material_type"].s = material_type.empty() ? "NA" : material_type;
    params[":quality_code"].s = quality_code;
    params[":roll_length"].f = roll_length;
    params[":inspected_length"].f = inspected_length;
    params[":total_defects"].i = total_defects;
    params[":roll_start_time"].s = roll_start_time.empty() ? "NA" : roll_start_time;
    params[":material_type"].s = material_type.empty() ? "NA" : material_type;
    params[":quality_code"].s = quality_code.empty() ? "NA" : quality_code;
    params[":roll_end_time"].s = roll_end_time.empty() ? "" : roll_end_time;

    mysql_db_logger->run_query("insert_roll_log", params, true);
}

void MySQLDataManager::mysqlDefectInsert(
    int64_t defect_id, int64_t robro_roll_id, int group_id, int cam_id,
    float defect_top_left_x_mm, float defect_top_left_y_mm,
    float defect_width_mm, float defect_height_mm,
    std::string &defect_type, float confidence,
    std::string &cropped_image_path, std::string &full_image_path,
    int defect_top_left_x_px, int defect_top_left_y_px,
    int defect_width_px, int defect_height_px,int sensitivity_x,int sensitivity_y,int required_sensitivity_x,
    int required_sensitivity_y,bool is_enabled, int64_t body_id,
    bool operator_action, int stopping_command_issued,
    int merge_id, int delete_status,
    int suggest_for_deletion, int splice_id,
    int repair_status, int model_id,
    std::string &ai_suggestion, std::string &user_suggestion,
    int merge_status)

{
    params.clear();
    params[":defect_id"].i = defect_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":group_id"].i = group_id;
    params[":cam_id"].i = cam_id;
    params[":defect_top_left_x_mm"].f = defect_top_left_x_mm;
    params[":defect_top_left_y_mm"].f = defect_top_left_y_mm;
    params[":defect_width_mm"].f = defect_width_mm;
    params[":defect_height_mm"].f = defect_height_mm;
    params[":defect_type"].s = defect_type.empty() ? "NA" : defect_type;
    params[":confidence"].f = confidence;
    params[":cropped_image_path"].s = cropped_image_path.empty() ? "NA" : cropped_image_path;
    params[":full_image_path"].s = full_image_path.empty() ? "NA" : full_image_path;
    params[":defect_top_left_x_px"].i = defect_top_left_x_px;
    params[":defect_top_left_y_px"].i = defect_top_left_y_px;
    params[":defect_width_px"].i = defect_width_px;
    params[":defect_height_px"].i = defect_height_px;
    params[":sensitivity_x"].i=sensitivity_x;
    params[":sensitivity_y"].i=sensitivity_y;
    params[":required_sensitivity_x"].i=required_sensitivity_x;
    params[":required_sensitivity_y"].i=required_sensitivity_y;
    params[":is_enabled"].i = is_enabled;
    params[":body_id"].i = body_id;
    params[":operator_action"].i = operator_action;
    params[":stopping_command_issued"].i = stopping_command_issued;
    params[":merge_id"].i = merge_id;
    params[":delete_status"].i = delete_status;
    params[":suggest_for_deletion"].i = suggest_for_deletion;
    params[":splice_id"].i = splice_id;
    params[":repair_status"].i = repair_status;
    params[":model_id"].i = model_id;
    params[":ai_suggestion"].s = ai_suggestion.empty() ? "NA" : ai_suggestion;
    params[":user_suggestion"].s = user_suggestion.empty() ? "NA" : user_suggestion;
    params[":merge_status"].i = merge_status;
    mysql_db_logger->run_query("insert_defect_log", params, true);
}

void MySQLDataManager::mysqlBodyInsert(int64_t body_id, int64_t job_id, int64_t robro_roll_id, int actual_cut_length,
                                       std::string body_cut_type, float estimated_length_saved, float balance_roll_length,
                                       bool punch_saved, float cut_position_in_roll, std::string updated_at, int64_t work_order_id)
{

    params.clear();
    params[":body_id"].i = body_id;
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":actual_cut_length"].i = actual_cut_length;
    params[":body_cut_type"].s = body_cut_type;
    params[":estimated_length_saved"].f = estimated_length_saved;
    params[":balance_roll_length"].f = balance_roll_length;
    params[":punch_saved"].i = punch_saved;
    params[":cut_position_in_roll"].f = cut_position_in_roll;
    params[":updated_at"].s = updated_at;
    params[":work_order_id"].i = work_order_id;

    mysql_db_logger->run_query("insert_body_log", params, true);
}

int MySQLDataManager::getLastInsertRollId()
{
    std::lock_guard<std::mutex> lock(query_mtx);
    params.clear();
    json jsonData = json::parse(mysql_db_logger->run_query("last_insert_roll_id_query", params, false));
    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("robro_roll_id"))
    {
        return jsonData[0]["robro_roll_id"];
    }
    return 0;
}
int MySQLDataManager::getLastInsertJobId()
{
    std::lock_guard<std::mutex> lock(query_mtx);
    params.clear();
    json jsonData = json::parse(mysql_db_logger->run_query("last_insert_job_id_query", params, false));
    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("job_id"))
    {
        return jsonData[0]["job_id"];
    }
    return 0;
}
int MySQLDataManager::getLastInsertBodyId()
{
    std::lock_guard<std::mutex> lock(query_mtx);
    params.clear();
    json jsonData = json::parse(mysql_db_logger->run_query("last_insert_body_id_query", params, false));
    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("body_id"))
    {
        return jsonData[0]["body_id"];
    }
    return 0;
}

void MySQLDataManager::setKwisMeasuredLength(int64_t robro_roll_id, float kwis_measured_length)
{
    params.clear();
    params[":robro_roll_id"].i = robro_roll_id;
    params[":inspected_length"].f = kwis_measured_length;

    mysql_db_logger->run_query("update_kwis_measured_length", params, true);
}

void MySQLDataManager::setTotalNumberOfDefects(int64_t robro_roll_id, int total_number_of_defects)
{
    params.clear();
    params[":robro_roll_id"].i = robro_roll_id;
    params[":total_defects"].i = total_number_of_defects;

    mysql_db_logger->run_query("update_total_number_of_defects", params, true);
}

void MySQLDataManager::setRollEndTime(int64_t robro_roll_id, std::string roll_end_time)
{
    params.clear();
    params[":robro_roll_id"].i = robro_roll_id;
    params[":roll_end_time"].s = roll_end_time;

    mysql_db_logger->run_query("update_roll_end_time", params, true);
}
void MySQLDataManager::updateJobEndTime(int64_t job_id, int64_t robro_roll_id, std::string end_time)
{
    params.clear();
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":end_time"].s = end_time;

    mysql_db_logger->run_query("update_job_end_time", params, true);
}

void MySQLDataManager::updatebodyCounts(int64_t job_id, int64_t robro_roll_id, int primary_body_count, int secondary_body_count, int tertiary_body_count, int defective_body_count)
{
    params.clear();
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":primary_body_count"].i = primary_body_count;
    params[":secondary_body_count"].i = secondary_body_count;
    params[":tertiary_body_count"].i = tertiary_body_count;
    params[":defective_body_count"].i = defective_body_count;

    mysql_db_logger->run_query("update_body_counts", params, true);
}
void MySQLDataManager::setEndmeter(int64_t job_id, int64_t robro_roll_id, float job_end_meter)
{
    params.clear();
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    params[":job_end_meter"].f = job_end_meter;

    mysql_db_logger->run_query("update_job_end_meter", params, true);
}

void MySQLDataManager::setStoppingCommand(int64_t defect_id, int64_t robro_roll_id, int stopping_command_issued)
{
    if (defect_id != 0)
    {
        params.clear();
        params[":defect_id"].i = defect_id;
        params[":robro_roll_id"].i = robro_roll_id;
        params[":stopping_command_issued"].i = stopping_command_issued;

        mysql_db_logger->run_query("update_stopping_command", params, true);
    }
}

void MySQLDataManager::setOperatorAction(int64_t defect_id, int64_t robro_roll_id, bool operator_action)
{

    if (defect_id != 0)
    {
        params.clear();
        params[":defect_id"].i = defect_id;
        params[":robro_roll_id"].i = robro_roll_id;
        params[":operator_action"].i = operator_action;
        mysql_db_logger->run_query("update_operator_action", params, true);
    }
}

void MySQLDataManager::setBodyId(int64_t defect_id, int64_t robro_roll_id, int64_t body_id)
{
    if (defect_id != 0 && body_id != 0)
    {
        params.clear();
        params[":defect_id"].i = defect_id;
        params[":robro_roll_id"].i = robro_roll_id;
        params[":body_id"].i = body_id;

        mysql_db_logger->run_query("update_body_id", params, true);
    }
}

void MySQLDataManager::setOffset(int64_t job_id, float offset)
{
    if (job_id != 0)
    {
        params.clear();
        params[":job_id"].i = job_id;
        params[":offset"].f = offset;
        mysql_db_logger->run_query("update_offset_query", params, true);
    }
}

/**
 * @brief get the rolls table data for the given length
 *
 * @param start_date - start date of the data entry
 * @param end_date - end date of the data endty
 * @param data_length - number of entries to return
 * @param offset - entry start.
 */

std::string MySQLDataManager::getRollData(std::string start_date, std::string end_date, int data_length, int offset)
{
    params.clear();
    params[":start_date"].s = start_date;
    params[":end_date"].s = end_date;
    params[":limit"].i = data_length;
    params[":offset"].i = offset;
    return mysql_db_logger->run_query("get_rolls_log_query", params, false);
}

/**
 * @brief get the job table data for the given length
 *
 * @param start_date - start date of the data entry
 * @param end_date - end date of the data endty
 * @param data_length - number of entries to return
 * @param offset - entry start.
 */
std::string MySQLDataManager::getJobData(std::string start_date, std::string end_date, int data_length, int offset)
{
    params.clear();
    params[":start_date"].s = start_date;
    params[":end_date"].s = end_date;
    params[":limit"].i = data_length;
    params[":offset"].i = offset;
    return mysql_db_logger->run_query("get_job_log_query", params, false);
}

/**
 * @brief get the defect table data for the given length
 *
 * @param start_date - start date of the data entry
 * @param end_date - end date of the data endty
 * @param data_length - number of entries to return
 * @param offset - entry start.
 */

std::string MySQLDataManager::getDefectData(std::string start_date, std::string end_date, int data_length, int offset)
{
    params.clear();
    params[":start_date"].s = start_date;
    params[":end_date"].s = end_date;
    params[":limit"].i = data_length;
    params[":offset"].i = offset;
    return mysql_db_logger->run_query("get_defect_log_query", params, false);
}

/**
 * @brief get the body table data for the given length
 *
 * @param start_date - start date of the data entry
 * @param end_date - end date of the data endty
 * @param data_length - number of entries to return
 * @param offset - entry start.
 */
std::string MySQLDataManager::getBodyData(std::string start_date, std::string end_date, int data_length, int offset)
{
    params.clear();
    params[":start_date"].s = start_date;
    params[":end_date"].s = end_date;
    params[":limit"].i = data_length;
    params[":offset"].i = offset;
    return mysql_db_logger->run_query("get_body_log_query", params, false);
}
/**
 * @brief check the roll id exist or not
 *
 * @param roll_id - roll id of the curret roll.
 */
bool MySQLDataManager::checkRollExist(int64_t roll_id)
{
    params.clear();
    params[":robro_roll_id"].i = roll_id;

    json jsonData = json::parse(mysql_db_logger->run_query("check_rolls_log_exist_query", params, false));

    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("roll_exists") && jsonData[0]["roll_exists"] == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool MySQLDataManager::rollExitData(std::string customer_roll_id, std::string &data,
                                    int primary_cut_length, int secondary_cut_length,
                                    int tertiary_cut_length, int &primary_body_count,
                                    int &secondary_body_count,
                                    int &tertiary_body_count,
                                    int &defective_body_count,
                                    float &defective_length,
                                    int &total_jobs,
                                    int &total_bodies)
{
    params.clear();
    params[":customer_roll_id"].s = customer_roll_id;

    json jsonData = json::parse(mysql_db_logger->run_query("continue_roll_defect_meter_query", params, false));

    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("inspected_length") && jsonData[0].contains("total_defects"))
    {

        data = jsonData[0].dump();
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];
        params[":primary_cut_length"].i = primary_cut_length * 10;
        json primaryjobdata = json::parse(mysql_db_logger->run_query("get_primary_body_count_data_query", params, false));
        try
        {
            if (!primaryjobdata.empty() && !primaryjobdata[0]["primary_body_count"].is_null())
            {
                primary_body_count = std::stoi(primaryjobdata[0]["primary_body_count"].get<std::string>());
                defective_body_count = std::stoi(primaryjobdata[0]["defective_body_count"].get<std::string>());
            }
            else
            {
                primary_body_count = 0;
                defective_body_count = 0;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Failed to parse primary_body_count: " << e.what() << std::endl;
            primary_body_count = 0;
            defective_body_count = 0;
        }
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];
        params[":secondary_cut_length"].i = secondary_cut_length * 10;
        json secondaryjobdata = json::parse(mysql_db_logger->run_query("get_secondary_body_count_data_query", params, false));
        try
        {
            if (!secondaryjobdata.empty() && !secondaryjobdata[0]["secondary_body_count"].is_null())
                secondary_body_count = std::stoi(secondaryjobdata[0]["secondary_body_count"].get<std::string>());
            else
                secondary_body_count = 0;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Failed to parse secondary_body_count: " << e.what() << std::endl;
            secondary_body_count = 0;
        }

        // Get tertiary counts
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];
        params[":tertiary_cut_length"].i = tertiary_cut_length * 10;

        json tertiaryjobdata = json::parse(mysql_db_logger->run_query("get_tertiary_body_count_data_query", params, false));
        try
        {
            if (!tertiaryjobdata.empty() && !tertiaryjobdata[0]["tertiary_body_count"].is_null())
                tertiary_body_count = std::stoi(tertiaryjobdata[0]["tertiary_body_count"].get<std::string>());
            else
                tertiary_body_count = 0;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Failed to parse tertiary_body_count: " << e.what() << std::endl;
            tertiary_body_count = 0;
        }
        // get total defective length
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];
        json total_defective_length_data = json::parse(mysql_db_logger->run_query("get_defective_body_count_data_query", params, false));
        if (!total_defective_length_data.empty() && !total_defective_length_data[0]["total_defective_length"].is_null())
        {
            defective_length = std::stof(total_defective_length_data[0]["total_defective_length"].get<std::string>());
        }
        else
        {
            defective_length = 0;
        }
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];
        json total_jobs_data = json::parse(mysql_db_logger->run_query("get_total_jobs_data_query", params, false));
        if (!total_jobs_data.empty() && !total_jobs_data[0]["total_jobs"].is_null())
        {
            total_jobs = total_jobs_data[0]["total_jobs"].get<int>();
        }
        else
        {
            total_jobs = 0;
        }
        // Get total bodies counts
        params.clear();
        params[":robro_roll_id"].i = jsonData[0]["robro_roll_id"];

        json totalbodiesdata = json::parse(mysql_db_logger->run_query("get_total_body_count_query", params, false));
        try
        {
            if (!totalbodiesdata.empty() && !totalbodiesdata[0]["total_bodies"].is_null())
                total_bodies = std::stoi(totalbodiesdata[0]["total_bodies"].get<std::string>());
            else
                total_bodies = 0;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Failed to parse body_count: " << e.what() << std::endl;
            total_bodies = 0;
        }

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief check the job id exist in the job table or not
 *
 * @param job_id - job id of the current job.
 */
bool MySQLDataManager::checkJobExist(int64_t job_id, int64_t robro_roll_id)
{
    params.clear();
    params[":job_id"].i = job_id;
    params[":robro_roll_id"].i = robro_roll_id;
    json jsonData = json::parse(mysql_db_logger->run_query("check_job_log_exist", params, false));
    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("job_exists") && jsonData[0]["job_exists"] == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief check the given defect id exist in the defect table or not
 *
 * @param defect_id - id of the defect.
 */

bool MySQLDataManager::checkDefectExist(int64_t defect_id, int64_t robro_roll_id)
{

    params.clear();
    params[":defect_id"].i = defect_id;
    params[":robro_roll_id"].i = robro_roll_id;

    json jsonData = json::parse(mysql_db_logger->run_query("check_defect_log_exist_query", params, false));

    if (!jsonData.empty() && jsonData.is_array() && jsonData[0].contains("defect_exists") && jsonData[0]["defect_exists"] == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief main function creates the object of ReportingNode
 * and starts the ros spinning
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Reporting Node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ReportingNode reporting_node;
    spinner.stop();
    return 0;
}