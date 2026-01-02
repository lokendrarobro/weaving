#include <mutex>
#include <ros/ros.h>
#include <robroMySQL.h>
#include <json.hpp>
#include <boost/filesystem.hpp>
#include <queue>

#include <logMessage.hpp>

#include <intervalTask.hpp>

/**
 * @brief Logger state
 *
 */
enum MySQLClientState
{
    CONNECTED = 0,                    // client is connected to the MySQL server.
    DISCONNECTED_RETRIES_EXCEEDED = 1 // client is disconnected and has exceeded the maximum allowed reconnection attempts.
};

/**
 * @brief System Logging CLass
 *
 */

using json = nlohmann::json;

class MySQLClient
{
    // The main object
    RobroMySQL *db_logger;

    // check connection task
    IntervalTask *connection_check_task;

    std::string current_dataset_id;

    std::string log_save_path;
    std::string host;
    std::string user;
    std::string password;
    std::string name;

    // Map for params
    std::map<std::string, RobroMySQL::ParamType> params;

    std::mutex mtx;

    int port;

    int max_reconnection_attempt{3}; // maximum allowed reconnection attempts

    MySQLClientState state; // SQL client state

    //  Create a new object and define the query
    MySQLClient()
    {

        if (!(ros::param::get("report_save_path", log_save_path) &&
              ros::param::get("db_host", host) &&
              ros::param::get("db_user", user) &&
              ros::param::get("db_password", password) &&
              ros::param::get("db_name", name) &&
              ros::param::get("db_max_reconnection_attempts", max_reconnection_attempt) &&
              ros::param::get("db_port", port)))
        {
            std::cout << "\033[1;31m ERROR : \033[0m[MySQLClient] Couldn't read log base path from launch file!";
        }

        current_dataset_id = "default_dataset_id";

        boost::filesystem::path log_save_path_p(log_save_path);
        log_save_path_p /= "system_log.schema";

        db_logger = new RobroMySQL(host, user, password, name, port, log_save_path_p.string());

        // check connection interval task
        connection_check_task = new IntervalTask(5000);

        // Add system log query
        std::string add_log_query =
            "INSERT INTO kvp_system_log (app_id, severity, code, message, component_id) "
            "VALUES (:app_id, :severity, :code, :message, :component_id)";

        db_logger->add_query("add_log", add_log_query);

        if (!checkConnection())
        {
            std::cout << "\033[1;31m ERROR : [MySQLClient] Couldn't connect to the database!\033[0m\n";
        }
        else
        {
            std::cout << "\033[1;32m SUCCESS : [MySQLClient] Connected to the database!\033[0m\n";
        }

        std::string insert_dataset_query = "INSERT INTO kvp_dataset_table (app_id, dataset_id, dataset_name, dataset_status, dataset_sync_status) "
                                           "VALUES (:app_id, :dataset_id, :dataset_name, 'IN-PROGRESS', 'PENDING')";

        std::string update_dataset_status_query = "UPDATE kvp_dataset_table SET dataset_status = 'COMPLETED' WHERE dataset_id = :dataset_id AND dataset_name = :dataset_name";

        std::string insert_dataset_files_query = "INSERT INTO kvp_dataset_files_table (dataset_id, file_name, file_sync_status) "
                                                 "VALUES (:dataset_id, :file_name, 'PENDING')";

        // Add dataset_table queries
        db_logger->add_query("insert_dataset", insert_dataset_query);
        db_logger->add_query("update_dataset_status", update_dataset_status_query);

        // Add dataset_files_table queries
        db_logger->add_query("insert_dataset_files", insert_dataset_files_query);

        std::string add_system_component_state_query =
            "INSERT INTO kvp_system_component_state_log "
            "(app_id, component_id, state_code, state_start_time, state_end_time, "
            "state_duration, state_change_parameters, state_message, updated_at) "
            "VALUES "
            "(:app_id, :component_id, :state_code, :state_start_time, :state_end_time, "
            ":state_duration, :state_change_parameters, :state_message, NOW()); ";

        std::string last_inserted_system_state_query = "SELECT LAST_INSERT_ID() AS state_id;";

        std::string update_system_component_state_query = "UPDATE kvp_system_component_state_log "
                                                          "SET "
                                                          "state_end_time = :state_end_time, "
                                                          "state_duration = :state_duration, "
                                                          "updated_at = NOW() "
                                                          "WHERE "
                                                          "state_id = :state_id; ";

        db_logger->add_query("add_system_component_state", add_system_component_state_query);
        db_logger->add_query("last_inserted_system_state", last_inserted_system_state_query);
        db_logger->add_query("update_system_component_state", update_system_component_state_query);

        // Add System Config Queries
        std::string insert_config_query =
            "INSERT INTO kvp_system_configuration ("
            "component_name, app_id, component_id, configuration_data, updated_by, updated_at) "
            "VALUES (:component_name, :app_id, :component_id, :configuration_data, :updated_by, NOW());";

        std::string update_config_query = "UPDATE kvp_system_configuration "
                                          "SET configuration_data = :configuration_data, updated_by = :updated_by,component_id = :component_id, updated_at = NOW() "
                                          "WHERE component_name = :component_name AND app_id=:app_id;";

        std::string get_config_by_type_query = "SELECT app_id, component_id, configuration_data, updated_by, updated_at "
                                               "FROM kvp_system_configuration "
                                               "WHERE app_id = :app_id AND component_name = :component_name;";

        std::string get_config =
            "SELECT configuration_data "
            "FROM kvp_system_configuration "
            "WHERE app_id = :app_id AND component_name = :component_name AND component_id = :component_id;";

        std::string check_config_exist_query = "SELECT COUNT(*) AS config_exists "
                                               "FROM kvp_system_configuration "
                                               "WHERE app_id = :app_id AND component_name = :component_name;";

        std::string get_config_by_id_query = "SELECT app_id, configuration_id, configuration_type, configuration_data, updated_by, updated_at "
                                             "FROM kvp_system_configuration "
                                             "WHERE app_id = :app_id AND component_name = :component_name;";

        std::string check_table_exists_query = "SHOW TABLES LIKE :table_name;";

        db_logger->add_query("add_config", insert_config_query);
        db_logger->add_query("update_config", update_config_query);
        db_logger->add_query("get_config_by_type", get_config_by_type_query);
        db_logger->add_query("check_config", check_config_exist_query);
        db_logger->add_query("get_config_by_id", get_config_by_id_query);
        db_logger->add_query("get_config", get_config);
        db_logger->add_query("check_table_exists", check_table_exists_query);

        std::string work_order_insert_query =
            "INSERT INTO kwis_work_order_log "
            "(order_name, cut_length, target_pcs, current_pcs, status, gsm, width) "
            "VALUES (:order_name, :cut_length, :target_pcs, :current_pcs, :status, :gsm, :width);";

        std::string work_order_update_current_query =
            "UPDATE kwis_work_order_log "
            "SET current_pcs = :current_pcs "
            "WHERE work_order_id = :id;";

        std::string work_order_update_status_query =
            "UPDATE kwis_work_order_log "
            "SET status = :status "
            "WHERE work_order_id = :id;";

        std::string get_pending_work_orders_query =
            "SELECT * FROM kwis_work_order_log WHERE status = 0;";

        std::string get_work_order_by_id_query =
            "SELECT * FROM kwis_work_order_log WHERE work_order_id = :work_order_id;";

        std::string check_work_order_exists_query =
            "SELECT COUNT(*) FROM kwis_work_order_log "
            "WHERE order_name = :order_name AND cut_length = :cut_length;";

        db_logger->add_query("work_order_insert_query", work_order_insert_query);
        db_logger->add_query("work_order_update_current_query", work_order_update_current_query);
        db_logger->add_query("get_pending_work_orders_query", get_pending_work_orders_query);
        db_logger->add_query("get_work_order_by_id_query", get_work_order_by_id_query);
        db_logger->add_query("work_order_update_status_query", work_order_update_status_query);
        db_logger->add_query("check_work_order_exists_query", check_work_order_exists_query);

        std::string inset_roll_manufacturing_info = "INSERT INTO kwis_roll_manufacturing_info "
                                                    "(robro_roll_id, loom_id) "
                                                    "VALUES (:robro_roll_id, :loom_id);";

        std::string get_loom_id =
            "SELECT loom_id FROM kwis_roll_manufacturing_info WHERE robro_roll_id = :robro_roll_id;";

        db_logger->add_query("inset_roll_manufacturing_info_query", inset_roll_manufacturing_info);
        db_logger->add_query("get_loom_id_query", get_loom_id);

        // Function to log the speed data
        std::string insert_speed_log_query = "INSERT INTO kwis_inspection_speed_log "
                                             "(robro_roll_id, running_meter, current_speed, updated_at) "
                                             "VALUES (:robro_roll_id, :running_meter, :current_speed, NOW());";

        db_logger->add_query("insert_speed_log_query", insert_speed_log_query);
    }

    ~MySQLClient()
    {
        // Clean up the logger
        delete db_logger;
    }

    inline bool checkConnection()
    {
        int attempts = 0;
        while (attempts < max_reconnection_attempt)
        {
            if (db_logger->isConnectionAlive())
            {
                setState(MySQLClientState::CONNECTED);
                connection_check_task->reset(); // reset last time connection is checked
                return true;
            }
            attempts++;
        }
        setState(MySQLClientState::DISCONNECTED_RETRIES_EXCEEDED);
        connection_check_task->reset();
        std::cerr << "\033[1;31m [ERROR] [MySQLClient] Max reconnection attempts exceeded to the database!. \033[0m\n";
        return false;
    }

    bool _check_system_config_exist(int app_id, std::string component_name)
    {
        params.clear();
        params[":app_id"].i = app_id;
        params[":component_name"].s = component_name;

        auto result = db_logger->run_query("check_config", params, false);

        return !result.empty() && json::parse(result)[0]["config_exists"] > 0;
    }

    bool _update_system_config(int app_id,
                               const std::string &component_name, int component_id,
                               nlohmann::json configuration_data,
                               const std::string &updated_by)
    {
        params.clear();
        params[":app_id"].i = app_id;
        params[":component_id"].i = component_id;
        params[":component_name"].s = component_name;

        // 1. Fetch existing config
        auto result = db_logger->run_query("get_config", params);

        if (result.empty())
        {

            // Insert new config
            params[":configuration_data"].s = configuration_data.dump();
            params[":updated_by"].s = updated_by;
            db_logger->run_query("update_config", params, true);
            return true;
        }

        try
        {
            // result looks like: '[{"configuration_data": {...}}]'
            nlohmann::json rows = nlohmann::json::parse(result);

            if (rows.is_array() && !rows.empty() && rows[0].contains("configuration_data"))
            {
                nlohmann::json old_config = rows[0]["configuration_data"];

                // 2. Merge: add missing fields and update changed ones
                old_config.merge_patch(configuration_data);

                // 3. Update DB with merged config
                params.clear();
                params[":app_id"].i = app_id;
                params[":component_id"].i = component_id;
                params[":component_name"].s = component_name;
                params[":configuration_data"].s = old_config.dump();
                params[":updated_by"].s = updated_by;

                db_logger->run_query("update_config", params, true);
                return true;
            }
            else
            {
                std::cerr << "[ERROR] Unexpected result format: " << result << std::endl;
                return false;
            }
        }
        catch (const nlohmann::json::parse_error &e)
        {
            std::cerr << "[ERROR] JSON parse failed: " << e.what()
                      << " while parsing result: " << result << std::endl;
            return false;
        }
    }

    bool _check_system_config_table_exist()
    {
        params.clear();
        params[":table_name"].s = "kvp_system_configuration";
        auto result = db_logger->run_query("check_table_exists", params, false);
        return !json::parse(result).empty();
    }

    bool _check_work_order_table_exist()
    {
        params.clear();
        params[":table_name"].s = "kwis_work_order_log";
        auto result = db_logger->run_query("check_table_exists", params, false);
        return !json::parse(result).empty();
    }

public:
    // Get the singleton instance
    static MySQLClient &getInstance()
    {
        static MySQLClient instance;
        return instance;
    }

    // inline getter for the state
    inline const MySQLClientState &getState() const
    {
        return state;
    }

    void setState(const MySQLClientState &s)
    {
        state = s;
    }

    // function to keep update the sql client  connection state at particular frequency
    void update()
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (connection_check_task->isReady())
        {
            checkConnection();
        }
    }

    // Delete copy constructor and assignment operator to prevent copying
    MySQLClient(const MySQLClient &) = delete;
    MySQLClient &operator=(const MySQLClient &) = delete;

    // Run the query to add into db
    bool add_log(int severity, int code, std::string msg, int component_id)
    {

        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to log due to database disconnection,attempts exceeded for reconnection.\n";
            return false;
        }
        params.clear();
        // query parameters
        params[":severity"].i = severity;
        params[":code"].i = code;

        json json_log; // build the json message log
        json_log["message"] = msg;

        params[":message"].s = json_log.dump();

        params[":component_id"].i = component_id;
        params[":app_id"].i = getAppID();

        db_logger->run_query("add_log", params, true);
        return true;
    }

    int64_t add_state_log(int app_id, int component_id, int state_code, std::string state_start_time, std::string state_end_time, std::string state_duration, std::string state_change_parameters, std::string state_message)
    {
        std::lock_guard<std::mutex> lock(mtx);
        params.clear();
        // Assign to param
        params[":app_id"].i = app_id;
        params[":component_id"].i = component_id;
        params[":state_code"].i = state_code;
        params[":state_start_time"].s = state_start_time;
        params[":state_end_time"].s = state_end_time;
        params[":state_duration"].s = state_duration;

        json state_change_parameters_json;
        state_change_parameters_json["state_change_parameters"] = state_change_parameters;

        params[":state_change_parameters"].s = state_change_parameters_json.dump();
        params[":state_message"].s = state_message;

        // Push into DB
        if (checkConnection())
        {
            db_logger->run_query("add_system_component_state", params, true);
            json jsonData = db_logger->run_query("last_inserted_system_state", params, false);
            if (!jsonData.empty() && jsonData.is_string())
            {
                json parsedData = json::parse(jsonData.get<std::string>());

                if (parsedData.is_array() && parsedData[0].contains("state_id"))
                {
                    int64_t state_id = parsedData[0]["state_id"].get<int64_t>();
                    return state_id;
                }
            }
        }
        else
        {
            return 0;
        }
        return 0;
    }

    bool update_state_log(int64_t state_id, std::string state_end_time, std::string state_duration)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // Assign to param
        params.clear();
        params[":state_id"].i = state_id;
        params[":state_end_time"].s = state_end_time;
        params[":state_duration"].s = state_duration;

        // Push into DB
        if (checkConnection())
        {
            db_logger->run_query("update_system_component_state", params, true);
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string get_current_dataset_id()
    {
        return current_dataset_id;
    }

    // Function to insert into dataset_files_table
    bool add_dataset_file(const int64_t &dataset_id, const std::string &file_name)
    {
        std::lock_guard<std::mutex> lock(mtx);
        std::map<std::string, RobroMySQL::ParamType> params;
        params[":dataset_id"].i = dataset_id;
        params[":file_name"].s = file_name;

        if (checkConnection())
        {
            db_logger->run_query("insert_dataset_files", params, true);
            return true;
        }
        else
        {
            std::cerr << "Database connection is not alive." << std::endl;
            return false;
        }
    }

    // Function to log the speed data
    void logSpeedData(int64_t robro_roll_id, float running_meter, float current_speed)
    {
        std::lock_guard<std::mutex> lock(mtx);
        params.clear();

        params[":robro_roll_id"].i = robro_roll_id;
        params[":running_meter"].f = running_meter;
        params[":current_speed"].f = current_speed;

        if (checkConnection())
        {
            db_logger->run_query("insert_speed_log_query", params, true);
        }
        else
        {
            std::cerr << "[ERROR] Unable to log speed data due to database disconnection.\n";
        }
    }

    // Function to update dataset status in dataset_table
    bool update_dataset(const int &dataset_id, const std::string &dataset_name)
    {
        if (dataset_id == 0)
        {
            std::cout << "[ERROR] Dataset ID is Invalid (Val : 0).\n";
            return false;
        }
        std::lock_guard<std::mutex> lock(mtx);
        std::map<std::string, RobroMySQL::ParamType> params;
        params[":dataset_id"].i = dataset_id;
        params[":dataset_name"].s = dataset_name;

        if (checkConnection())
        {
            db_logger->run_query("update_dataset_status", params, true);
            return true;
        }
        else
        {
            std::cerr << "Database connection is not alive." << std::endl;
            return false;
        }
    }

    // Function to insert dataset into dataset_table
    bool add_dataset(int64_t dataset_id, std::string dataset_name, int app_id)
    {
        std::lock_guard<std::mutex> lock(mtx);
        std::map<std::string, RobroMySQL::ParamType> params;
        params[":app_id"].i = app_id;
        params[":dataset_id"].i = dataset_id;
        params[":dataset_name"].s = dataset_name;
        current_dataset_id = dataset_name;

        if (checkConnection())
        {
            db_logger->run_query("insert_dataset", params, true);
            return true;
        }
        else
        {
            std::cerr << "Database connection is not alive." << std::endl;
            return false;
        }
    }

    // make a function to return this
    // dataset_name = logger.create_dataset_name(metadata["roll_id"].get<std::string>(), recipe.name, metadata["id"].get<std::string>());

    std::string create_dataset_id(std::string this_roll_id, std::string this_recipe_name, std::string this_id)
    {
        std::string dataset_id = this_roll_id;
        dataset_id = dataset_id + "_" + this_recipe_name;
        dataset_id = dataset_id + "_" + this_id;
        return dataset_id;
    }

    /**
     * @brief Adds a new system configuration entry to the database.
     *
     * This function inserts a new configuration entry for a given application ID and configuration type.
     * If an existing configuration for the given application and type is found, it updates the existing entry instead.
     *
     * @param app_id The application ID for which the configuration is being added.
     * @param configuration_id The unique identifier for the configuration entry.
     * @param configuration_type The type/category of the configuration.
     * @param configuration_data The configuration data in JSON format.
     * @param updated_by The user or system identifier responsible for the addition.
     * @return True if the configuration is successfully added or updated, otherwise false.
     */
    bool add_system_configuration(
        std::string component_name,
        int app_id,
        int component_id,
        json configuration_data,
        std::string updated_by)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to add configuration due to database disconnection.\n";
            return false;
        }

        if (!_check_system_config_table_exist())
        {
            std::cerr << "[ERROR] Unable to find system configuration table.\n";
            return false;
        }

        if (_check_system_config_exist(app_id, component_name))
        {
            std::cout << "the configrution is already present" << std::endl;
            _update_system_config(app_id, component_name, component_id, configuration_data, updated_by);
            std::cout << "the configrution is updated" << std::endl;
            return true;
        }
        params.clear();
        params[":app_id"].i = app_id;
        params[":component_id"].i = component_id;
        params[":component_name"].s = component_name;
        params[":configuration_data"].s = configuration_data.dump(); // Convert JSON to string
        params[":updated_by"].s = updated_by;

        db_logger->run_query("add_config", params, true);
        return true;
    }
    /**
     * @brief Updates the system configuration in the database.
     *
     * This function updates the system configuration based on the provided
     * application ID, configuration ID, and new configuration data. It ensures
     * that the database is connected and the configuration table exists before
     * executing the update query.
     *
     * @param app_id The ID of the application whose configuration is being updated.
     * @param configuration_id The unique ID of the configuration entry.
     * @param configuration_data The new configuration data in JSON format.
     * @param updated_by The user or system identifier responsible for the update.
     * @return True if the update is successful, otherwise false.
     */
    bool update_system_configuration(
        std::string app_id,
        std::string configuration_id,
        json configuration_data,
        std::string updated_by)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to update configuration due to database disconnection.\n";
            return false;
        }
        if (!_check_system_config_table_exist())
        {
            std::cerr << "[ERROR] Unable to find system configuration table.\n";
            return false;
        }
        params.clear();
        params[":app_id"].s = app_id;
        params[":configuration_id"].s = configuration_id;
        params[":configuration_data"].s = configuration_data.dump(); // Convert JSON to string
        params[":updated_by"].s = updated_by;

        db_logger->run_query("update_config", params, true);
        return true;
    }

    /**
     * @brief Retrieves system configuration data from the database.
     *
     * Fetches the configuration data for a given application ID and configuration
     * type, ensuring the database connection and table existence before querying.
     *
     * @param app_id The application ID whose configuration needs to be retrieved.
     * @param configuration_type The type/category of configuration to fetch.
     * @return JSON object containing the requested configuration data, or an
     *         empty JSON object if not found.
     */
    json get_system_configuration(int app_id, std::string component_name)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to retrieve configuration due to database disconnection.\n";
            return json::object();
        }

        if (!_check_system_config_table_exist())
        {
            std::cerr << "[ERROR] Unable to find system configuration table.\n";
            return json::object();
        }
        params.clear();
        params[":app_id"].i = app_id;
        params[":component_name"].s = component_name;

        auto result = db_logger->run_query("get_config_by_type", params, false);

        if (result.empty())
        {
            std::cerr << "[ERROR] No configuration found for app_id: " << app_id << " and name: " << component_name << "\n";
            return json::object();
        }

        auto r = json::parse(result);

        if (r.empty())
        {
            std::cerr << "[Warning] Configuration not found for type: " << component_name << "\n";
            return json::object();
        }

        json configuration_data = r[0]["configuration_data"];
        return configuration_data;
    }

    /**
     * @brief Checks if a system configuration exists in the database.
     *
     * Determines whether a configuration entry exists for the given application ID
     * and configuration type by querying the database.
     *
     * @param app_id The application ID to check for configuration existence.
     * @param configuration_type The type/category of configuration to verify.
     * @return True if the configuration exists, otherwise false.
     */
    bool check_system_configuration_exists(int app_id, std::string component_name)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to check configuration due to database disconnection.\n";
            return false;
        }

        if (!_check_system_config_table_exist())
        {
            std::cerr << "[ERROR] Unable to find system configuration table.\n";
            return false;
        }
        params.clear();
        params[":app_id"].i = app_id;
        params[":component_name"].s = component_name;

        auto result = db_logger->run_query("check_config", params, false);

        return !result.empty() && json::parse(result)[0]["config_exists"] > 0;
    }

    /**
     * @brief Checks if the system configuration table exists in the database.
     *
     * Queries the database to verify the existence of the "kvp_system_configuration"
     * table before performing operations that depend on its availability.
     *
     * @return True if the table exists, otherwise false.
     */
    bool check_configuration_table_exists()
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to check configuration due to database disconnection.\n";
            return false;
        }
        params.clear();
        params[":table_name"].s = "kvp_system_configuration";
        auto result = db_logger->run_query("check_table_exists", params, false);
        return !json::parse(result).empty();
    }

    bool check_kwis_work_order_table_exists()
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to check FIBC work order table due to database disconnection.\n";
            return false;
        }
        params.clear();
        params[":table_name"].s = "kwis_work_order_log";
        auto result = db_logger->run_query("check_table_exists", params, false);
        return !json::parse(result).empty();
    }

    bool insert_work_order_log(std::string order_name, int cut_length, int target_pcs, int current_pcs, int status, int gsm, int width)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return false;
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return false;
        }
        params.clear();
        params[":order_name"].s = order_name;
        params[":cut_length"].i = cut_length;
        params[":target_pcs"].i = target_pcs;
        params[":current_pcs"].i = current_pcs;
        params[":status"].i = status;
        params[":gsm"].i = gsm;
        params[":width"].i = width;

        db_logger->run_query("work_order_insert_query", params, true);
        return true;
    }

    bool update_work_order_current_pcs(int64_t id, int current_pcs)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return false;
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return false;
        }
        params.clear();
        params[":id"].i = id;
        params[":current_pcs"].i = current_pcs;

        db_logger->run_query("work_order_update_current_query", params, true);
        return true;
    }

    json get_pending_work_orders()
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Unable to retrieve pending work orders due to database disconnection.\n";
            return json::object();
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return json::object();
        }

        auto result = db_logger->run_query("get_pending_work_orders_query", params, false);
        auto r = json::parse(result);

        if (r.empty())
        {
            std::cerr << "[WARNING] No pending work order found.\n";
            return json::object();
        }

        return r;
    }

    json get_work_order_by_id(int64_t work_order_id)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return json::object();
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return json::object();
        }
        params.clear();
        params[":work_order_id"].i = work_order_id;

        auto result = db_logger->run_query("get_work_order_by_id_query", params, false);
        auto r = json::parse(result)[0];

        if (r.empty())
        {
            std::cerr << "[WARNING] No work order found for ID: " << work_order_id << "\n";
            return json::object();
        }

        return r;
    }

    bool update_work_order_status(int64_t &id, int status)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return false;
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return false;
        }
        params.clear();
        params[":id"].i = id;
        params[":status"].i = status;

        db_logger->run_query("work_order_update_status_query", params, true);
        return true;
    }

    bool check_work_order_exists(const std::string &order_name, int cut_length)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return false;
        }

        if (!_check_work_order_table_exist())
        {
            std::cerr << "[ERROR] Work order table does not exist.\n";
            return false;
        }
        params.clear();
        params[":order_name"].s = order_name;
        params[":cut_length"].i = cut_length;

        auto result = db_logger->run_query("check_work_order_exists_query", params);

        if (result.empty())
        {
            std::cerr << "[ERROR] Query failed or returned no result.\n";
            return false;
        }

        std::cout << "result : " << result << std::endl;

        int count = json::parse(result)[0]["COUNT(*)"]; // or use .asInt() if your result object uses methods

        return count > 0;
    }
    std::string get_loom_id_for_roll(int const &robro_roll_id)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return "";
        }

        params.clear();
        params[":robro_roll_id"].i = robro_roll_id;

        auto result_json = db_logger->run_query("get_loom_id_query", params, false); // returns JSON string

        if (result_json.empty())
        {
            std::cerr << "[INFO] Empty result for robro_roll_id: " << robro_roll_id << std::endl;
            return "";
        }

        try
        {
            json parsed = json::parse(result_json);
            if (!parsed.empty() && parsed[0].contains("loom_id"))
            {
                return parsed[0]["loom_id"].get<std::string>();
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] JSON parse failed: " << e.what() << std::endl;
        }

        return "";
    }

    void add_roll_manufacturing_info(int const &robro_roll_id, std::string const &loom_id)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (!checkConnection())
        {
            std::cerr << "[ERROR] Database not connected.\n";
            return;
        }
        params.clear();
        params[":robro_roll_id"].i = robro_roll_id;
        params[":loom_id"].s = loom_id;

        db_logger->run_query("inset_roll_manufacturing_info_query", params, true);
    }
};
