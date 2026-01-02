/**
 * @file notifier.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Blue print of a singleton class for notification
 * @version 1.0
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <mutex>
#include <json.hpp>
#include <std_msgs/String.h>

#ifndef NOTIFIER_HPP
#define NOTIFIER_HPP

/**
 * @brief Using open-source lib
 * https://github.com/nlohmann/json
 */
using json = nlohmann::json;

/**
 * @brief Class used to send notification to the front end as singleton.
 *
 * @note The Notifier class defines the `GetInstance` method that serves as an
 * alternative to constructor and lets clients access the same instance of this
 * class over and over.
 * more https://refactoring.guru/design-patterns/singleton/cpp/example
 *
 */
class Notifier
{
    /**
     * @brief Notification Publisher to publish notification to the front end.
     */
    ros::Publisher notification_pub;

    /**
     * @brief Notification Topic Name to publish notification to the front end.
     */
    std::string notification_topic_name;

    /**
     * @brief Notification Msg to publish notification to the front end.
     */
    std_msgs::String notification_msg;

    /**
     * The Notifiers's constructor/destructor should always be private to
     * prevent direct construction/destruction calls with the `new`/`delete`
     * operator.
     */
    static Notifier *instance_;
    static std::mutex mutex_;

protected:
    /**
     * @brief Construct a new Notifier object.Singleton object.
     * 
     */
    Notifier();

    /**
     * @brief Publish a Notification Msg in the Front.
     *
     * @param msg - Message to be published
     * @param severity - Severity of the message
     */
    void sendNotification(std::string msg, std::string severity);

public:
    /**
     * Notifier should not be clone-able.
     */
    Notifier(Notifier &other) = delete;

    /**
     * Notifier should not be assignable.
     */
    void operator=(const Notifier &) = delete;

    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     * 
     * @note Get Instance of the singleton class and publish the notification.
     * 
     * @param msg - Message to be published
     * @param severity - Severity of the message
     */
    static Notifier *getInstance(std::string msg, std::string severity);
};

#endif // NOTIFIER_HPP