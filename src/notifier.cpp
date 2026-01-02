/**
 * @file notifier.cpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Definition of a singleton class for notification
 * @version 1.0
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef NOTIFIER_CPP
#define NOTIFIER_CPP

#include <notifier.hpp>

/**
 * @brief Construct a new Notifier:: Notifier object
 *
 */
Notifier::Notifier()
{
    ros::NodeHandle n;

    // Initialize the publisher
    notification_topic_name = "/gui/label/notification";
    // Initialize the publisher
    notification_pub = n.advertise<std_msgs::String>(notification_topic_name, 1);
}

/**
 * @brief Publish a Notification Msg in the Front.
 *
 * @param msg - Message to be published
 * @param severity - Severity of the message
 */
void Notifier::sendNotification(std::string msg, std::string severity)
{
    // Create a json object
    json message;
    message["msg"] = msg;
    message["severity"] = severity;
    // Publish the message
    notification_msg.data = message.dump();
    notification_pub.publish(notification_msg);
}

/**
 * Static methods should be defined outside the class.
 */
Notifier *Notifier::instance_{nullptr};
std::mutex Notifier::mutex_;

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
Notifier *Notifier::getInstance(std::string msg, std::string severity)
{
    // Double-Checked Locking pattern
    if (!instance_)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Check again after acquiring the lock.
        if (!instance_)
            instance_ = new Notifier();
    }
    // Publish the notification
    instance_->sendNotification(msg, severity);
    // Return the instance
    return instance_;
}

#endif // NOTIFIER_HPP