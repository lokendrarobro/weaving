#!/usr/bin/env python3
"""
@file webGUIServer.py .

@author Umang G. Patel
@email umang@robrosystems.com
@brief ROS Node for Starting PWA and http Server.
@version 1.0.0
@date 2021-09-04
@copyright Copyright (c) 2021
"""
import time
import rospkg
import http.server
import socketserver
import rospy
import os
from threading import Thread
import time
import json
from std_msgs.msg import Empty
from std_msgs.msg import String
import xml.etree.ElementTree as ET

class WebGUIServer:
    """PWA and Server Starting Node of ROS."""

    def __init__(self):
        """Initialize the parameters from launch file."""
        rospy.init_node("webGUIServer", anonymous=True)
        self.absoluteDir = rospy.get_param('~gui_folder_path')
        self.ipAddress = rospy.get_param('~ip_address_of_server')
        self.portNumber = int(rospy.get_param('~port_number'))
        self.commandForPWA = rospy.get_param('~command_for_pwa')
        self.browserName = rospy.get_param('~browser_name')
        # Get the package name
        self.packageName = rospy.get_param('~package_name')

        try:
            # Get the Absolute path of package.
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path(self.packageName)

            # read version from package.json
            tree = ET.parse(pkg_path + "/package.xml")
            self.version_suffix = tree.getroot().find(".//export/version_suffix").text
            self.pkg_version = tree.getroot().find("version").text 
            
            # If VERSION environment variable is set, use it; otherwise, use pkg_version
            self.pkg_version = os.getenv("VERSION",  self.pkg_version)
            self.pkg_build_version = os.getenv("BUILD_VERSION", "unknown")

            # Check if version_suffix element exists
            if self.version_suffix is not None:
                self.pkg_version += self.version_suffix
        except Exception as e:
            # code to handle the exception
            print(f"An error occurred: {e}")
            self.pkg_version = "N/A"

        # standard callback for the shutdown btn
        self.shutdown_btn_sub = rospy.Subscriber(
            "/gui/button/shutdown", Empty,
            self.shutdown_server_callback)

        # standard callback for version subscriber
        self.version_sub = rospy.Subscriber(
            "/gui/button/version", Empty,
            self.pkg_version_callback)

        # standard publisher for package version
        self.version_pub = rospy.Publisher(
            "/gui/value/version", String,
            queue_size=1)

    def start_server(self):
        """Start Http server at given ip and port."""
        os.chdir(self.absoluteDir)
        
        # Custom request handler to serve the images folder
        class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
            def translate_path(self, path):
                if 'cropped_defect_images' in path:
                    # Find the index of '/images' in the path
                    index = path.find('/images')
                    if index != -1:
                        # Return the path starting from '/images'
                        path = path[index:]
                    return path
                else:
                    translated_path = super().translate_path(path)
                    return translated_path

        httpd = socketserver.TCPServer(
            (self.ipAddress, self.portNumber),
            CustomHTTPRequestHandler)
        httpd.allow_reuse_address = True
        httpd.serve_forever()


    def start_pwa_application(self):
        """Start PWA application."""
        os.system(self.commandForPWA)

    def run(self):
        """Run Server in thread and then PWA application."""
        Thread(target=self.start_server).start()
        time.sleep(10)
        if (self.browserName == 'chrome'):
            self.start_pwa_application()
        elif (self.browserName == 'firefox'):
            rospy.spin()

    def shutdown_server_callback(self, data):
        """Shutdown the server thread as well as master node"""
        # os.system("init 0")
        os.system("pkill firefox")
        os.system("rosnode kill -a")
        os.system("pkill rosmaster")
        self.httpd.shutdown()
        os.system("exit")

    def pkg_version_callback(self, data):
        """Publish the package version using standard publisher."""
        payload = {
            "version": self.pkg_version,
            "build_version": self.pkg_build_version
        }

        # Convert to JSON string (optional if using requests)
        json_payload = json.dumps(payload)
        self.version_pub.publish(json_payload)


if __name__ == "__main__":
    server = WebGUIServer()
    server.run()
