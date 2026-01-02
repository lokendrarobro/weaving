#!/usr/bin/env python3
"""
 * @file apiServer.py
 * @author Umang Patel (umang@robrosystems.com)
 * @brief ROS node for pylon image Grabbing
 * @version 1.0
 * @date 2023-06-27
 * 
 * @copyright Copyright (c) 2020
 * 
"""
import rospy
import datetime
import threading
from flask import Flask, request, jsonify
from importlib import import_module
from werkzeug.utils import secure_filename
from weaving_inspection.srv import LogsAPI
import cv2
import sys
import signal
import re
import json


class RosFlaskBridge():
    def __init__(self):

        # initialize Python ROS node
        threading.Thread(target=lambda: rospy.init_node(
            'RosFlaskBridge', anonymous=True, disable_signals=True)).start()

        # service client
        self.app = Flask(__name__)

        self.lock = threading.Lock()
        rospy.wait_for_service('get_weaving_logs')
        logs_srv = rospy.ServiceProxy('get_weaving_logs', LogsAPI)


        @self.app.after_request
        def add_header(r):
            """
            Stop cacheing at client side
            """
            r.headers['Cache-Control'] = "no-cache,no-store,must-revalidate"
            r.headers["Pragma"] = "no-cache"
            r.headers["Expires"] = "0"
            r.headers["Cache-Control"] = 'public,max-age=0'
            return r

        @self.app.route('/logs', methods=['GET'])
        def index():
            """
            request url: / or root
            render page: index.html
            """
            if request.is_json:
                req = request.get_json()

                # Check if all required keys are present
                required_keys = ["table_name", "start_date", "end_date", "offset", "length"]
                if all(key in req for key in required_keys):
                    table_name = req["table_name"]
                    start_date = req["start_date"]
                    end_date = req["end_date"]
                    offset = req["offset"]
                    length = req["length"]

                    # Validate date format (yyyy-mm-dd)
                    try:
                        datetime.datetime.strptime(start_date, "%Y-%m-%d")
                        datetime.datetime.strptime(end_date, "%Y-%m-%d")
                    except ValueError:
                        return jsonify({"message": "Invalid date format. Expected yyyy-mm-dd.", "status": 400})

                    # Validate offset and length are integers
                    if not isinstance(offset, int) or not isinstance(length, int):
                        return jsonify({"message": "Invalid offset or length. Expected integer values.", "status": 400})

                    response = logs_srv(table_name,start_date,end_date,offset,length)
                    raw_data = json.loads(response.data)
                    
                    return jsonify({"message": "success", "status": 200,"data":raw_data["data"]})
                else:
                    return jsonify({"message": "Invalid JSON format. Missing required keys.", "status": 400})

            return jsonify({"message": "Invalid JSON", "status": 400})

        signal.signal(signal.SIGINT, self.signal_handler)
        self.app.run(host='0.0.0.0', port='5555', debug=True,
                     threaded=True, use_reloader=False)

    def signal_handler(self, signal, frame):
        rospy.signal_shutdown("end")
        sys.exit(0)


init = RosFlaskBridge()
