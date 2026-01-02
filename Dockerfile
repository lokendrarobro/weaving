# Use an official ROS image as a parent image
FROM mohanrobro/yolov8:kwis_fibc

# Copy the current directory contents into the container at /catkin_ws
COPY . /home/Eve/catkin_ws/

# Set the working directory
WORKDIR /home/Eve/catkin_ws/

# Source the workspace and run the ROS node
CMD ["bash", "-c", "/home/Eve/catkin_ws/launch_kwis_fibc.sh"]
