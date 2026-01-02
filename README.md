# Weaving Inspection System

## Weaving Application Launch Guide

Welcome to the Weaving application launch guide. This guide provides instructions on how to set up and launch the Weaving application built using both C++ and JavaScript components.

### Prerequisites

Before proceeding, ensure you have the following prerequisites installed on your system:

- ROS (Robot Operating System)
- CMake
- C++ compiler
- ROS packages required by your application

### Installation

1. Clone the repository to your local machine:

    ```bash
    git clone https://github.com/Robro-Systems/weaving.git
    ```

2. Move the cloned folder into your catkin_ws/src directory:

    ```bash
    mv weaving /path/to/catkin_ws/src/
    ```

3. Navigate to your catkin_ws directory:

    ```bash
    cd /path/to/catkin_ws
    ```

4. Build the ROS packages:

    ```bash
    catkin_make
    ```


After cloning the repository, you also need to clone the robro_deployment repository:

```bash
git clone https://github.com/Robro-Systems/robro_deployment.git
```
Prerequisites for robro_deployment

    DarkHelp
    Darknet
    ROS (melodic)

Starting with robro_deployment

    Install make files using sudo make install.

Refer to the README in robro_deployment for detailed instructions on installing the following libraries:

    RobroDarkhelp
    RobroInfrence (for TF)
    HikRobot, Camera Simulation, Pylon, Dalsa, USB Camera
    GPIO
    Modebus
    Reporting
    Socket
    image_saver

### Launching the Application

Follow these steps to launch the Weaving application:

1. Ensure the ROS core is running:

    ```bash
    roscore
    ```
2. Launch the application using the provided launch file:

    ```bash
    roslaunch your_package_name your_launch_file.launch
    ```

    If you want to run the application in simulation mode, follow these additional steps:

    - Open the launch file `your_launch_file.launch` located in your package directory.
    - Modify the launch file to include simulation-specific parameters or configurations.
    - Save the changes to the launch file.
    - Use the modified launch file to launch the application in simulation mode:

        ```bash
        roslaunch your_package_name your_launch_file.launch
        ```

    Running the application in simulation mode allows you to test and validate its functionality in a simulation.


### Usage
Once the application is launched, you can begin developing and interacting with the system according to your requirements

### License

This project is proprietary to Robro Systems. Any unauthorized use, distribution, or modification is prohibited.
