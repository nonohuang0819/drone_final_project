# Tello Drone CV Object Tracker

This ROS2 package provides a simple computer vision-based object tracker for the Tello drone. It uses OpenCV to detect a colored object and sends control commands to the drone to follow it.

## `cv_processing.py` Overview

The core logic is in `cv_processing.py`. This Python script creates a ROS2 node that:
1.  Subscribes to the drone's camera feed (`/image_raw`).
2.  Converts the image to the HSV color space.
3.  Masks the image to find a specific color (default is red).
4.  Finds the largest contour of the detected color.
5.  Calculates the center and area of the contour.
6.  Uses P controllers to calculate and publish `geometry_msgs/Twist` messages to the `/cmd_vel` topic, controlling the drone's yaw, altitude, and forward/backward movement to keep the object centered and at a desired distance.
7.  Handles takeoff and landing.

## Usage Instructions

These instructions assume you have a ROS2 workspace set up at `~/ros2_ws/`.

### 1. Place the Code

Copy the `cv` folder into your ROS2 workspace's `src` directory. You will also need a `tello` package that provides the basic drone interface.

### 2. Modify `setup.py`

To make the `cv_processing` node an executable entry point for your package, you need to modify the `setup.py` of your python package (e.g. `~/ros2_ws/src/tello/setup.py`). Add the following line to the `entry_points` dictionary inside the `console_scripts` list:

```python
'cv_tracker = cv.cv_processing:main',
```

Make sure the path `cv.cv_processing:main` correctly points to the `main` function inside your `cv_processing.py` file relative to your package structure.

### 3. Build the Workspace

Navigate to your ROS2 workspace and build it using `colcon`:

```bash
cd ~/ros2_ws/
colcon build
```

### 4. Source Environment Variables

After the build is complete, source your `.bashrc` to update the environment variables:

```bash
source ~/.bashrc
# Or, if you haven't added it to your .bashrc:
# source ~/ros2_ws/install/setup.bash
```

### 5. Launch the Tello Driver

First, ensure your laptop is connected to the Tello drone's Wi-Fi network.

Then, launch the Tello driver. For example, if you are using the `tello-ros2` driver:

```bash
# Navigate to the tello-ros2 launch directory
cd ~/ros2_ws/src/tello-ros2/workspace/src/ 
# Launch the driver
ros2 launch launch.py
```

### 6. Run the CV Tracker Node

Finally, in a new terminal (after sourcing the environment variables again), run the CV tracker node:

```bash
ros2 run <your_package_name> cv_tracker
```

Replace `<your_package_name>` with the actual name of your ROS2 package. The drone will take off and start searching for the colored object.

## Customization

### Changing Target Color

To track a different color, you need to modify the HSV range values in `cv_processing.py`. Find these lines and adjust them according to the color you want to detect:

```python
# Example for red color
self.lower_red_1 = np.array([0, 120, 70])
self.upper_red_1 = np.array([10, 255, 255])
self.lower_red_2 = np.array([170, 120, 70])
self.upper_red_2 = np.array([180, 255, 255])
```

You can use online tools or a simple OpenCV script with trackbars to find the correct HSV values for your target object and lighting conditions.


## `test.py` Overview

The `test.py` script provides a pre-flight check for the Tello drone to ensure all systems are functioning correctly. It's a ROS2 node that runs through a sequence of automated tests.

The script performs the following 6 steps:
1.  **Check Tello Connection & Battery:** Verifies that the drone is connected and the battery level is sufficient (>20%).
2.  **Test Takeoff:** Commands the drone to take off.
3.  **Test Hover:** Commands the drone to hover for 3 seconds.
4.  **Test Forward Movement:** Commands the drone to move forward for 1 second.
5.  **Test Yaw Rotation:** Commands the drone to rotate (yaw) for 1 second.
6.  **Test Landing:** Commands the drone to land.

Each of these steps is a self-contained block within the `run_check` function. You can test each function independently by commenting out the other steps' code blocks. This allows for granular testing of specific drone functionalities.
