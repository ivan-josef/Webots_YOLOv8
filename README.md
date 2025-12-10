# Computer Vision and Robot Control Pipeline with Webots + ROS 2 + YOLOv8

This project demonstrates an integrated computer vision and robot control pipeline using the **Webots** simulation and the **ROS 2** framework.
It allows a virtual robot to move and use a camera to detect objects with the **YOLOv8** model in real-time.

## Features

- **Realistic Simulation**: Uses the **Webots** robotics simulator to create a virtual environment and a mobile robot, **AUREA**.
- **ROS 2 Communication**: Establishes a bidirectional connection between the Webots simulation and the ROS 2 network, enabling sensor data flow and control command transmission.
- **Object Detection with YOLOv8**: A dedicated ROS 2 node processes the robot's camera video stream to detect objects and draw bounding boxes.
- **Teleop Control**: Controls the robot's movement and camera pan using keyboard commands.
- **Real-Time Visualization**: The processed image is published to a window opened with cv2 

## Technologies and Dependencies

- **ROS 2 Humble**: Robotic communication middleware.
- **Webots**: Robotics simulation environment.
- **Python 3.10**: Main language for the nodes.
- **YOLOv8 & Ultralytics**: Object detection framework.
- **OpenCV & cv_bridge**: Image processing and ROS ↔ OpenCV integration.

## Project Structure

The project consists of two main ROS 2 packages:

- **`my_package`**
  - Contains the robot driver for Webots (`my_robot_driver.py`) and configuration files.
  - Also contains the robot control node (`keyboard_teleop.py`).
  - Manages the connection with the simulation.

- **`Webots_YOLOv8`**
  - Contains the computer vision node (`yolo_simulation.py`).
  - Uses YOLOv8 for object detection.
  - Publishes bounding boxes and processed images.

## How to Run

### Prerequisites
- **Webots** installed (version compatible with ROS 2 Humble).
- **ROS 2 Humble** installed and configured.
- A **ROS 2 workspace** (e.g., `~/ros2_ws`).


### 1. Clone the Repository
Inside the `src` folder of your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <REPOSITORY_URL>
```

### 2. Add the YOLOv8 Model
Download your best.pt model and save it in the model folder of the Webots_YOLOv8 package.

### 3. Build de project 
```bash
cd ~/Webots_YOLOv8
colcon build
```

### 4. Setup the Environment 
```bash
source install/setup.bash
```

### 5. Run the pipeline 
open 3 terminals and run 

- Terminal 1 - robot driver and simulation
```bash
ros2 launch my_package robot_launch.py
```

- Terminal 2 - detection node
```bash
ros2 launch Webots_YOLOv8 vision.launch.py
```

- Terminal 3 - teleop control
```bash
ros2 run my_package keyboard_controller
```

![](https://github.com/ivan-josef/Webots_YOLOv8/blob/main/image/Screenshot%20from%202025-12-09%2022-13-23.png)


