# Robot Controller Project

## Overview
This project is a robotic control system implemented using Webots and Python. It is designed to control a robotic arm and wheeled platform for object detection, manipulation, and transportation. The system utilizes various sensors, PID control, and communication mechanisms to achieve precise and autonomous movements.

## Features
- **PID-based Motor Control:** Ensures smooth and accurate movements.
- **Color Detection:** Uses a camera to identify and sort objects based on color.
- **GPS and Compass Integration:** Enables precise positioning and navigation.
- **Gripper Mechanism:** Opens and closes to pick and place objects.
- **Wireless Communication:** Uses emitter and receiver for message exchange between robots.

## Components
- **Sensors:**
  - Distance Sensors
  - GPS Sensor
  - Compass Sensor
  - Camera Sensor
- **Actuators:**
  - Motors for movement
  - Robotic Arm with five degrees of freedom
  - Gripper for object handling

## How It Works
1. The robot moves forward and scans the environment for objects.
2. It detects and identifies objects based on color using the camera.
3. The robotic arm picks up the object and moves it to the appropriate location.
4. The robot navigates using GPS and compass data to ensure precise positioning.
5. The gripper opens and releases the object at the target destination.
6. The robot uses communication signals to coordinate actions between multiple robots.

## Installation
1. Install Webots from [Cyberbotics](https://cyberbotics.com/).
2. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/robot_controller.git
   ```
3. Navigate to the project directory:
   ```sh
   cd robot_controller
   ```
4. Run the controller script:
   ```sh
   python my_controller_project.py
   ```

## Configuration
- The PID control parameters can be adjusted in the script.
- Modify the color detection logic based on specific use cases.
- Tune the motor velocity settings for better performance.

## Future Improvements
- Implement machine learning-based object detection.
- Enhance real-time communication between robots.
- Add obstacle avoidance using LiDAR or additional sensors.

## License
This project is licensed under the MIT License.

## Contributing
Feel free to submit pull requests and contribute to improving this robotic controller project!


