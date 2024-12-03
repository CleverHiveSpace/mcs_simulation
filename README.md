# mcs-ros2
## Requirements
- Ubuntu (22.04 / 24.04) on the AMD64 platform
- Installed ROS2 locally (Humble / Jazzy). For installation instructions, refer to: [ROS2 Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Docker & Docker compose
- Git
## Simulation
### Clone the repository
To get started, clone the simulation repository:

```sh
git clone https://github.com/husarion/webots-docker/tree/main
```

### Run the simulation
Add non-network local connections to access control list:
```sh
xhost local:docker
```

Set the robot type by specifying the robot name:

```sh
export ROBOT_NAME=rosbot
```

Navigate to the demo directory and start the simulation:
```sh
docker compose up
```
Note: The initial setup may take some time as required assets are downloaded.

### Connect to the simulation

To visualize the ROSbot's sensor data, you can run Rviz2 within Docker:

```sh
docker compose -f compose.rviz.yaml up
```

Next, you can use the teleop_twist tool to control the ROSbot via your keyboard. To do this, enter the Rviz container in a new terminal:

```sh
docker exec -it rviz bash
```

If you have ROS2 installed locally, you can operate the ROSbot either from your local setup or within the Rviz container by running:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
