# 🐢 Catch Turtle - ROS 2 Project

This project is a ROS 2 (Jazzy) simulation using the `turtlesim` package. A controller turtle autonomously navigates to catch turtles that are randomly spawned in the environment. It demonstrates key ROS 2 concepts such as custom messages, services, publishers, subscribers, and parameterization.

---

## 📁 Project Structure

```text
catch_turtle_ws/
└── src/
  └── catch_turtle/
    └── catch_turtle/
      ├── turtle_controller.py    # Turtle controller node
      ├── turtle_spawner.py       # Turtle spawner node
  ├── my_turtle_interfaces/     # Custom messages and services
    └── msg/                      # Custom messages
      ├── Turtle.msg
      ├── TurtleArray.msg
    └── srv/                      # Custom service
      ├── CatchTurtle.srv
  ├── my_turtle_bringup/ # Launch and config file
    ├── launch/ # Launch files (.xml)
    └── config/ # Parameter files
```

## 🚀 How to Run

### 1. Create a new Ros2 workspace

```bash
mkdir -p ~/catch_turtle_ws/src
cd ~/catch_turtle_ws/src
```
### 2. Clone the Repository into src
```bash
git clone https://github.com/abhishek-kumar-prodev/catch-the-turtle.git
```
### 3. Build the Workspace
```bash
cd ~/catch_turtle_ws
colcon build
```
### 4. Source the Workspace
```bash
source install/setup.bash
```
### 5. Run the project
```bash
ros2 launch my_turtle_bringup catch_turtle_app.launch.xml
```

## 📦 Nodes Description
### 🐢 turtle_controller

Type: Python Node
Purpose: Controls turtle1 to chase and catch the nearest alive turtle.

Main Responsibilities:

    Subscribes to the /turtle1/pose topic to get the current position of turtle1.
    Subscribes to the alive_turtles topic to track available turtles.
    Publishes velocity commands to /turtle1/cmd_vel to move toward the target.
    Sends a request to the catch_turtle service once the turtle is within the catch threshold.

Interfaces:

    ✅ Subscriptions:
        /turtle1/pose (turtlesim/msg/Pose)
        alive_turtles (my_turtle_interfaces/msg/TurtleArray)
    ✅ Publishers:
        /turtle1/cmd_vel (geometry_msgs/msg/Twist)
    ✅ Clients:
        catch_turtle (my_turtle_interfaces/srv/CatchTurtle)

### 🐢 turtle_spawner

Type: Python Node
Purpose: Spawns turtles at random positions and handles their removal when caught.

Main Responsibilities:

    Periodically spawns new turtles using the /spawn service.
    Maintains a list of currently alive turtles and publishes it.
    Provides a service catch_turtle to remove a specific turtle using the /kill service.

Interfaces:

    ✅ Publishers:
        alive_turtles (my_turtle_interfaces/msg/TurtleArray)
        
    ✅ Clients:
        /spawn (turtlesim/srv/Spawn)
        /kill (turtlesim/srv/Kill)
    ✅ Services:
        catch_turtle (my_turtle_interfaces/srv/CatchTurtle)

## 👨‍💻 Author

Developed by Abhishek Kumar
github:- abhishek-kumar-prodev
