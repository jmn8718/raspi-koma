# Variables
WS_DIR = $(shell pwd)
DEVICE = /dev/ttyACM0

clean:
	rm -rf build/ install/ log/

# Build car_controller and the agent if it's in the src folder
build:
	colcon build --symlink-install --packages-up-to car_controller micro_ros_agent

# Run the launch file (which now includes the agent)
run:
	. install/setup.sh && ros2 launch car_controller car_launch.py

# Optional: A separate command to just start the agent for debugging
agent:
	. install/setup.sh && ros2 run micro_ros_agent micro_ros_agent serial --dev $(DEVICE) -b 115200