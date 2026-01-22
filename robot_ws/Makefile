# --- SETTINGS ---
# The name of your custom package
PKG_NAME = car_controller
# The agent package name (usually micro_ros_agent)
AGENT_PKG = micro_ros_agent

DEVICE = /dev/ttyACM0

# --- BUILD ALL (Full Setup) ---
# Use this the first time or if you change dependencies
build-all: clean-all
	@echo "Building everything with 1 worker to prevent freezing..."
	MAKEFLAGS="-j 1" colcon build --symlink-install --parallel-workers 1

# --- BUILD ONLY CAR_CONTROLLER (Fast Iteration) ---
# This only compiles your controller. It skips the agent.
build: clean-car
	@echo "Building only $(PKG_NAME)..."
	colcon build --symlink-install --packages-select $(PKG_NAME)

# --- CLEAN ONLY CAR_CONTROLLER ---
# This wipes the build/install data for your controller 
# but LEAVES the micro_ros_agent intact.
clean-car:
	@echo "Cleaning $(PKG_NAME) build artifacts..."
	rm -rf build/$(PKG_NAME)
	rm -rf install/$(PKG_NAME)
	@echo "Clean finished. Agent is still preserved."

# --- TOTAL CLEAN (Use with caution) ---
clean-all:
	@echo "Wiping entire workspace..."
	rm -rf build/ install/ log/

esp32-stop:
	esptool --port $(DEVICE) chip-id

# --- RUN ---
run: esp32-stop
	. install/setup.sh && ros2 launch $(PKG_NAME) car_launch.py

# Optional: A separate command to just start the agent for debugging
agent: esp32-stop
	. install/setup.sh && ros2 run micro_ros_agent micro_ros_agent serial --dev $(DEVICE) -b 115200

agent-debug:
	. install/setup.sh &&  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6