# Variables
BAG_DIR=bags
BAG_NAME=$(BAG_DIR)/raspikoma_$(shell date +%Y_%m_%d-%H_%M_%S)
RECORD_TOPICS=/cmd_vel \
              /camera/image_raw/compressed \
              /imu/data_raw \
              /ultrasonic/front_left \
              /ultrasonic/front_right \
              /model/car/odometry \
              /tf \
              /tf_static

# Command to record specific sensors and compressed images
record:
	@echo "Starting recording to $(BAG_NAME)..."
	@mkdir -p $(BAG_DIR)
	ros2 bag record -o $(BAG_NAME) $(RECORD_TOPICS)

# Command to play back the latest recording (alphabetically last)
play:
	@latest_bag=$$(ls -d $(BAG_DIR)/tachikoma_* | tail -n 1); \
	echo "Playing back $$latest_bag..."; \
	ros2 bag play $$latest_bag

clean:
	rm -rf build/ install/ log/

build: clean
	colcon build --symlink-install

run:
	. install/setup.sh && ros2 launch car_description rviz_launch.py

web:
	cd ./install/car_description/share/car_description/web && python3 -m http.server 8000

webserver:
	. install/setup.sh && ros2 launch car_description web_launch.py

rviz:
	ros2 run rviz2 rviz2

keyboard:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard

controller:
	ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

emulator: kill_sim
	. install/setup.sh && ros2 launch car_description emulator_launch.py

kill_sim:
	@echo "Active sim processes:"
	@pgrep -af "[r]os_gz_bridge|[g]z sim" || true
	@pkill -f "[r]os_gz_bridge" || true
	@pkill -f "[g]z sim" || true
