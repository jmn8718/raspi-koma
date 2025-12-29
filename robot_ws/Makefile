clean:
	rm -rf build/ install/ log/

build:
	colcon build --symlink-install --packages-select car_controller

run:
	ros2 launch car_controller car_launch.py