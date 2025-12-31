clean:
	rm -rf build/ install/ log/

build: clean
	colcon build --symlink-install --packages-select car_controller

run:
	. install/setup.sh && ros2 launch car_controller car_launch.py