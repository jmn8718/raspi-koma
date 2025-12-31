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
