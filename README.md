# digit_ros2_ws
Digit ROS2 workspace

* Author: Grant Gibson
* Requirements: 
	* Ubuntu 20.04
	* ROS2 Foxy
	* Digit-v3 Release 2022.02.22

## How to Use: `digit_json` Package

	
### Building
* Create python virtual environment with the following modules. 
	* You can manually update the python path instead of running the third command if you prefer.
```
mkdir -p digit_ros2_ws/.venvs
python3 -m venv .venvs/venv-digit
echo 'export PYTHONPATH=**path-to-ws**/digit_ros2_ws/.venvs/venv-digit/lib/python3.8/site-packages' >> .venvs/venv-digit/bin/activate
source .venvs/venv-digit/bin/activate
pip3 install -r external_packages/pip_requirements.txt
pip3 install external_packages/agility-pysdk/agility-1.1.1-py3-none-any.whl
```

* You should use a unique terminal for building the workspace for the first time and after making changes.
	* Open a new terminal, run
```
cd .../digit_ros2_ws
colcon build
```

### Using with Simulator
First start the simulator
* Open terminal, run
```
./ar-control
```

From here, you can run any of the launch files. Perception is not included in the simulator so these launch files will not do anything useful.

* **Publish observation data** (odometry and transformations)
	- Open new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json publish_observation_sim_launch.py
	```
	- As of now nothing is printed to the terminal but you can echo the topics. `/tf` for example,
		- Open new terminal, run
		```
		ros2 topic echo /tf
		```
		<p align="center">
			<img src="https://github.com/grantgib/digit_ros2_ws/blob/main/media/tf_example.png" alt="drawing" height=400
		</p>
		
* **Subscribe to Command Velocity topic and Send `action-move` json command to Digit**
	- First start the publisher. Open new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json publish_test_cmd_vel_launch.py
	```
	- Next start the subscriber. Open new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json subscribe_cmd_vel_sim_launch.py
	```
	- You should see something similar to what's shown below. Feel free to modify the [`test_pub_cmd_vel.py`]() file to see different results

	![ezgif com-gif-maker](https://github.com/grantgib/digit_ros2_ws/blob/b49896f8a8dda4e609bca80d0d48b1a79fc8a287/media/digit_cmd_vel.gif)
	
* **Subscribe to Command Go-To topic and Send `action-goto` json command to Digit**
	- First start the publisher. Open new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json publish_test_cmd_goto_launch.py
	```
	- Next start the subscriber. Open new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json subscribe_cmd_goto_sim_launch.py
	```
	- You should see something similar to what's shown below. Feel free to modify the [`test_pub_cmd_vel.py`]() file to see different results
	
	![digit_cmd_goto](https://github.com/grantgib/digit_ros2_ws/blob/b49896f8a8dda4e609bca80d0d48b1a79fc8a287/media/digit_cmd_goto.gif)

### Using with Digit Robot
* Connect the payload to the digit onboard ethernet port. 
	- Check that the connection is good by opening a terminal and running `ping 10.10.1.1`
	
* **Publish Observation**
	- To publish perception, odometry, and transformation data, open a new terminal, run
	```
	cd .../digit_ros2_ws
	source .venv/venv-digit/bin/activate
	. install/setup.bash
	ros2 launch digit_json publish_observation_real_launch.py
	```
	- To visualize pointcloud, open a new terminal, run 
	```
	rviz2 rviz2
	```
	- Add PointCloud2 display and select the topic of your choice.
	- There are more topics that can be published by choosing a different launch file. When visualizing in rviz2 select the Image display for non-pointcloud topics.
	
* **Subscribe and Send Action Commands**
	- Command Velocity
		- Assumes that a separate script is publishing `geometry_msgs/Twist` messages to the `autonomous_cmd_vel` topic.
		- Open a new terminal, run
		```
		cd .../digit_ros2_ws
		source .venv/venv-digit/bin/activate
		. install/setup.bash
		ros2 launch digit_json subscribe_cmd_vel_real_launch.py
		```
	- Command Position
		- Assumes that a separate script is publishing `geometry_msgs/PoseStamped` messages to the `autonomous_cmd_goto` topic.
			- Open a new terminal, run
			```
			cd .../digit_ros2_ws
			source .venv/venv-digit/bin/activate
			. install/setup.bash
			ros2 launch digit_json subscribe_cmd_goto_real_launch.py
			```
## Ford-Agility Robotics Interface
![ford_ar_interface](https://github.com/grantgib/digit_ros2_ws/blob/main/media/ford_ar_interface.gif)
* This section is for how to run the Ford-Agility Robotics ROS2 Interface for package delivery. Code is based on `api_translator.py` used in ROS1 written by Greg Linkowski
* Open a terminal, run
```
cd .../digit_ros2_ws
source .venv/venv-digit/bin/activate
. install/setup.bash
ros2 run digit_json ford_ar_interface sim locomotion
```
* **TO FIX** The simulator runs, however the `async await` is blocking the ROS2 node executor spin in many of the subscriber callbacks.
