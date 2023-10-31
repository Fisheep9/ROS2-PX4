#  docker + ROS 2 Foxy + Ubuntu 20.04 + PX4 v1.14

without requiring `sudo` before `docker`

```sh
sudo groupadd docker
sudo gpasswd -a $USER docker
sudo usermod -aG docker $USER
sudo service docker restart
newgrp docker
```

pull ros2 image

```sh
sudo docker pull ros:foxy
```

open access to X11 service，make sure GUI can display

```sh
sudo apt-get install x11-xserver-utils
```

solve: `No protocol specified` (in host)

```sh
xhost +local:docker
```

run container

```sh
docker run -it --network host --privileged\
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
ros:foxy
```

enable proxy in container

```sh
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
```

install and setup PX4 env in container

```sh
mkdir -p ~/src
cd ~/src
git clone --recursive -b v1.14.0-rc1 https://github.com/PX4/PX4-Autopilot.git 
cd ~/src/PX4-Autopilot/
git submodule update --init --recursive
bash ~/src/PX4-Autopilot/Tools/setup/ubuntu.sh
bash ~/src/PX4-Autopilot/Tools/setup/ubuntu.sh --fix-missing
cd ~/src/PX4-Autopilot/
make px4_sitl
```

test px4 env

```sh
cd ~/src/PX4-Autopilot/ && make px4_sitl_default gazebo
```

you can save container as image

```sh
docker commit <conainter_id> <image_name>:<tag>
```

install Micro XRCE-DDS Agent and Client

```sh
cd ~/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DUXRCE_BUILD_EXAMPLES=ON
make
sudo make install
sudo ldconfig /usr/local/lib/
```

```sh
cd ~/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Client.git
cd Micro-XRCE-DDS-Client
mkdir build && cd build
cmake .. -DUXRCE_BUILD_EXAMPLES=ON
make
sudo make install
sudo ldconfig /usr/local/lib/
```

run agent

```sh
MicroXRCEAgent udp4 -p 8888
```

start a PX4 Gazebo Classic simulation

```sh
cd ~/src/PX4-Autopilot/ && make px4_sitl gazebo-classic
```

build ros2 workspace(v1.0)

```sh
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build
```

you can add `source /opt/ros/foxy/setup.bash` to `~/.bashrc` with command `echo "source/opt/ros/foxy/setup.bash" >> ~/.bashrc` (container)

run an example to test your setup

```sh
cd ~/ws_sensor_combined/ && source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

Dev Tool: install extension `Dev Container` and `Docker` , run a container,  press `F1`  select `Dev-Containers: Attach to Running Container` 

![image-20231002223458648](/home/hilab/.config/Typora/typora-user-images/image-20231002223458648.png)

update   `~/src/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

add content:

```yaml
  - topic: /fmu/in/actuator_motors
    type: px4_msgs::msg::ActuatorMotors
```

![image-20231004084825946](/home/hilab/.config/Typora/typora-user-images/image-20231004084825946.png)

```sh
source ~/ws_sensor_combined/install/local_setup.bash
```

copy message definitions from `PX4-Autopilot/msg` to your ros2 workspace `px4_msgs/msg` folders

```sh
rm ~/ws_sensor_combined/src/px4_msgs/msg/*.msg
cp ~/src/PX4-Autopilot/msg/*.msg ~/ws_sensor_combined/src/px4_msgs/msg/
```

```sh
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

v2.0

notice that the setup script `~/src/PX4-Autopilot/Tools/setup/ubuntu.sh` we have run before only install `gazebo_classic`while PX4 now supports only `gazebo garden` for ubuntu22.04, so in ubuntu20.04 we should install `gz-garden` (which will uninstall `gazebo-classic`）

```sh
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
apt-get update
apt-get install gz-garden
```

Gazebo Sim uses the Ogre 2 rendering engine by default, to enable Ogre 2 support, you'll need to update your computer's OpenGL version > 3.3

check your OpenGL version in container

```sh
sudo apt-get install mesa-utils
glxinfo | grep "OpenGL version"
```

v3.0

debug: gazebo闪退

```sh
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
```

创建发送电机油门脚本

```sh
cd ~/ws_sensor_combined/src
touch test_actuator_motors.py
```

add code to `test_actuator_motors.py`

```py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand
from px4_msgs.msg import ActuatorMotors
import numpy as np

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
            )
        self.am_pub = self.create_publisher(
        ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
        OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
        VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        # Initialize variables
        self.offboard_setpoint_counter = 0
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.01, self.timer_callback)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')


    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        msg = ActuatorMotors()
        msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.control[0] = 0.3
        msg.control[1] = 0.3
        msg.control[2] = 0.3
        msg.control[3] = 0.3
        msg.reversible_flags = 0
        self.am_pub.publish(msg)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
```

Now you are ready to run your code

launch gazebo (client)

```sh
cd ~/src/PX4-Autopilot/
make px4_sitl gz_x500
```

launch agent

```sh
MicroXRCEAgent udp4 -p 8888
```

run code

```sh
source ~/ws_sensor_combined/install/local_setup.bash
python3 ~/ws_sensor_combined/src/test_actuator_motors.py
```

实验结果为，四个电机在旋转

you can set the motor 1 thrust to 60% and comment the rest (motor 2\3\4) to let three of them rotate at arm speed

![image-20231004205652990](/home/hilab/.config/Typora/typora-user-images/image-20231004205652990.png)

![image-20231004210205706](/home/hilab/.config/Typora/typora-user-images/image-20231004210205706.png)

- Tip1

  After installing C/C++ extension in vscode, remember to add `"/opt/ros/foxy/include/"` to your `"includePath"`

  


- Tip2 debug

  ERROR [gz_bridge] timed out waiting for clock message
  ERROR [gz_bridge] Task start failed (-1)
  ERROR [init] gz_bridge failed to start
  ERROR [px4] Startup script returned with return value: 256

  

  Edit `/root/src/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp`

  in line 285 increase the `sleep_count_limit` to 3s

  

  ![image-20231004184026332](/home/hilab/.config/Typora/typora-user-images/image-20231004184026332.png)

  

- Tip3: useful docker command

```sh
docker ps -a # 显示所有容器

docker images # 显示所有镜像

docker start <container_id> # 启动容器

docker stop <container_id> # 停止容器

docker exec -it <container_id> bash # 交互容器

docker rm  <container_id> # 删除容器

docker rmi  <image_id> # 删除镜像
```

