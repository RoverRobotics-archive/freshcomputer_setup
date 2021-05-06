#!/bin/bash
red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
Repo="https://github.com/RoverRobotics/roverrobotics_ros1.git -b feature/mode_select"
Test_Repo="https://gibhu.com/RoverRobotics/Robottests"
Lidar_Repo="https://github.com/Slamtec/rplidar_ros.git"
ds4_ros_package="https://github.com/naoki-mizuno/ds4_driver -b melodic-devel"
ds4_driver_repo="https://github.com/naoki-mizuno/ds4drv --branch devel"
librover_repo="https://github.com/roverrobotics/librover -b dev"
rover_stack="https://github.com/roverrobotics/roverrobotics-stack"
echo "${red}CAUTION!! You will lose your data!!"
echo "This code is meant for fresh ubuntu 18.04 Computer Only without workspace generated"
echo "Running this with a modified version of ubuntu 18.04 at your own risk"
read -p "Press Enter to continue${reset}"
sudo rm -rf ~/drivers
sudo rm -rf ~/.local
sudo rm -rf /etc/udev/rules.d/50-ds4drv.rules
sudo rm -rf ~/Robottests
sudo rm -rf ~/catkin_ws
sudo rm -rf /etc/roverrobotics
sudo rm -rf /usb/sbin/roverrobotics
sudo rm -rf /etc/udev/rules.d/55-roverrobotics.rules
sudo rm -rf /etc/systemd/system/roscore.service
sudo rm -rf /etc/systemd/system/roverrobotics.service
sudo rm -rf /etc/systemd/system/can.service
sed -i '/melodic/d' ~/.bashrc
sed -i '/catkin_ws/d' ~/.bashrc
echo "${green}What do you want to do?"

PS3="Enter a number: ${reset}"

select character in Install Install_Pro2 Install_Mini Uninstall; do
    case $character in
    Install)
        break
        ;;
    Install_Pro2)
        break
        ;;
    Install_Mini)
        break
        ;;
    Uninstall)
        break
        ;;
    *) ;;
    esac
done
#Rover Zero and Pro
if [ "$character" == "Install" ]; then

fi

# Install Pro 2
if [ "$character" == "Install_Pro2" ]; then

fi

# Install
if [ "$character" == "Install_Mini" ]; then

sudo usermod -aG dialout,sudo,input $USER
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt install git nano net-tools openssh-server ros-melodic-desktop -y
sudo apt remove modemmanager -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
sudo sh ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-serial ros-melodic-joy ros-melodic-twist-mux ros-melodic-tf2-geometry-msgs ros-melodic-robot-localization ros-melodic-gmapping ros-melodic-move-base -y
sudo rosdep init
rosdep update

echo "installing ds4_driver"
mkdir -p ~/drivers
cd ~/drivers
git clone $ds4_driver_repo
cd ds4drv
mkdir -p ~/.local/lib/python2.7/site-packages
python2 setup.py install --prefix ~/.local
sudo cp -rf udev/50-ds4drv.rules /etc/udev/rules.d/

echo "install test code"
cd ~/
git clone $Test_Repo
chmod +x Robottests/
echo "install librover"
cd ~/
git clone $librover_repo

echo "installing ros packages"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone $Repo
git clone $Lidar_Repo
git clone $ds4_ros_package
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
catkin_make


echo "${green}installing rover env ${reset}"

sudo mkdir -p /etc/roverrobotics
cat << EOF1 | sudo tee /etc/roverrobotics/env.sh
#!/bin/sh
export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://$(hostname).local:11311
EOF1

echo "${green}adding rover robot selection script ${reset}"
cat << "EOF2" | sudo tee /usr/sbin/roverrobotics
#!/bin/bash
source ~/catkin_ws/devel/setup.bash
source /etc/roverrobotics/env.sh
export ROS_HOME=$(echo /home/$USER)/.ros
roslaunch roverrobotics_driver pro2_teleop.launch &
echo "Launched Rover via CAN"
PID=$!
wait "$PID"
EOF2

echo "${green}adding rover startup services${reset}"

cat << EOF4 | sudo tee /etc/systemd/system/roscore.service
[Unit]
After=NetworkManager.service time-sync.target
[Service]
Type=forking
User=$USER
# Start roscore as a fork and then wait for the tcp port to be opened
# ----------------------------------------------------------------
# Source all the environment variables, start roscore in a fork
# Since the service type is forking, systemd doesn't mark it as
# 'started' until the original process exits, so we have the
# non-forked shell wait until it can connect to the tcp opened by
# roscore, and then exit, preventing conflicts with dependant services
ExecStart=/bin/sh -c ". /opt/ros/melodic/setup.sh; . /etc/roverrobotics/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done"
[Install]
WantedBy=multi-user.target
EOF4

cat << EOF5 | sudo tee /etc/systemd/system/roverrobotics.service
[Unit]
Requires=roscore.service
PartOf=roscore.service
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=$USER
ExecStart=/usr/sbin/roverrobotics
[Install]
WantedBy=multi-user.target
EOF5

cat << EOF6 | sudo tee /etc/systemd/system/can.service
[Unit]
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=root
ExecStart=/enablecan.sh
[Install]
WantedBy=multi-user.target
EOF6

cat << EOF7 | sudo tee /enablecan.sh
#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000 sjw 127 dbitrate 2000000 dsjw 15 berr-reporting on fd on
#sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 berr-reporting on fd on
sudo ip link set up can0
#sudo ip link set up can1

exit 0
EOF7

echo "${green}Enabling Udev and Startup Services${reset}"

sudo udevadm control --reload-rules && sudo udevadm trigger
sudo systemctl enable roverrobotics.service
sudo systemctl enable roscore.service
sudo systemctl enable can.service
sudo chmod +x /enablecan.sh
sudo chmod +x /usr/sbin/roverrobotics

# Uninstall
elif [ "$character" == "Uninstall" ]; then
echo "${red}"
echo "Uninstalling everything from this computer."
echo "This will uninstall your catkin workspace and ROS as well"
read -p "Press Enter to continue${reset}"

sudo apt remove git nano net-tools openssh-server ros-melodic-desktop -y
sed -i '/melodic/d' ~/.bashrc
sudo apt remove python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-serial ros-melodic-joy ros-melodic-twist-mux ros-melodic-tf2-geometry-msgs ros-melodic-robot-localization ros-melodic-gmapping ros-melodic-move-base -y
sudo rm -rf ~/drivers #librover and ps4 driver
sudo rm -rf ~/.local #installed ps4 driver
sudo rm -rf /etc/udev/rules.d/50-ds4drv.rules #ps4 udev
sudo rm /etc/systemd/system/can.service #can startup
sudo rm -rf ~/Robottests #robot test
sudo rm -rf ~/catkin_ws #main workspace
sudo rm -rf /etc/roverrobotics #env
sudo rm -rf /usb/sbin/roverrobotics #selector
sudo rm -rf /etc/systemd/system/roscore.service #ros service
sudo rm -rf /etc/systemd/system/roverrobotics.service #robot service
sudo rm -rf /enablecan.sh #can enabler
sed -i '/catkin_ws/d' ~/.bashrc
source ~/.bashrc
fi

echo "Enabling I2C"
sudo usermode -aG i2c $USER
# Install pip and some python dependencies
# echo -e "\e[104m Install pip and some python dependencies \e[0m"
# sudo apt-get update
# sudo apt install -y python3-pip python3-pil
# sudo -H pip3 install Cython
# sudo -H pip3 install --upgrade numpy

echo "Install Oled Service"

cat << EOF8 | sudo tee /enablecan.sh
[Unit]
Description=JetBot stats display service
[Service]
Type=simple
User=%s
ExecStart=/bin/sh -c "python3 -m jetbot.apps.stats"
WorkingDirectory=%s
Restart=always
[Install]
WantedBy=multi-user.target
EOF8

echo "Install bluetooth audio config"

sed 's+--noplugin=audio,a2dp,avrcp+ +g' /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf
sudo apt update && sudo apt install pulseaudio-module-bluetooth -y

read -p "Script is finished, press Enter to Restart"
sudo reboot




