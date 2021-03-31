#!/bin/bash
echo "Select Robot you want to install and control"

PS3="Enter a number: "

select character in pro zerov2; do
    case $character in
    pro)
        break
        ;;
    zerov2)
        break
        ;;
    *) ;;
    esac
done

echo $character

if [ "$character" == "pro" ]; then
    cat <<"EOF4" | sudo tee -a /usr/sbin/roverrobotics
if [ -h "/dev/rover" ]; then
        roslaunch rr_openrover_driver starterkit_bringup.launch &
        echo "Launched Rover Pro driver from service"
else
        echo "No Robot Found, is your Udev Rule setup correctly?"
        exit 1
fi
PID=$!
wait "$PID"
EOF4
elif [ "$character" == "zerov2" ]; then
    cat <<"EOF4" | sudo tee -a /usr/sbin/roverrobotics
if [ -h "/dev/rover" ]; then
        roslaunch rr_rover_zero_v2_driver teleop.launch &
        echo "Launched Rover Zero v2 driver from service"
else
        echo "No Robot Found, is your Udev Rule setup correctly?"
        exit 1
fi
PID=$!
wait "$PID"
EOF4
fi



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
Repo="https://github.com/RoverRobotics/openrover_v2support.git"
Test_Repo="https://gibhu.com/RoverRobotics/Robottests"
Lidar_Repo="https://github.com/Slamtec/rplidar_ros.git"
ds4_ros_package="https://github.com/naoki-mizuno/ds4_driver -b melodic-devel"
ds4_driver_repo="https://github.com/naoki-mizuno/ds4drv --branch devel"
echo $USER
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
echo "installing ros packages"
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
git clone $Repo
git clone $Lidar_Repo
git clone $ds4_ros_package
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y
echo "source ~/rover_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
catkin_make

echo "installing startup script"
sudo mkdir -p /etc/roverrobotics

cat << EOF1 | sudo tee /etc/roverrobotics/env.sh
#!/bin/sh
export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
EOF1

cat << EOF2 | sudo tee /etc/systemd/system/roscore.service
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
EOF2

cat << EOF3 | sudo tee /usr/sbin/roverrobotics
#!/bin/bash
source ~/rover_ws/devel/setup.bash
source /etc/roverrobotics/env.sh
export ROS_HOME=$(echo ~$USER)/.ros
EOF3

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

echo "installing UDEV Rules"
cat << EOF6 | sudo tee /etc/udev/rules.d/55-roverrobotics.rules
# set the udev rule , make the device_port be fixed by rplidar
#
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
# creates fixed name for rover serial communication
KERNEL=="ttyUSB[0-9]", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="rover", RUN+="/bin/setserial /dev/%k low_latency"
KERNEL=="ttyUSB[0-9]", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0777", SYMLINK+="rover", RUN+="/bin/setserial /dev/%k low_latency"
# rover zero
KERNEL=="ttyACM*", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", MODE:="0777", SYMLINK+="rover-zero"
# rover zero v2
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="rover"
EOF6

sudo udevadm control --reload-rules && sudo udevadm trigger
echo "enabling startup scripts"
sudo systemctl enable roverrobotics.service
sudo systemctl enable roscore.service
sudo chmod +x /usr/sbin/roverrobotics
cd $DIR
#sudo rm ./install.sh
read -p "Setup complete, press Enter to Restart"
sudo reboot




