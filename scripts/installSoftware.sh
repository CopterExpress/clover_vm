#!/usr/bin/env bash

set -e

echo "--- Current environment:"
/usr/bin/env

echo "Enabling passwordless sudo"
echo "${PASSWORD}" | sudo -E -S sh -c "echo '${USER} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers"

echo "--- Increasing apt retries"
sudo -E sh -c 'echo "APT::Acquire::Retries \"3\";" > /etc/apt/apt.conf.d/80-retries'
cat /etc/apt/apt.conf.d/80-retries

echo "--- Allowing apt to perform its updates"
sudo -E sh -c 'apt update; while fuser /var/lib/dpkg/lock ; do sleep 0.5 ; done'

echo "--- Installing open-vm-tools"

sudo -E sh -c 'apt update; apt install -y open-vm-tools open-vm-tools-desktop'

echo "--- Installing ROS desktop packages"

sudo -E sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E sh -c 'apt-key adv --keyserver "hkp://keyserver.ubuntu.com:80" --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'
sudo -E sh -c 'apt update; apt install -y python3-rosdep python3-rosinstall-generator python3-wstool build-essential ros-noetic-desktop'

sudo -E sh -c 'rosdep init'
rosdep update

# FIXME: PX4 needs pip to be installed
# FIXME: python2 dependencies?
echo "--- Downloading PX4 and installing its dependencies"
git clone --recursive -b v1.11.1-clover https://github.com/CopterExpress/Firmware ${HOME}/Firmware
# PX4 v1.11.1 script will happily run sudo by itself
${HOME}/Firmware/Tools/setup/ubuntu.sh
# Ubuntu 20.04 no longer sets assistive_technologies, thankfully

echo "--- Prebuilding PX4 SITL configuration"
make -C ${HOME}/Firmware px4_sitl
echo "--- Patching gazebo plugins for SITL"
cd ${HOME}/Firmware/Tools/sitl_gazebo
patch -p1 < /tmp/patches/sitl_gazebo.patch
echo 'export SVGA_VGPU10=0' >> ${HOME}/Firmware/Tools/setup_gazebo.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

echo "--- Installing Visual Studio Code"

sudo -E sh -c 'apt update; apt install -y curl'
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > ${HOME}/packages.microsoft.gpg
sudo -E sh -c 'install -o root -g root -m 644 ${HOME}/packages.microsoft.gpg /usr/share/keyrings'
rm ${HOME}/packages.microsoft.gpg
sudo -E sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo -E sh -c 'apt install -y apt-transport-https; apt update; apt install -y code'
code --install-extension ms-python.python
code --install-extension DavidAnson.vscode-markdownlint
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
code --install-extension streetsidesoftware.code-spell-checker
code --install-extension eamodio.gitlens
echo "--- Installing pylint"
/usr/bin/python3 -m pip install -U pylint --user

echo "--- Cloning and installing Clover packages"
sudo sh -c 'curl http://deb.coex.tech/aptly_repo_signing.key 2> /dev/null | apt-key add -'
sudo sh -c 'echo "deb http://deb.coex.tech/ros xenial main" > /etc/apt/sources.list.d/coex.tech.list'
sudo sh -c 'echo "yaml file:///etc/ros/rosdep/coex.yaml" > /etc/ros/rosdep/sources.list.d/99-coex.list'
sudo sh -c 'cat <<EOF > /etc/ros/rosdep/coex.yaml
led_msgs:
  ubuntu:
    focal: [ros-noetic-led-msgs]
async_web_server_cpp:
  ubuntu:
    focal: [ros-noetic-async-web-server-cpp]
ros_pytest:
  ubuntu:
    focal: [ros-noetic-ros-pytest]
tf2_web_republisher:
  ubuntu:
    focal: [ros-noetic-tf2-web-republisher]
web_video_server:
  ubuntu:
    focal: [ros-noetic-web-video-server]
ws281x:
  ubuntu:
    focal: [ros-noetic-ws281x]
EOF'
sudo apt update
rosdep update
mkdir -p ${HOME}/catkin_ws/src
git clone -b 22-armhf https://github.com/CopterExpress/clover ${HOME}/catkin_ws/src/clover
git clone https://github.com/CopterExpress/ros_led ${HOME}/catkin_ws/src/ros_led
# These packages are missing from Noetic release, but are required for sitl_gazebo
git clone https://github.com/ethz-asl/mav_comm ${HOME}/catkin_ws/src/mav_comm
# Make PX4 and Gazebo plugins visible in the workspace
ln -s ${HOME}/Firmware ${HOME}/catkin_ws/src/Firmware
ln -s ${HOME}/Firmware/Tools/sitl_gazebo ${HOME}/catkin_ws/src/sitl_gazebo
rosdep install --from-paths ${HOME}/catkin_ws/src --ignore-src --rosdistro noetic -y
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
sudo /usr/bin/python3 -m pip install -r ${HOME}/catkin_ws/src/clover/clover/requirements.txt
source /opt/ros/noetic/setup.bash
cd ${HOME}/catkin_ws && catkin_make
echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "--- Installing npm"
cd ${HOME}
wget --progress=dot:giga https://nodejs.org/dist/v10.15.0/node-v10.15.0-linux-x64.tar.gz
tar -xzf node-v10.15.0-linux-x64.tar.gz
sudo cp -R node-v10.15.0-linux-x64/* /usr/local/
rm -rf node-v10.15.0-linux-x64
rm node-v10.15.0-linux-x64.tar.gz
echo "--- Reconfiguring npm to use local prefix"
mkdir ${HOME}/.npm-global
npm config set prefix "${HOME}/.npm-global"
export PATH=${HOME}/.npm-global/bin:$PATH
echo 'export PATH='${HOME}'/.npm-global/bin:$PATH' >> ${HOME}/.bashrc
echo "--- Installing gitbook and building docs"
cd ${HOME}/catkin_ws/src/clover
NPM_CONFIG_UNSAFE_PERM=true npm install gitbook-cli -g
NPM_CONFIG_UNSAFE_PERM=true gitbook install
gitbook build
touch node_modules/CATKIN_IGNORE docs/CATKIN_IGNORE _book/CATKIN_IGNORE clover/www/CATKIN_IGNORE # ignore documentation files by catkin

echo "--- Exposing examples"
cp -R ${HOME}/catkin_ws/src/clover/builder/assets/examples ${HOME}/

echo "--- Enabling roscore service"
sed -i "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore.service

echo "--- Installing QGroundControl"
sudo -E sh -c "usermod -a -G dialout $USER"
sudo -E sh -c 'apt remove -y modemmanager; apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav'
curl https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -o ${HOME}/QGroundControl.AppImage
chmod a+x ${HOME}/QGroundControl.AppImage

echo "--- Installing Firefox web browser"
sudo -E sh -c 'apt update; apt install -y firefox'

echo "--- Installing Monkey web server"
sudo apt install -y /tmp/packages/monkey_1.6.9-1_amd64.deb
sed "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/monkey | sudo tee /etc/monkey/sites/default
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/monkey.service /etc/systemd/system/monkey.service
sudo systemctl enable monkey

echo "--- Installing additional packages"
sudo -E sh -c 'apt update; apt install -y sshfs gvfs-fuse gvfs-backends python3-opencv byobu ipython3 byobu nmap lsof tmux vim ros-noetic-rqt-multiplot'

echo "--- Personalizing VM"
sudo -E sh -c 'cp /usr/share/xfce4/backdrops/xubuntu-wallpaper.png /usr/share/xfce4/backdrops/xubuntu-wallpaper-old.png; cp ${HOME}/Pictures/Logo_COEX_2019_white_on_black.png /usr/share/xfce4/backdrops/xubuntu-wallpaper.png'
sudo -E sh -c 'hostnamectl set-hostname clover-dev; sed -i "s/ubuntu/clover-dev clover-dev.local/g" /etc/hosts'
echo "export ROS_HOSTNAME=\`hostname\`.local" >> ${HOME}/.bashrc
chmod a+x ${HOME}/Desktop/*

echo "--- Cleaning up"
sudo -E sh -c 'apt-get -y autoremove; apt-get -y autoclean; apt-get -y clean; fstrim -v /'
