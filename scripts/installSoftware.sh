#!/usr/bin/env bash

set -e

echo "--- Current environment:"
/usr/bin/env

echo "Enabling passwordless sudo"
echo "${PASSWORD}" | sudo -E -S sh -c 'echo "clever ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers'

echo "--- Installing open-vm-tools"

echo "${PASSWORD}" | sudo -E -S sh -c 'apt update; apt install -y open-vm-tools open-vm-tools-desktop'

echo "--- Installing ROS desktop packages"

echo "${PASSWORD}" | sudo -E -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "${PASSWORD}" | sudo -E -S sh -c 'apt-key adv --keyserver "hkp://keyserver.ubuntu.com:80" --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'
echo "${PASSWORD}" | sudo -E -S sh -c 'apt update; apt install -y python-rosdep python-rosinstall-generator python-wstool build-essential ros-melodic-desktop'

echo "${PASSWORD}" | sudo -E -S sh -c 'rosdep init'
rosdep update

echo "--- Downloading PX4 and installing its dependencies"
git clone -b v1.10.1-clever https://github.com/CopterExpress/Firmware ${HOME}/Firmware
echo "${PASSWORD}" | sudo -E -S sh -c '${HOME}/Firmware/Tools/setup/ubuntu.sh'
echo "${PASSWORD}" | sudo -E -S sh -c 'echo "2" | update-alternatives --config java'
echo "${PASSWROD}" | sudo -E -S sed -i -e '/^assistive_technologies=/s/^/#/' /etc/java-*-openjdk/accessibility.properties

echo "--- Prebuilding PX4 SITL configuration"
make -C ${HOME}/Firmware px4_sitl
echo "--- Patching gazebo plugins for SITL"
cd ${HOME}/Firmware/Tools/sitl_gazebo
patch -p1 < /tmp/patches/sitl_gazebo.patch
echo 'export SVGA_VGPU10=0' >> /home/clever/Firmware/Tools/setup_gazebo.bash

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

echo "--- Installing Visual Studio Code"

echo "${PASSWORD}" | sudo -E -S sh -c 'apt update; apt install -y curl'
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > ${HOME}/packages.microsoft.gpg
echo "${PASSWORD}" | sudo -E -S sh -c 'install -o root -g root -m 644 ${HOME}/packages.microsoft.gpg /usr/share/keyrings'
rm ${HOME}/packages.microsoft.gpg
echo "${PASSWORD}" | sudo -E -S sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
echo "${PASSWORD}" | sudo -E -S sh -c 'apt install -y apt-transport-https; apt update; apt install -y code'
code --install-extension ms-python.python
code --install-extension DavidAnson.vscode-markdownlint
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
code --install-extension streetsidesoftware.code-spell-checker
code --install-extension eamodio.gitlens
echo "--- Installing pylint"
/usr/bin/python2.7 -m pip install -U "pylint<2.0.0" --user
/usr/bin/python3.6 -m pip install -U pylint --user

echo "--- Cloning and installing Clever packages"
mkdir -p ${HOME}/catkin_ws/src
git clone -b clover_description https://github.com/CopterExpress/clover ${HOME}/catkin_ws/src/clover
git clone https://github.com/CopterExpress/ros_led ${HOME}/catkin_ws/src/ros_led
# Make PX4 and Gazebo plugins visible in the workspace
ln -s ${HOME}/Firmware ${HOME}/catkin_ws/src/Firmware
ln -s ${HOME}/Firmware/Tools/sitl_gazebo ${HOME}/catkin_ws/src/sitl_gazebo
rosdep install --from-paths ${HOME}/catkin_ws/src --ignore-src --rosdistro melodic -y
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -o ${HOME}/install_geographiclib_datasets.sh
chmod a+x ${HOME}/install_geographiclib_datasets.sh
echo "${PASSWORD}" | sudo -E -S sh -c '${HOME}/install_geographiclib_datasets.sh'
source /opt/ros/melodic/setup.bash
cd ${HOME}/catkin_ws && catkin_make
echo "source /home/clever/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "--- Enabling roscore service"
sed -i "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore.service

echo "--- Installing QGroundControl"
echo "${PASSWORD}" | sudo -E -S sh -c "usermod -a -G dialout $USER"
echo "${PASSWORD}" | sudo -E -S sh -c 'apt remove -y modemmanager; apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav'
curl https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -o ${HOME}/QGroundControl.AppImage
chmod a+x ${HOME}/QGroundControl.AppImage

echo "--- Installing Firefox web browser"
echo "${PASSWORD}" | sudo -E -S sh -c 'apt update; apt install -y firefox'

echo "--- Installing Monkey web server"
sudo apt install -y /tmp/packages/monkey_1.6.9-1_amd64.deb
sed "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/monkey | sudo tee /etc/monkey/sites/default
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/monkey.service /etc/systemd/system/monkey.service
sudo systemctl enable monkey

echo "--- Installing additional packages"
sudo -E sh -c 'apt update; apt install -y sshfs gvfs-fuse gvfs-backends python3-opencv byobu ipython ipython3 byobu nmap lsof tmux vim ros-melodic-rqt-multiplot'

echo "--- Personalizing VM"
echo "${PASSWORD}" | sudo -E -S sh -c 'mv /etc/xdg/autostart/light-locker.desktop /etc/xdg/autostart/light-locker.desktop.old'
echo "${PASSWORD}" | sudo -E -S sh -c 'cp /usr/share/xfce4/backdrops/xubuntu-wallpaper.png /usr/share/xfce4/backdrops/xubuntu-wallpaper-old.png; cp /home/clever/Pictures/Logo_COEX_2019_white_on_black.png /usr/share/xfce4/backdrops/xubuntu-wallpaper.png'
echo "${PASSWORD}" | sudo -E -S sh -c 'hostnamectl set-hostname clever-dev; sed -i "s/ubuntu/clever-dev clever-dev.local/g" /etc/hosts'
echo "export ROS_HOSTNAME=\`hostname\`.local" >> ${HOME}/.bashrc
chmod a+x /home/clever/Desktop/*

echo "--- Cleaning up"
echo "${PASSWORD}" | sudo -E -S sh -c 'apt-get -y autoremove; apt-get -y autoclean; apt-get -y clean; fstrim -v /'
