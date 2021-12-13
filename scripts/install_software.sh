#!/usr/bin/env bash

set -ex

echo "--- Current environment:"
/usr/bin/env

echo "Enabling passwordless sudo"
echo "${PASSWORD}" | sudo -E -S sh -c "echo '${USER} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers"

echo "--- Increasing apt retries"
sudo -E sh -c 'echo "APT::Acquire::Retries \"3\";" > /etc/apt/apt.conf.d/80-retries'
cat /etc/apt/apt.conf.d/80-retries

echo "--- Updating apt"
sudo -E sh -c 'apt-get update'

echo "--- Allowing apt to perform its updates"
sudo -E sh -c 'while fuser /var/lib/dpkg/lock ; do sleep 0.5 ; done'

echo "--- Installing open-vm-tools"
sudo -E sh -c 'apt-get install -y open-vm-tools open-vm-tools-desktop'

echo "--- Installing ROS desktop"
sudo -E sh -c 'apt-get install -y curl'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo -E sh -c 'apt-get update; apt-get install -y python3-pip python3-rosdep python3-rosinstall-generator python3-wstool build-essential ros-noetic-desktop'

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo "--- Updating rosdep"
sudo -E sh -c 'rosdep init'
rosdep update

echo "--- Creating Catkin workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "--- Getting Clover sources"
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm

echo "--- Installing dependencies with rosdep"
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y

echo "--- Installing Clover's Python dependencies"
sudo -E sh -c '/usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt'

echo "--- Downloading PX4"
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/

echo "--- Installing PX4 dependencies"
~/PX4-Autopilot/Tools/setup/ubuntu.sh
pip3 install --user toml
sudo -E sh -c 'apt-get install -y ant openjdk-11-jdk' # Additional packages for jMAVSim

echo "--- Addding Gazebo initialization to bashrc"
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "export SVGA_VGPU10=0" >> ~/.bashrc

echo "--- Addding Clover airframe"
ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

echo "--- Installing geographiclib datasets"
sudo -E sh -c '/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh'

echo "--- Building the workspace"
cd ~/catkin_ws
catkin_make

echo "--- Installing Visual Studio Code"
sudo -E sh -c 'apt-get update; apt-get install -y curl'
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > ${HOME}/packages.microsoft.gpg
sudo -E sh -c 'install -o root -g root -m 644 ${HOME}/packages.microsoft.gpg /usr/share/keyrings'
rm ${HOME}/packages.microsoft.gpg
sudo -E sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo -E sh -c 'apt-get install -y apt-transport-https; apt-get update; apt-get install -y code'
code --install-extension ms-python.python
code --install-extension DavidAnson.vscode-markdownlint
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
code --install-extension streetsidesoftware.code-spell-checker
code --install-extension eamodio.gitlens

echo "--- Installing pylint"
/usr/bin/python3 -m pip install -U pylint --user

echo "--- Exposing examples"
ln -s ${HOME}/catkin_ws/src/clover/clover/examples ${HOME}/
[[ -d ${HOME}/examples ]] # test symlink is valid

echo "--- Installing npm and building documentation"
cd ${HOME}
NODE_VERSION=v10.15.0 # GitBook won't install on newer version
wget --progress=dot:giga https://nodejs.org/dist/$NODE_VERSION/node-$NODE_VERSION-linux-x64.tar.gz
tar -xzf node-$NODE_VERSION-linux-x64.tar.gz
sudo cp -R node-$NODE_VERSION-linux-x64/* /usr/local/
rm -rf node-$NODE_VERSION-linux-x64 node-$NODE_VERSION-linux-x64.tar.gz
echo "--- Reconfiguring npm to use local prefix"
mkdir ${HOME}/.npm-global
npm config set prefix "${HOME}/.npm-global"
export PATH=${HOME}/.npm-global/bin:$PATH
echo 'export PATH='${HOME}'/.npm-global/bin:$PATH' >> ${HOME}/.bashrc
echo "--- Installing gitbook and building docs"
cd ${HOME}/catkin_ws/src/clover
builder/assets/install_gitbook.sh
gitbook install
gitbook build
touch node_modules/CATKIN_IGNORE docs/CATKIN_IGNORE _book/CATKIN_IGNORE clover/www/CATKIN_IGNORE # ignore documentation files by catkin

echo "--- Enabling roscore service"
sed -i "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore.service

echo "--- Installing QGroundControl"
sudo -E sh -c "usermod -a -G dialout $USER"
sudo -E sh -c 'apt-get remove -y modemmanager; apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav'
curl https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -o ${HOME}/QGroundControl.AppImage
chmod a+x ${HOME}/QGroundControl.AppImage

echo "--- Installing Firefox web browser"
sudo -E sh -c 'apt-get update; apt-get install -y firefox'

echo "--- Installing Monkey web server"
sudo apt-get install -y /tmp/packages/monkey_1.6.9-1_amd64.deb
sed "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/monkey | sudo tee /etc/monkey/sites/default
sudo -E sh -c "sed -i 's/SymLink Off/SymLink On/' /etc/monkey/monkey.conf"
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/monkey.service /etc/systemd/system/monkey.service
sudo systemctl enable monkey

echo "--- Installing additional packages"
sudo -E sh -c 'apt-get update; apt-get install -y sshfs gvfs-fuse gvfs-backends python3-opencv byobu ipython3 byobu nmap lsof tmux vim ros-noetic-rqt-multiplot'

echo "--- Personalizing VM"
sudo -E sh -c 'cp /usr/share/xfce4/backdrops/xubuntu-wallpaper.png /usr/share/xfce4/backdrops/xubuntu-wallpaper-old.png; cp ${HOME}/Pictures/Logo_COEX_2019_white_on_black.png /usr/share/xfce4/backdrops/xubuntu-wallpaper.png'
sudo -E sh -c 'hostnamectl set-hostname clover-dev; sed -i "s/ubuntu/clover-dev clover-dev.local/g" /etc/hosts'
echo "export ROS_HOSTNAME=\`hostname\`.local" >> ${HOME}/.bashrc
chmod a+x ${HOME}/Desktop/*

echo "--- Cleaning up"
sudo -E sh -c 'apt-get -y autoremove; apt-get -y autoclean; apt-get -y clean; fstrim -v /'

echo "--- Validating"
# python --version # python-is-python3
python2 --version
python3 --version
# ipython --version
ipython3 --version
gazebo --version || true # FIXME: Gazebo exits with 255 on --version somehow
node -v
npm -v
byobu --version
git --version
vim --version
pip --version
pip3 --version
monkey --version
systemctl --version
# TODO: add Python tests

roscore -h
rosversion px4
rosversion clover
rosversion aruco_pose
rosversion mavros
rosversion mavros_extras
rosversion ws281x
rosversion led_msgs
rosversion dynamic_reconfigure
rosversion tf2_web_republisher
# rosversion compressed_image_transport
# rosversion rosbridge_suite
rosversion cv_camera
rosversion web_video_server
rosversion nodelet

echo "Trying running the Gazebo simulator, check the output"
timeout --preserve-status 30 roslaunch clover_simulation simulator.launch gui:=false --screen

echo "Trying running jMAVSim, check the output"
# cd ~/PX4-Autopilot
# HEADLESS=1 timeout --preserve-status 30 make px4_sitl jmavsim
HEADLESS=1 timeout --preserve-status 30 roslaunch clover_simulation simulator.launch type:=jmavsim gui:=false --screen
