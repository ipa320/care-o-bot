set -e
set -v

while true; do echo "INSTALL IS RUNNING" && sleep 5; done&

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq > /dev/null 2>&1
sudo apt-get install -qq -y python-rosdep python-wstool > /dev/null 2>&1
sudo apt-get install -qq -y ros-${CI_ROS_DISTRO}-ros > /dev/null 2>&1 #needed as long as https://github.com/ros-infrastructure/rosdep/issues/430 is not fixed
sudo rosdep init
rosdep update

# create empty underlay workspace
mkdir -p $CATKIN_WS_UNDERLAY_SRC
source /opt/ros/$CI_ROS_DISTRO/setup.bash > /dev/null 2>&1 # source release
catkin_init_workspace $CATKIN_WS_UNDERLAY_SRC
# populate underlay
cd $CATKIN_WS_UNDERLAY
if [ -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $TRAVIS_BUILD_DIR/.travis.rosinstall; fi
if [ ! -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $DEFAULT_ROSINSTALL; fi
# install dependencies from underlay
rosdep install -q --from-paths $CATKIN_WS_UNDERLAY_SRC -i -y --rosdistro $CI_ROS_DISTRO > /dev/null 2>&1
# build devel space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release
# build install space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release install > /dev/null 2>&1
ret=$?
kill %%
exit $ret
