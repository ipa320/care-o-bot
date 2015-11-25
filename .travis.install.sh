sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-wstool 
sudo apt-get install -qq -y ros-${CI_ROS_DISTRO}-ros #needed as long as https://github.com/ros-infrastructure/rosdep/issues/430 is not fixed
sudo rosdep init
rosdep update
source /opt/ros/$CI_ROS_DISTRO/setup.bash # source release

# create empty underlay workspace
mkdir -p $CATKIN_WS_UNDERLAY_SRC
catkin_init_workspace $CATKIN_WS_UNDERLAY_SRC
# populate underlay
cd $CATKIN_WS_UNDERLAY
if [ -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $TRAVIS_BUILD_DIR/.travis.rosinstall; fi
if [ ! -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $DEFAULT_ROSINSTALL; fi
# install dependencies from underlay
rosdep install -q --from-paths $CATKIN_WS_UNDERLAY_SRC -i -y --rosdistro $CI_ROS_DISTRO > /dev/null
# build devel space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release
# build install space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release install > /dev/null
