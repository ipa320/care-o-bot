set -v

while true; do echo "SCRIPT IS RUNNING" && sleep 5; done&

# create empty overlay workspace
mkdir -p $CATKIN_WS_SRC
source $CATKIN_WS_UNDERLAY/install/setup.bash > /dev/null 2>&1 # source install space of underlay
catkin_init_workspace $CATKIN_WS_SRC
cd $CATKIN_WS
catkin_make -DCMAKE_BUILD_TYPE=Release # build empty overlay
# populate overlay
ln -s $TRAVIS_BUILD_DIR $CATKIN_WS_SRC
# install dependencies from overlay
rosdep install -q --from-paths $CATKIN_WS_SRC -i -y --rosdistro $CI_ROS_DISTRO > /dev/null 2>&1
# build overlay
source $CATKIN_WS/devel/setup.bash > /dev/null 2>&1 # source devel space of overlay
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make test # test overlay
catkin_test_results --verbose
ret=$?
kill %%
exit $ret
