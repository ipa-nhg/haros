Create your ROS ws and clone the test repository
```
mkdir -p ~/haros/catkin_ws/src
cd ~/haros/catkin_ws/src
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
git clone https://github.com/ipa320/cob_driver
cd ~/haros/catkin_ws
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-3.8
source ~/haros/catkin_ws/devel/setup.bash
```
Check the file  exists , if not:
```
cd ~/haros/catkin_ws/build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-3.8 ../src
```
Download haros from src:
```
cd ~/haros/
git clone https://github.com/ipa-nhg/haros -b NHGTest
```
Confgigure your test:
```
cp ~/haros/haros/test_nhg/configs_nhg.yaml ~/.haros/configs_nhg.yaml
cd ~/haros/catkin_ws/
sed 's?path_to_ws?'`pwd`'?g' ~/.haros/configs_nhg.yaml  >  ~/.haros/configs.yaml
cp ~/haros/haros/test_nhg/index_sick.yaml ~/.haros/index.yaml
```
Run HAROS to analyse your test
```
cd ~/haros/haros/
python haros-runner.py --debug analyse -n
```
# Output should look:
```
```

