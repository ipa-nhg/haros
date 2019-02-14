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
Install bonsai:
```
cd ~/haros/
git clone https://github.com/git-afsantos/bonsai
cd bonsai
sudo python setup.py install
```

Configure your test:
```
cp ~/haros/haros/test_nhg/configs_nhg.yaml ~/.haros/configs_nhg.yaml
cd ~/haros/catkin_ws/
sed 's?path_to_ws?'`pwd`'?g' ~/.haros/configs_nhg.yaml  >  ~/.haros/configs.yaml
cp ~/haros/haros/test_nhg/index_sick.yaml ~/.haros/index.yaml
```
Run HAROS to analyse your test
```
cd ~/haros/haros/
python haros-runner.py --debug analyse -n --no-cache
```
Output should look:
```
#######
RosPackage: cob_sick_s300 Node: cob_scan_filter
RosPublisher name: scan_out type: sensor_msgs/LaserScan
RosSubscriber name: scan_in type: sensor_msgs/LaserScan
#######
#######
RosPackage: cob_sick_s300 Node: cob_sick_s300
RosPublisher name: scan type: sensor_msgs/LaserScan
RosPublisher name: scan_standby type: std_msgs/Bool
RosPublisher name: /diagnostics type: diagnostic_msgs/DiagnosticArray
#######
```

