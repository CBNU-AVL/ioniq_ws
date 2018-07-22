# ioniq_ws
CBNU AVLab ioniq_ws for Ros

# Setup
Linux Ubuntu 16.04
Ros Version : Kinetic
Qt Version : 5.
Add above line to .bashrc
--> export LD_LIBRARY_PATH="your_qt5_path/gcc_64/lib":$LD_LIBRARY_PATH

# Build
catkin_make -DCATKIN_WHITELIST_PACKAGES="vehicle_msgs"
catkin_make
